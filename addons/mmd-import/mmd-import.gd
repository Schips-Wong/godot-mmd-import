@tool
extends EditorPlugin

const TOOL_RETARGET := "MMD: 重映射动画库 (根位移)"
const TOOL_BAKE := "MMD: 烘焙动画库 (自包含)"
const AnimationRetarget = preload("res://addons/mmd-import/mmd_animation_retarget.gd")
const PoseBaker = preload("res://addons/mmd-import/mmd_pose_baker.gd")

var pmxImporter: EditorSceneFormatImporter
var vmdImporter: EditorImportPlugin


func _enter_tree() -> void:
	# Initialization of the plugin goes here.
	print("MMD Import plugin activated.")
	pmxImporter = PMXImporter.new()
	add_scene_format_importer_plugin(pmxImporter)
	vmdImporter = VMDImporter.new()
	add_import_plugin(vmdImporter, false)
	add_tool_menu_item(TOOL_RETARGET, _retarget_animation_library)
	add_tool_menu_item(TOOL_BAKE, _bake_animation_libraries)
	pass


func _exit_tree() -> void:
	# Clean-up of the plugin goes here.
	remove_tool_menu_item(TOOL_BAKE)
	remove_tool_menu_item(TOOL_RETARGET)
	remove_import_plugin(vmdImporter)
	vmdImporter = null
	remove_scene_format_importer_plugin(pmxImporter)
	pmxImporter = null
	pass


## Rewrites the AnimationLibrary of the edited scene so that the tracks of the
## bones SkeletonProfileHumanoid does not know (root motion, attachments) drive
## the MMD skeleton. Select the source model scene in the FileSystem dock first.
func _retarget_animation_library() -> void:
	var editor := get_editor_interface()
	var scene := editor.get_edited_scene_root()
	if scene == null:
		_show_message("请先打开目标场景（例如 remap-test.tscn）。")
		return
	var player := _find_animation_player(scene)
	var target := _find_skeleton(scene)
	if player == null or target == null:
		_show_message("当前场景里找不到 AnimationPlayer 或 Skeleton3D。")
		return

	var source_path := ""
	for path in editor.get_selected_paths():
		if path.get_extension() in ["tscn", "scn", "gltf", "glb"]:
			source_path = path
			break
	if source_path.is_empty():
		_show_message("请先在「文件系统」面板中选中源模型场景（例如 P0-remap-with-an.tscn），再执行本命令。")
		return

	var source_scene := load(source_path) as PackedScene
	if source_scene == null:
		_show_message("无法加载源场景：%s" % source_path)
		return
	var source_root := source_scene.instantiate()
	var source := _find_skeleton(source_root)
	if source == null:
		source_root.free()
		_show_message("源场景里没有 Skeleton3D：%s" % source_path)
		return

	var report := []
	for library_name in player.get_animation_library_list():
		var library := player.get_animation_library(library_name)
		if library == null or not AnimationRetarget.can_remap(library):
			continue
		var output_path := _output_path(library.resource_path, "_remapped")
		if output_path.is_empty():
			report.append("%s: 内嵌动画库无法保存，已跳过" % library_name)
			continue
		var remapped := AnimationRetarget.remap_library(source, target, library)
		var error := ResourceSaver.save(remapped, output_path)
		if error != OK:
			report.append("%s: 保存失败 (%d)" % [library_name, error])
			continue
		player.remove_animation_library(library_name)
		player.add_animation_library(library_name, load(output_path))
		report.append("%s -> %s" % [library_name, output_path])

	source_root.free()
	editor.mark_scene_as_unsaved()
	_show_message("重映射完成：\n" + "\n".join(report))


## Bakes the MMD pose rules (IK and 付与) into every AnimationLibrary of the
## edited scene, whatever its source: imported VMD, a retargeted glTF library or
## a hand made animation. The baked libraries are saved next to the source and
## replaced in the edited scene, so the motions play identically without any
## runtime solver (exported game, glTF export, third party projects).
func _bake_animation_libraries() -> void:
	var editor := get_editor_interface()
	var scene := editor.get_edited_scene_root()
	if scene == null:
		_show_message("请先打开目标场景。")
		return
	var player := _find_animation_player(scene)
	var target := _find_skeleton(scene)
	if player == null or target == null:
		_show_message("当前场景里找不到 AnimationPlayer 或 Skeleton3D。")
		return
	if not target.has_meta("mmd_bone_data"):
		_show_message("当前 Skeleton3D 不是 MMD 模型（缺少 mmd_bone_data），无法烘焙。")
		return

	var report := []
	for library_name in player.get_animation_library_list():
		var library := player.get_animation_library(library_name)
		if library == null:
			continue
		var output_path := _output_path(library.resource_path, "_baked")
		if output_path.is_empty():
			report.append("%s: 内嵌动画库无法保存，已跳过" % library_name)
			continue
		# The source library stays untouched; the baked copy is what gets saved
		# and played, so running the tool twice does not bake twice.
		var baked := library.duplicate(true) as AnimationLibrary
		var baked_count := PoseBaker.bake_library(target, baked, true)
		if baked_count == 0:
			report.append("%s: 无需烘焙（已烘焙，或没有 IK/付与 规则）" % library_name)
			continue
		var error := ResourceSaver.save(baked, output_path)
		if error != OK:
			report.append("%s: 保存失败 (%d)" % [library_name, error])
			continue
		player.remove_animation_library(library_name)
		player.add_animation_library(library_name, load(output_path))
		report.append("%s: %d 个动画 -> %s" % [library_name, baked_count, output_path])

	editor.mark_scene_as_unsaved()
	_show_message("烘焙完成，动画现在不依赖运行时 solver：\n" + "\n".join(report))


func _output_path(path: String, suffix: String) -> String:
	if path.is_empty():
		return ""
	var base := path.get_basename()
	if base.get_file().ends_with(suffix):
		return path
	return "%s%s.res" % [base, suffix]


func _find_skeleton(root: Node) -> Skeleton3D:
	if root is Skeleton3D:
		return root as Skeleton3D
	var found := root.find_children("*", "Skeleton3D", true, false)
	return found[0] as Skeleton3D if not found.is_empty() else null


func _find_animation_player(root: Node) -> AnimationPlayer:
	if root is AnimationPlayer:
		return root as AnimationPlayer
	var found := root.find_children("*", "AnimationPlayer", true, false)
	return found[0] as AnimationPlayer if not found.is_empty() else null


func _show_message(text: String) -> void:
	print("[MMD] ", text)
	var dialog := AcceptDialog.new()
	dialog.title = "MMD Import"
	dialog.dialog_text = text
	get_editor_interface().get_base_control().add_child(dialog)
	dialog.popup_centered()
	dialog.confirmed.connect(dialog.queue_free)
	dialog.close_requested.connect(dialog.queue_free)
