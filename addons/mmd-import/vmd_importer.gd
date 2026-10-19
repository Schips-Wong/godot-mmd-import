class_name VMDImporter extends EditorImportPlugin

## Imports MMD motion files (.vmd) as an [AnimationLibrary] holding one
## [Animation] with the bone motion of the file.
##
## Godot stores bone poses as absolute local transforms, so the rest pose of the
## skeleton is needed to place the translation keys. The importer therefore
## looks for an already imported .pmx model (the reference model) that shares the
## most bone names with the motion file. Without a reference model only the
## rotation tracks are generated.
##
## IK is precomputed: the reference skeleton is posed frame by frame, the PMX IK
## chains are solved and the results are written into the animation as normal
## rotation keys (see [MMDIKBaker]). The imported motion therefore plays without
## the runtime IK solver.
##
## The track paths are "SkeletonNodePath:BoneName" relative to the player's
## root node, so the library can be played by any [AnimationMixer] whose
## [member AnimationMixer.root_node] points at the node containing the skeleton:
## [codeblock]
## var player: AnimationPlayer = $AnimationPlayer
## player.root_node = player.get_path_to($Model)
## player.add_animation_library("dance", load("res://motions/dance.vmd"))
## player.play("dance/dance")
## [/codeblock]

const OPTION_SCALE := "mmd/scale"
const OPTION_FPS := "mmd/fps"
const OPTION_REFERENCE_MODEL := "mmd/reference_model"
const OPTION_SKELETON_PATH := "mmd/skeleton_path"

const VMD_EXTENSION := "vmd"
const REFERENCE_SEARCH_DEPTH := 2
const REFERENCE_SEARCH_LIMIT := 64
## Runs after scenes, so the reference .pmx models are imported already.
const IMPORT_ORDER := 200


class ReferenceModel:
	var m_root: Node
	var m_skeleton: Skeleton3D
	var m_skeletonNodePath: String
	var m_source: String

	func release() -> void:
		if m_root != null:
			m_root.free()
			m_root = null
		m_skeleton = null


func _get_importer_name() -> String:
	return "mmd.vmd"


func _get_visible_name() -> String:
	return "MMD Motion"


func _get_recognized_extensions() -> PackedStringArray:
	return PackedStringArray([VMD_EXTENSION])


func _get_save_extension() -> String:
	return "res"


func _get_resource_type() -> String:
	return "AnimationLibrary"


func _get_import_order() -> int:
	return IMPORT_ORDER


func _get_preset_count() -> int:
	return 1


func _get_preset_name(_preset_index: int) -> String:
	return "Default"


func _get_import_options(_path: String, _preset_index: int) -> Array[Dictionary]:
	return [
		{
			"name": OPTION_SCALE,
			"default_value": 0.1,
			"property_hint": PROPERTY_HINT_RANGE,
			"hint_string": "0.001,10,0.001",
		},
		{
			"name": OPTION_FPS,
			"default_value": 30.0,
			"property_hint": PROPERTY_HINT_RANGE,
			"hint_string": "1,240,0.001",
		},
		{
			"name": OPTION_REFERENCE_MODEL,
			"default_value": "",
			"property_hint": PROPERTY_HINT_FILE,
			"hint_string": "*.pmx",
		},
		{
			"name": OPTION_SKELETON_PATH,
			"default_value": "",
			"property_hint": PROPERTY_HINT_NONE,
			"hint_string": "",
		},
	]


func _get_option_visibility(_path: String, _option_name: StringName, _options: Dictionary) -> bool:
	return true


func _can_import_threaded() -> bool:
	return false


func _import(source_file: String, save_path: String, options: Dictionary, _platform_variants: Array[String], _gen_files: Array[String]) -> Error:
	var vmd := VMDFile.new()
	if not vmd.setup(source_file):
		push_error("MMD: cannot import %s: %s" % [source_file, vmd.errorString])
		return ERR_PARSE_ERROR

	var scale := float(options.get(OPTION_SCALE, 0.1))
	var fps := float(options.get(OPTION_FPS, 30.0))
	var requested_reference := String(options.get(OPTION_REFERENCE_MODEL, ""))
	var skeleton_path := String(options.get(OPTION_SKELETON_PATH, ""))

	print("MMD: motion '%s' (%s): %d bones, %d bone keys, %d morph keys, %d frames" % [
		vmd.m_header.m_modelName,
		source_file.get_file(),
		vmd.m_boneKeyFrames.size(),
		vmd.get_bone_key_frame_count(),
		vmd.m_morphKeyFrames.size(),
		vmd.get_last_frame(),
	])

	var reference := _resolve_reference(source_file, requested_reference, vmd)
	var skeleton: Skeleton3D = null
	if reference != null:
		skeleton = reference.m_skeleton
		if skeleton_path.is_empty():
			skeleton_path = reference.m_skeletonNodePath
	if skeleton_path.is_empty():
		skeleton_path = "Bone"

	if skeleton == null:
		push_warning("MMD: no reference model found for %s, only rotation tracks are imported and IK cannot be baked. Set '%s' in the import dock to bind translations and IK." % [source_file.get_file(), OPTION_REFERENCE_MODEL])
	else:
		print("MMD: %d of %d animated bones found in the reference skeleton, bone tracks are relative to '%s'" % [
			VMDAnimationBuilder.count_matching_bones(vmd, skeleton),
			vmd.m_boneKeyFrames.size(),
			skeleton_path,
		])

	var animation := VMDAnimationBuilder.build_bone_animation(vmd, skeleton, skeleton_path, scale, fps)
	if animation != null and skeleton != null:
		MMDIKBaker.bake(vmd, skeleton, skeleton_path, scale, fps, animation)
	if reference != null:
		reference.release()

	var animation_name := source_file.get_file().get_basename()
	var library := AnimationLibrary.new()
	library.resource_name = animation_name
	if animation == null:
		push_warning("MMD: %s has no bone key frames, the imported library is empty" % source_file)
	else:
		animation.resource_name = animation_name
		# Keep the duration of the whole motion (morph and camera keys may run
		# past the last bone key) so playing stops when MMD would stop.
		if fps > 0.0:
			animation.length = maxf(animation.length, float(vmd.get_last_frame()) / fps)
		library.add_animation(animation_name, animation)

	var path := "%s.%s" % [save_path, _get_save_extension()]
	var err := ResourceSaver.save(library, path)
	if err != OK:
		push_error("MMD: cannot save imported motion to %s" % path)
	return err


func _resolve_reference(vmd_path: String, requested: String, vmd: VMDFile) -> ReferenceModel:
	if not requested.is_empty():
		return _load_reference(requested, vmd)
	return _find_best_reference(vmd_path, vmd)


## Scans the folder of the motion file for imported models sharing bone names
## with the motion and returns the best match.
func _find_best_reference(vmd_path: String, vmd: VMDFile) -> ReferenceModel:
	var candidates := PackedStringArray()
	_collect_models(vmd_path.get_base_dir(), candidates, REFERENCE_SEARCH_DEPTH)

	var best: ReferenceModel = null
	var best_score := 0
	for candidate in candidates:
		var model := _load_reference(candidate, vmd)
		if model == null:
			continue
		var score := VMDAnimationBuilder.count_matching_bones(vmd, model.m_skeleton)
		if score > best_score:
			best_score = score
			if best != null:
				best.release()
			best = model
		else:
			model.release()
	if best == null:
		return null
	print("MMD: reference model for %s: %s (%d matched bones)" % [vmd_path.get_file(), best.m_source, best_score])
	return best


func _collect_models(directory: String, out: PackedStringArray, depth: int) -> void:
	if out.size() >= REFERENCE_SEARCH_LIMIT:
		return
	var files := DirAccess.get_files_at(directory)
	if files != null:
		for file_name in files:
			if file_name.get_extension().to_lower() != "pmx":
				continue
			out.append(directory.path_join(file_name))
			if out.size() >= REFERENCE_SEARCH_LIMIT:
				return
	if depth <= 0:
		return
	var directories := DirAccess.get_directories_at(directory)
	if directories == null:
		return
	for directory_name in directories:
		if directory_name.begins_with(".") or directory_name == "addons":
			continue
		_collect_models(directory.path_join(directory_name), out, depth - 1)


func _load_reference(path: String, vmd: VMDFile) -> ReferenceModel:
	var localized := path
	if not localized.begins_with("res://") and not localized.is_absolute_path():
		localized = "res://" + localized
	if not ResourceLoader.exists(localized):
		print_verbose("MMD: reference model %s is not imported yet" % localized)
		return null
	var packed := load(localized) as PackedScene
	if packed == null:
		return null
	var root := packed.instantiate()
	if root == null:
		return null
	var skeleton := VMDAnimationBuilder.find_skeleton(root)
	if skeleton == null:
		root.free()
		print_verbose("MMD: no Skeleton3D in reference model %s" % localized)
		return null
	if VMDAnimationBuilder.count_matching_bones(vmd, skeleton) == 0:
		root.free()
		return null

	var model := ReferenceModel.new()
	model.m_root = root
	model.m_skeleton = skeleton
	model.m_skeletonNodePath = VMDAnimationBuilder.get_skeleton_node_path(root, skeleton)
	model.m_source = localized
	return model
