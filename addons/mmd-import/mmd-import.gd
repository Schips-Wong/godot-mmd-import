@tool
extends EditorPlugin

var pmxImporter: EditorSceneFormatImporter
var vmdImporter: EditorImportPlugin

func _enter_tree() -> void:
	# Initialization of the plugin goes here.
	print("MMD Import plugin activated.")
	pmxImporter = PMXImporter.new()
	add_scene_format_importer_plugin(pmxImporter)
	vmdImporter = VMDImporter.new()
	add_import_plugin(vmdImporter, false)
	pass


func _exit_tree() -> void:
	# Clean-up of the plugin goes here.
	remove_import_plugin(vmdImporter)
	vmdImporter = null
	remove_scene_format_importer_plugin(pmxImporter)
	pmxImporter = null
	pass
