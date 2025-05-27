@tool
extends EditorPlugin

var pmxImporter: EditorSceneFormatImporter

func _enter_tree() -> void:
	# Initialization of the plugin goes here.
	print("MMD Import plugin activated.")
	pmxImporter = PMXImporter.new()
	add_scene_format_importer_plugin(pmxImporter)
	pass


func _exit_tree() -> void:
	# Clean-up of the plugin goes here.
	remove_scene_format_importer_plugin(pmxImporter)
	pass
