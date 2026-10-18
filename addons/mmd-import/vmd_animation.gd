class_name VMDAnimationBuilder

## Converts parsed [VMDFile] key frames into Godot [Animation] tracks.
##
## Axis conversion matches [PMXImporter]: MMD is Y-up with Z pointing into the
## screen, Godot is Y-up with -Z pointing forward, so the position is mirrored
## on Z. Mirroring is an improper transform, which turns the rotation axis into
## a pseudo vector: (x, y, z) becomes (-x, -y, z) and the angle stays the same.
##
## Bone tracks are written as "SkeletonNodePath:BoneName" following Godot's
## skeleton track path convention. The animation is relative to the
## [member AnimationMixer.root_node] of the player that plays it.


## Converts an MMD position offset into Godot space.
static func convert_position(position: Vector3, scale: float = 0.1) -> Vector3:
	return Vector3(position.x, position.y, -position.z) * scale


## Converts an MMD rotation into Godot space.
static func convert_rotation(rotation: Quaternion) -> Quaternion:
	var converted := Quaternion(-rotation.x, -rotation.y, rotation.z, rotation.w)
	if is_zero_approx(converted.length_squared()):
		return Quaternion.IDENTITY
	return converted.normalized()


## Returns the first [Skeleton3D] found below [param node] (inclusive).
static func find_skeleton(node: Node) -> Skeleton3D:
	if node is Skeleton3D:
		return node
	for child in node.get_children():
		var skeleton := find_skeleton(child)
		if skeleton != null:
			return skeleton
	return null


## Builds the node path of [param skeleton] as seen from [param root], without
## a leading slash. Returns an empty string when they are not in the same tree.
static func get_skeleton_node_path(root: Node, skeleton: Skeleton3D) -> String:
	if root == null or skeleton == null or root == skeleton:
		return ""
	var path := root.get_path_to(skeleton)
	if path.is_empty():
		return ""
	return String(path)


## Counts how many bones of [param vmd] exist in [param skeleton]. Used to pick
## the best matching reference model for a motion file.
static func count_matching_bones(vmd: VMDFile, skeleton: Skeleton3D) -> int:
	if vmd == null or skeleton == null:
		return 0
	var names := {}
	for i in range(skeleton.get_bone_count()):
		names[skeleton.get_bone_name(i)] = true
	var matched := 0
	for name in vmd.m_boneKeyFrames:
		if names.has(name):
			matched += 1
	return matched


## Builds one [Animation] holding a position and a rotation track for every
## bone of [param vmd] that exists in [param skeleton].
##
## [param skeleton_node_path] is the path from the playing node to the
## skeleton, for example "Bone" or "../Model/Bone".
##
## Skeleton rotations are relative to the bone rest, so they do not need the
## rest data. Translations are absolute bone poses, so they are only written
## when a [param skeleton] is given (the bones missing in it are skipped).
## Without a skeleton every bone of the file gets a rotation track only.
static func build_bone_animation(vmd: VMDFile, skeleton: Skeleton3D, skeleton_node_path: String, scale: float = 0.1, fps: float = 30.0) -> Animation:
	if vmd == null:
		return null
	var frame_rate := fps if fps > 0.0 else 30.0
	var bone_indices := {}
	if skeleton != null:
		for i in range(skeleton.get_bone_count()):
			bone_indices[skeleton.get_bone_name(i)] = i

	var animation := Animation.new()
	animation.step = 1.0 / frame_rate
	animation.loop_mode = Animation.LOOP_NONE

	var missing := PackedStringArray()
	var track_count := 0
	var last_time := 0.0

	for bone_name in vmd.m_boneKeyFrames:
		var keys: Array = vmd.m_boneKeyFrames[bone_name]
		if keys.is_empty():
			continue
		var bone_index := -1
		var rest_origin := Vector3.ZERO
		if not bone_indices.is_empty():
			if not bone_indices.has(bone_name):
				missing.append(bone_name)
				continue
			bone_index = bone_indices[bone_name]
			rest_origin = skeleton.get_bone_rest(bone_index).origin
		var track_path := NodePath("%s:%s" % [skeleton_node_path, bone_name])

		var position_track := -1
		if bone_index >= 0 and _has_translation(keys):
			position_track = animation.add_track(Animation.TYPE_POSITION_3D)
			animation.track_set_path(position_track, track_path)
			animation.track_set_interpolation_type(position_track, Animation.INTERPOLATION_LINEAR)

		var rotation_track := animation.add_track(Animation.TYPE_ROTATION_3D)
		animation.track_set_path(rotation_track, track_path)
		animation.track_set_interpolation_type(rotation_track, Animation.INTERPOLATION_LINEAR)

		for key in keys:
			var time := float(key.m_frame) / frame_rate
			if position_track >= 0:
				animation.position_track_insert_key(position_track, time, rest_origin + convert_position(key.m_position, scale))
			animation.rotation_track_insert_key(rotation_track, time, convert_rotation(key.m_rotation))
			last_time = maxf(last_time, time)
		track_count += 1

	if track_count == 0:
		return null
	if last_time > animation.length:
		animation.length = last_time
	_report_missing_bones(missing)
	return animation


static func _has_translation(keys: Array) -> bool:
	for key in keys:
		var position: Vector3 = key.m_position
		if not position.is_zero_approx():
			return true
	return false


static func _report_missing_bones(missing: PackedStringArray) -> void:
	if missing.is_empty():
		return
	var shown := PackedStringArray()
	for i in range(mini(missing.size(), 10)):
		shown.append(missing[i])
	var message := "MMD: %d bone(s) of the motion file are not present in the reference skeleton: %s" % [missing.size(), ", ".join(shown)]
	if missing.size() > shown.size():
		message += " ..."
	print_verbose(message)
