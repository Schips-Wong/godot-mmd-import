class_name MMDAnimationRetarget

## Remaps an [AnimationLibrary] authored for another rig onto an MMD [Skeleton3D].
##
## [SkeletonProfileHumanoid] has no entry for the extra bones a humanoid rig uses
## to carry the character itself (`b` for the root motion, `d_top`/`d_wpn*` for
## attachments), so those tracks stay unbound on a PMX skeleton and the root
## motion (translation, turn) is lost. Mapping them onto an existing MMD bone
## such as [code]Root[/code] (全ての親) keeps the motion.
##
## A mapped bone that already has a track on the target (e.g. [code]Root[/code])
## is merged: the transforms of both source bones are composed, so nothing is
## overwritten.
##
## Position tracks are converted through [member Skeleton3D.motion_scale] of
## both skeletons. In the Godot retarget workflow it holds the hip height, so
## the motion keeps its amplitude relative to the body.

## Bones a gLTF rig animates but a PMX model does not have, and the MMD bone
## that should carry their motion. An empty target removes the track.
const DEFAULT_BONE_MAP := {
	"b": "Root",
}


## Returns a copy of [param library] usable on [param target]. The source
## library is not modified.
static func remap_library(source: Skeleton3D, target: Skeleton3D, library: AnimationLibrary, bone_map: Dictionary = {}) -> AnimationLibrary:
	if source == null or target == null or library == null:
		return library
	if bone_map.is_empty():
		bone_map = DEFAULT_BONE_MAP
	var map := _validate_map(source, target, bone_map)
	var out := AnimationLibrary.new()
	for animation_name in library.get_animation_list():
		var animation: Animation = library.get_animation(animation_name)
		if animation == null:
			continue
		out.add_animation(animation_name, remap_animation(source, target, animation, map))
	return out


## Returns true when [param library] contains tracks for a bone of the map, i.e.
## when remapping it would change anything.
static func can_remap(library: AnimationLibrary, bone_map: Dictionary = {}) -> bool:
	if library == null:
		return false
	if bone_map.is_empty():
		bone_map = DEFAULT_BONE_MAP
	for animation_name in library.get_animation_list():
		var animation: Animation = library.get_animation(animation_name)
		if animation == null:
			continue
		for track in animation.get_track_count():
			if bone_map.has(_track_bone(animation, track)):
				return true
	return false


## Returns a copy of [param animation] usable on [param target].
static func remap_animation(source: Skeleton3D, target: Skeleton3D, animation: Animation, bone_map: Dictionary = {}) -> Animation:
	if bone_map.is_empty():
		bone_map = DEFAULT_BONE_MAP
	var map := _validate_map(source, target, bone_map)
	var out := Animation.new()
	out.length = animation.length
	out.step = animation.step
	out.loop_mode = animation.loop_mode

	var motion_scale_src := maxf(source.motion_scale, 0.000001)
	var motion_scale_tgt := maxf(target.motion_scale, 0.000001)

	# A source track whose target bone is animated as well has to be composed
	# with the target track instead of replacing it (e.g. P0 animates both
	# "Root" and "b", the PMX model only has "Root").
	var merged_sources := {}
	var merged_targets := {}
	for source_bone in map:
		var target_bone := String(map[source_bone])
		if target_bone.is_empty() or target_bone == String(source_bone):
			continue
		for source_track in _find_tracks(animation, String(source_bone)):
			var target_track := _find_track(animation, target_bone, animation.track_get_type(source_track))
			if target_track >= 0:
				merged_sources[source_track] = target_track
				merged_targets[target_track] = source_track

	for track in animation.get_track_count():
		if merged_sources.has(track):
			continue
		var bone := _track_bone(animation, track)
		if merged_targets.has(track):
			_append_merged(out, animation, track, merged_targets[track], source, target, motion_scale_src, motion_scale_tgt)
			continue
		if map.has(bone):
			var target_bone := String(map[bone])
			if target_bone.is_empty():
				continue  # Explicitly dropped.
			if target_bone != bone:
				_append_remapped(out, animation, track, target_bone, source, target, motion_scale_src, motion_scale_tgt)
				continue
		_append_copy(out, animation, track)
	return out


# --------------------------------------------------------------------------
# Track builders
# --------------------------------------------------------------------------

## Copies a track without touching its path or values.
static func _append_copy(out: Animation, animation: Animation, track: int) -> void:
	var out_track := out.add_track(animation.track_get_type(track))
	_copy_settings(animation, track, out, out_track)
	out.track_set_path(out_track, animation.track_get_path(track))
	_copy_keys(out, out_track, animation, track)


## Rewrites a track onto another bone, converting its values to the target rest.
static func _append_remapped(out: Animation, animation: Animation, track: int, target_bone: String, source: Skeleton3D, target: Skeleton3D, motion_scale_src: float, motion_scale_tgt: float) -> void:
	var type := animation.track_get_type(track)
	var source_bone := _track_bone(animation, track)
	var source_idx := source.find_bone(source_bone)
	var target_idx := target.find_bone(target_bone)
	if source_idx < 0 or target_idx < 0:
		_append_copy(out, animation, track)
		return

	var out_track := out.add_track(type)
	_copy_settings(animation, track, out, out_track)
	out.track_set_path(out_track, _replace_bone(animation.track_get_path(track), target_bone))

	var key_count := animation.track_get_key_count(track)
	match type:
		Animation.TYPE_POSITION_3D:
			var base := target.get_bone_rest(target_idx).origin / motion_scale_tgt
			var offset := source.get_bone_rest(source_idx).origin / motion_scale_src
			for key in key_count:
				var position: Vector3 = animation.track_get_key_value(track, key)
				out.track_insert_key(out_track, animation.track_get_key_time(track, key), base + (position - offset))
		Animation.TYPE_ROTATION_3D:
			var source_rest := source.get_bone_rest(source_idx).basis.get_rotation_quaternion()
			var target_rest := target.get_bone_rest(target_idx).basis.get_rotation_quaternion()
			for key in key_count:
				var rotation: Quaternion = animation.track_get_key_value(track, key)
				out.track_insert_key(out_track, animation.track_get_key_time(track, key), (target_rest * (source_rest.inverse() * rotation)).normalized())
		_:
			_copy_keys(out, out_track, animation, track)


## Composes a source track onto the track the target bone already has.
static func _append_merged(out: Animation, animation: Animation, target_track: int, source_track: int, source: Skeleton3D, target: Skeleton3D, motion_scale_src: float, motion_scale_tgt: float) -> void:
	var type := animation.track_get_type(target_track)
	var target_bone := _track_bone(animation, target_track)
	var source_bone := _track_bone(animation, source_track)
	var source_idx := source.find_bone(source_bone)
	var target_idx := target.find_bone(target_bone)
	if source_idx < 0 or target_idx < 0:
		return

	var out_track := out.add_track(type)
	_copy_settings(animation, target_track, out, out_track)
	out.track_set_path(out_track, animation.track_get_path(target_track))

	match type:
		Animation.TYPE_POSITION_3D:
			# Both tracks store positions divided by the source motion scale
			# (see AnimationMixer), so the deltas can be added as they are. The
			# target track came from the source rig, so it misses the rest
			# offset of the target bone and has to be added back.
			var offset := source.get_bone_rest(source_idx).origin / motion_scale_src
			var base := target.get_bone_rest(target_idx).origin / motion_scale_tgt
			for time in _collect_times(animation, target_track, source_track):
				var existing: Vector3 = _sample_vector(animation, target_track, time)
				var motion: Vector3 = _sample_vector(animation, source_track, time)
				out.track_insert_key(out_track, time, base + existing + (motion - offset))
		Animation.TYPE_ROTATION_3D:
			var source_rest := source.get_bone_rest(source_idx).basis.get_rotation_quaternion()
			for time in _collect_times(animation, target_track, source_track):
				var existing: Quaternion = _sample_quaternion(animation, target_track, time)
				var motion: Quaternion = _sample_quaternion(animation, source_track, time)
				out.track_insert_key(out_track, time, (existing * (source_rest.inverse() * motion)).normalized())
		_:
			_copy_keys(out, out_track, animation, target_track)


static func _copy_keys(out: Animation, out_track: int, animation: Animation, track: int) -> void:
	for key in animation.track_get_key_count(track):
		out.track_insert_key(out_track, animation.track_get_key_time(track, key), animation.track_get_key_value(track, key))


static func _copy_settings(animation: Animation, track: int, out: Animation, out_track: int) -> void:
	out.track_set_interpolation_type(out_track, animation.track_get_interpolation_type(track))
	out.track_set_interpolation_loop_wrap(out_track, animation.track_get_interpolation_loop_wrap(track))
	if animation.track_get_type(track) == Animation.TYPE_VALUE:
		out.value_track_set_update_mode(out_track, animation.value_track_get_update_mode(track))


# --------------------------------------------------------------------------
# Track helpers
# --------------------------------------------------------------------------

static func _validate_map(source: Skeleton3D, target: Skeleton3D, bone_map: Dictionary) -> Dictionary:
	var map := {}
	for source_bone in bone_map:
		var target_bone := String(bone_map[source_bone])
		if target_bone.is_empty():
			map[String(source_bone)] = ""
			continue
		if source.find_bone(String(source_bone)) < 0:
			continue
		if target.find_bone(target_bone) < 0:
			push_warning("MMDAnimationRetarget: target skeleton has no bone '%s' for '%s'." % [target_bone, source_bone])
			continue
		map[String(source_bone)] = target_bone
	return map


static func _track_bone(animation: Animation, track: int) -> String:
	return String(animation.track_get_path(track).get_concatenated_subnames())


static func _find_tracks(animation: Animation, bone: String) -> Array:
	var tracks := []
	for track in animation.get_track_count():
		if _track_bone(animation, track) == bone:
			tracks.append(track)
	return tracks


static func _find_track(animation: Animation, bone: String, type: int) -> int:
	for track in animation.get_track_count():
		if animation.track_get_type(track) == type and _track_bone(animation, track) == bone:
			return track
	return -1


static func _replace_bone(path: NodePath, bone: String) -> NodePath:
	var names := String(path.get_concatenated_names())
	if names.is_empty():
		return NodePath(bone)
	return NodePath("%s:%s" % [names, bone])


static func _collect_times(animation: Animation, track_a: int, track_b: int) -> Array:
	var times := []
	for track in [track_a, track_b]:
		for key in animation.track_get_key_count(track):
			times.append(animation.track_get_key_time(track, key))
	times.sort()
	var unique := []
	for time in times:
		if unique.is_empty() or absf(float(unique[unique.size() - 1]) - float(time)) > 0.000001:
			unique.append(time)
	return unique


static func _sample_vector(animation: Animation, track: int, time: float) -> Vector3:
	var key_count := animation.track_get_key_count(track)
	if key_count == 0:
		return Vector3.ZERO
	if time <= animation.track_get_key_time(track, 0):
		return animation.track_get_key_value(track, 0)
	var last := key_count - 1
	if time >= animation.track_get_key_time(track, last):
		return animation.track_get_key_value(track, last)
	for key in range(1, key_count):
		var key_time := animation.track_get_key_time(track, key)
		if key_time < time:
			continue
		var previous := key - 1
		var previous_time := animation.track_get_key_time(track, previous)
		var span := key_time - previous_time
		var weight := 0.0 if span <= 0.0 else (time - previous_time) / span
		var from: Vector3 = animation.track_get_key_value(track, previous)
		var to: Vector3 = animation.track_get_key_value(track, key)
		return from.lerp(to, weight)
	return animation.track_get_key_value(track, last)


static func _sample_quaternion(animation: Animation, track: int, time: float) -> Quaternion:
	var key_count := animation.track_get_key_count(track)
	if key_count == 0:
		return Quaternion()
	if time <= animation.track_get_key_time(track, 0):
		return animation.track_get_key_value(track, 0)
	var last := key_count - 1
	if time >= animation.track_get_key_time(track, last):
		return animation.track_get_key_value(track, last)
	for key in range(1, key_count):
		var key_time := animation.track_get_key_time(track, key)
		if key_time < time:
			continue
		var previous := key - 1
		var previous_time := animation.track_get_key_time(track, previous)
		var span := key_time - previous_time
		var weight := 0.0 if span <= 0.0 else (time - previous_time) / span
		var from: Quaternion = animation.track_get_key_value(track, previous)
		var to: Quaternion = animation.track_get_key_value(track, key)
		return from.slerp(to, weight)
	return animation.track_get_key_value(track, last)
