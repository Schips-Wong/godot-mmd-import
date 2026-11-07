class_name MMDPoseBaker

## Bakes the MMD pose rules into ordinary animation tracks.
##
## [b]IK[/b]: MMD solves IK at playback time, but the result only depends on the
## motion and the rest pose, so it can be precomputed. The baker samples the
## pose of every frame, runs the PMX CCD solver on the reference skeleton and
## writes the chain rotations as normal rotation keys.
##
## [b]付与 (append / grant)[/b]: a bone inherits a weighted part of another bone's
## animation transform. It is baked as well, so the motion is self contained.
##
## Baking is [b]opt in[/b]: imported motions are solved at runtime by
## [MMDPoseSolver], which keeps them blendable. A baked motion is what a game
## needs when it has to play without the addon (glTF export, third party tools),
## because then the keys alone carry the final bone transforms.
##
## A baked [Animation] is marked with [constant BAKED_META] (付与) or
## [constant BAKED_IK_META] (IK). [MMDPoseSolver] reads those markers and skips
## the rule for the same motion, otherwise it would be applied twice.
##
## [b]Any AnimationLibrary[/b]: [method bake_library] and [method bake_animation]
## sample the tracks of an existing animation instead of a VMD file, so motions
## from other sources (retargeted rigs, hand made animations) can be made self
## contained as well.
##
## The rules are read from the [Skeleton3D] metadata written by [PMXImporter].
## The bones are evaluated in the PMX transform order, which is the order MMD
## uses too: the 付与 rule of a bone runs first, then its IK chain.


const DATA_META := "mmd_bone_data"

## Set on every animation that already contains the 付与 result.
const BAKED_META := "mmd_append_baked"
## Set on every animation that already contains the IK result.
const BAKED_IK_META := "mmd_ik_baked"

## Frame rate used when the animation does not define [member Animation.step].
const DEFAULT_FPS := 30.0
const MAX_SAMPLING_RATE := 240.0
const MAX_FRAMES := 20000
const MIN_KEY_DELTA := 0.0001


## VMD entry point: bakes the pose rules of [param vmd] into [param animation],
## which has to be built from the same file (see [VMDAnimationBuilder]).
##
## When [param bake_ik] is false the IK chains are left untouched: only 付与 is
## baked, which keeps the rotation keys of the file as they are and leaves the
## IK to [MMDPoseSolver] at runtime.
##
## Returns the number of baked bones.
static func bake(vmd: VMDFile, skeleton: Skeleton3D, skeleton_node_path: String, _scale: float, _fps: float, animation: Animation, bake_append: bool, bake_ik: bool = true) -> int:
	if vmd == null:
		return 0
	var toggles := {}
	if bake_ik:
		toggles = _collect_ik_toggles(vmd, skeleton, _collect_entries(skeleton))
	return bake_animation(skeleton, animation, bake_append, skeleton_node_path, toggles, bake_ik)


## Bakes every animation of [param library] in place. Returns the number of
## animations that were baked (already baked ones are skipped).
static func bake_library(skeleton: Skeleton3D, library: AnimationLibrary, bake_append: bool = true, bake_ik: bool = true) -> int:
	if skeleton == null or library == null:
		return 0
	var baked := 0
	for animation_name in library.get_animation_list():
		var animation: Animation = library.get_animation(animation_name)
		if animation == null:
			continue
		if bake_animation(skeleton, animation, bake_append, "", {}, bake_ik) > 0:
			baked += 1
	return baked


## Bakes the pose rules of [param skeleton] into [param animation] and returns
## the number of baked bones. The track can be any bone animation: the poses are
## read from the tracks of [param animation] itself.
##
## [param skeleton_node_path] is only used for tracks that have to be created
## (bones moved by IK or 付与 without keys of their own). When it is empty the
## path is taken from the existing tracks.
##
## [param ik_toggles] enables the VMD IK on/off property keys, see
## [method _collect_ik_toggles]; pass an empty dictionary for plain animations,
## where every IK chain stays enabled.
##
## [param bake_ik] set to false skips the IK chains: the rotations of the motion
## are kept and only the 付与 rule is written into the keys.
static func bake_animation(skeleton: Skeleton3D, animation: Animation, bake_append: bool = true, skeleton_node_path: String = "", ik_toggles: Dictionary = {}, bake_ik: bool = true) -> int:
	if skeleton == null or animation == null:
		return 0
	if is_baked(animation):
		return 0
	var entries := _collect_entries(skeleton)
	if entries.is_empty():
		return 0

	var has_ik := false
	var has_append := false
	for entry in entries:
		if bake_ik and int(entry["ik_target"]) >= 0:
			has_ik = true
		if bake_append and int(entry["append"]) >= 0:
			has_append = true
	if not has_ik and not has_append:
		return 0

	var bone_count := skeleton.get_bone_count()
	var track_root := skeleton_node_path
	if track_root.is_empty():
		track_root = _infer_track_root(animation)

	# Bone tracks of the motion, snapshotted because the baked keys are written
	# back into the same tracks: sampling the live [Animation] would feed the
	# 付与 result of a frame into the next one.
	var sample_tracks := {}
	var rotation_tracks := {}
	var position_tracks := {}
	for track in range(animation.get_track_count()):
		var type := animation.track_get_type(track)
		if type != Animation.TYPE_ROTATION_3D and type != Animation.TYPE_POSITION_3D and type != Animation.TYPE_SCALE_3D:
			continue
		var bone_name := _track_bone(animation, track)
		var bone := skeleton.find_bone(bone_name)
		if bone < 0:
			continue
		if not sample_tracks.has(bone):
			sample_tracks[bone] = {"rotation": {}, "position": {}, "scale": {}}
		var snapshot := _snapshot_track(animation, track)
		match type:
			Animation.TYPE_ROTATION_3D:
				sample_tracks[bone]["rotation"] = snapshot
				rotation_tracks[bone_name] = track
			Animation.TYPE_POSITION_3D:
				sample_tracks[bone]["position"] = snapshot
				position_tracks[bone_name] = track
			_:
				sample_tracks[bone]["scale"] = snapshot

	# Every bone modified by a rule needs a key on every frame: it has no keys of
	# its own in the motion (a knee moved by the leg IK only, a twist bone...).
	var baked_rotation := {}
	var baked_position := {}
	for entry in entries:
		var links: Array = entry["ik_links"]
		if bake_ik and int(entry["ik_target"]) >= 0 and not links.is_empty():
			for link in links:
				var link_bone: int = int(link["bone"])
				if link_bone < 0 or link_bone >= bone_count:
					continue
				var link_track := _ensure_track(animation, rotation_tracks, track_root, skeleton.get_bone_name(link_bone), Animation.TYPE_ROTATION_3D)
				if link_track >= 0:
					baked_rotation[link_bone] = link_track
		if bake_append and int(entry["append"]) >= 0:
			var append_bone: int = int(entry["bone"])
			if append_bone < 0 or append_bone >= bone_count:
				continue
			var append_name := skeleton.get_bone_name(append_bone)
			var append_track := _ensure_track(animation, rotation_tracks, track_root, append_name, Animation.TYPE_ROTATION_3D)
			if append_track >= 0:
				baked_rotation[append_bone] = append_track
			if entry["pos"]:
				var append_position := _ensure_track(animation, position_tracks, track_root, append_name, Animation.TYPE_POSITION_3D)
				if append_position >= 0:
					baked_position[append_bone] = append_position

	if baked_rotation.is_empty() and baked_position.is_empty():
		return 0

	# Bones whose pose is modified by a rule and therefore has to be restored
	# from the motion before every frame is evaluated.
	var modified_bones := {}
	for entry in entries:
		if int(entry["append"]) >= 0:
			modified_bones[int(entry["bone"])] = true
	for bone in baked_rotation:
		modified_bones[bone] = true
	for bone in baked_position:
		modified_bones[bone] = true
	for bone in modified_bones:
		if not sample_tracks.has(bone) and bone >= 0 and bone < bone_count:
			sample_tracks[bone] = {"rotation": {}, "position": {}, "scale": {}}

	var frame_rate := _sampling_rate(animation)
	var last_frame := int(round(animation.length * frame_rate))
	# IK can be toggled in the property section of the motion.
	for controller_name in ik_toggles:
		var states: Array = ik_toggles[controller_name]["states"]
		if not states.is_empty():
			last_frame = maxi(last_frame, int(states[states.size() - 1][0]))
	last_frame = mini(last_frame, MAX_FRAMES)

	# The skeleton is posed while baking; the caller may own it (edited scene),
	# so the original pose is restored afterwards.
	var saved_poses: Array[Transform3D] = []
	for bone in range(bone_count):
		saved_poses.append(skeleton.get_bone_pose(bone))
		_reset_to_rest(skeleton, bone)

	var global_cache := {}
	for frame in range(last_frame + 1):
		var time := float(frame) / frame_rate
		_pose_from_tracks(skeleton, sample_tracks, time)
		global_cache.clear()

		# Same order as MMD: 付与 of a bone, then the IK chain of that bone.
		for entry in entries:
			if int(entry["append"]) >= 0:
				MMDPoseSolver.apply_append(skeleton, entry)
				global_cache.clear()
			if not bake_ik:
				continue
			var target: int = int(entry["ik_target"])
			if target < 0 or target >= bone_count:
				continue
			var controller_name := skeleton.get_bone_name(int(entry["bone"]))
			if _is_ik_enabled(ik_toggles, controller_name, frame):
				MMDIKSolver.solve(skeleton, entry, global_cache)

		for bone in baked_rotation:
			animation.rotation_track_insert_key(baked_rotation[bone], time, skeleton.get_bone_pose_rotation(bone))
		for bone in baked_position:
			animation.position_track_insert_key(baked_position[bone], time, skeleton.get_bone_pose_position(bone))

	for bone in range(bone_count):
		skeleton.set_bone_pose(bone, saved_poses[bone])

	animation.length = maxf(animation.length, float(last_frame) / frame_rate)
	var baked_bones := {}
	for bone in baked_rotation:
		baked_bones[bone] = true
	for bone in baked_position:
		baked_bones[bone] = true
	if has_append:
		animation.set_meta(BAKED_META, true)
	if has_ik:
		animation.set_meta(BAKED_IK_META, true)
	print("MMD: baked %d bones over %d frames (付与: %s, IK: %s)" % [baked_bones.size(), last_frame + 1, bake_append, bake_ik])
	return baked_bones.size()


## Returns true when [param animation] already contains the result of a rule.
static func is_baked(animation: Animation) -> bool:
	if animation == null:
		return false
	for key in [BAKED_META, BAKED_IK_META]:
		if animation.has_meta(key) and bool(animation.get_meta(key)):
			return true
	return false


static func _collect_entries(skeleton: Skeleton3D) -> Array:
	if skeleton == null or not skeleton.has_meta(DATA_META):
		return []
	var data: Variant = skeleton.get_meta(DATA_META)
	if not data is Array:
		return []
	var entries: Array = []
	for entry in data:
		if entry is Dictionary:
			entries.append(entry)
	if entries.is_empty():
		return entries
	entries.sort_custom(MMDPoseSolver.sort_entries)
	return entries


## Returns the track of [param bone_name], creating it when the motion has no
## track for that bone yet. Returns -1 when the track cannot be created.
static func _ensure_track(animation: Animation, tracks: Dictionary, track_root: String, bone_name: String, type: int) -> int:
	if tracks.has(bone_name):
		return tracks[bone_name]
	var track := animation.add_track(type)
	animation.track_set_path(track, NodePath("%s:%s" % [track_root, bone_name]))
	animation.track_set_interpolation_type(track, Animation.INTERPOLATION_LINEAR)
	tracks[bone_name] = track
	return track


## Returns the node path part (the skeleton, relative to the mixer root) used by
## the bone tracks of [param animation], so new tracks match them.
static func _infer_track_root(animation: Animation) -> String:
	var counts := {}
	for track in range(animation.get_track_count()):
		var type := animation.track_get_type(track)
		if type != Animation.TYPE_ROTATION_3D and type != Animation.TYPE_POSITION_3D and type != Animation.TYPE_SCALE_3D:
			continue
		var names := String(animation.track_get_path(track).get_concatenated_names())
		if names.is_empty():
			continue
		counts[names] = int(counts.get(names, 0)) + 1
	var best := ""
	var best_count := 0
	for names in counts:
		if int(counts[names]) > best_count:
			best = names
			best_count = int(counts[names])
	return best


static func _track_bone(animation: Animation, track: int) -> String:
	return String(animation.track_get_path(track).get_concatenated_subnames())


## Applies the sampled pose of every track of [param sample_tracks]. Bones
## without a track fall back to their rest pose, which removes the rule result
## of the previous frame.
static func _pose_from_tracks(skeleton: Skeleton3D, sample_tracks: Dictionary, time: float) -> void:
	for bone in sample_tracks:
		var tracks: Dictionary = sample_tracks[bone]
		var rotation_track: Dictionary = tracks["rotation"]
		if rotation_track.is_empty():
			skeleton.set_bone_pose_rotation(bone, skeleton.get_bone_rest(bone).basis.get_rotation_quaternion())
		else:
			skeleton.set_bone_pose_rotation(bone, _sample_quaternion(rotation_track, time))
		var position_track: Dictionary = tracks["position"]
		if position_track.is_empty():
			skeleton.set_bone_pose_position(bone, skeleton.get_bone_rest(bone).origin)
		else:
			skeleton.set_bone_pose_position(bone, _sample_vector(position_track, time))
		var scale_track: Dictionary = tracks["scale"]
		if not scale_track.is_empty():
			skeleton.set_bone_pose_scale(bone, _sample_vector(scale_track, time))


## Returns the frame rate the tracks have to be sampled at: the animation step,
## but never coarser than the closest pair of keys.
static func _sampling_rate(animation: Animation) -> float:
	var rate := DEFAULT_FPS
	if animation.step > 0.0:
		rate = clampf(1.0 / animation.step, 1.0, MAX_SAMPLING_RATE)
	var smallest := INF
	for track in range(animation.get_track_count()):
		var key_count := animation.track_get_key_count(track)
		for key in range(1, key_count):
			var delta := animation.track_get_key_time(track, key) - animation.track_get_key_time(track, key - 1)
			if delta > MIN_KEY_DELTA:
				smallest = minf(smallest, delta)
	if smallest < INF:
		rate = maxf(rate, clampf(1.0 / smallest, 1.0, MAX_SAMPLING_RATE))
	return rate


static func _reset_to_rest(skeleton: Skeleton3D, bone: int) -> void:
	var rest := skeleton.get_bone_rest(bone)
	skeleton.set_bone_pose_position(bone, rest.origin)
	skeleton.set_bone_pose_rotation(bone, rest.basis.get_rotation_quaternion())
	skeleton.set_bone_pose_scale(bone, rest.basis.get_scale())


# --------------------------------------------------------------------------
# Track sampling
# --------------------------------------------------------------------------

## Copies the keys of [param track] so the frames can be sampled while the same
## track is written. Returns { "times": PackedFloat64Array, "values": Array,
## "interpolation": int }.
static func _snapshot_track(animation: Animation, track: int) -> Dictionary:
	var times := PackedFloat64Array()
	var values := []
	for key in range(animation.track_get_key_count(track)):
		times.append(animation.track_get_key_time(track, key))
		values.append(animation.track_get_key_value(track, key))
	return {
		"times": times,
		"values": values,
		"interpolation": animation.track_get_interpolation_type(track),
	}


## Returns the index of the key at or before [param time].
static func _snapshot_index(snapshot: Dictionary, time: float) -> int:
	var times: PackedFloat64Array = snapshot["times"]
	if times.is_empty():
		return -1
	var low := 0
	var high := times.size() - 1
	while low < high:
		var middle := (low + high + 1) / 2
		if times[middle] <= time:
			low = middle
		else:
			high = middle - 1
	return low


static func _sample_vector(snapshot: Dictionary, time: float) -> Vector3:
	var values: Array = snapshot["values"]
	var index := _snapshot_index(snapshot, time)
	if index < 0:
		return Vector3.ZERO
	var value: Vector3 = values[index]
	if index + 1 >= values.size():
		return value
	# Cubic keys are resampled linearly: the rule bones get a key on every
	# frame anyway, so the result stays smooth in the baked animation.
	if int(snapshot["interpolation"]) == Animation.INTERPOLATION_NEAREST:
		return value
	var times: PackedFloat64Array = snapshot["times"]
	var span := times[index + 1] - times[index]
	if span <= 0.0:
		return value
	var next: Vector3 = values[index + 1]
	return value.lerp(next, clampf((time - times[index]) / span, 0.0, 1.0))


static func _sample_quaternion(snapshot: Dictionary, time: float) -> Quaternion:
	var values: Array = snapshot["values"]
	var index := _snapshot_index(snapshot, time)
	if index < 0:
		return Quaternion.IDENTITY
	var value: Quaternion = values[index]
	if index + 1 >= values.size():
		return value
	if int(snapshot["interpolation"]) == Animation.INTERPOLATION_NEAREST:
		return value
	var times: PackedFloat64Array = snapshot["times"]
	var span := times[index + 1] - times[index]
	if span <= 0.0:
		return value
	var next: Quaternion = values[index + 1]
	return value.slerp(next, clampf((time - times[index]) / span, 0.0, 1.0))


# --------------------------------------------------------------------------
# IK toggles (VMD property key frames)
# --------------------------------------------------------------------------

## Returns controller bone name -> { "states": [[frame, enabled], ...],
## "index": int, "enabled": bool }. The IK is enabled until a property key says
## otherwise.
static func _collect_ik_toggles(vmd: VMDFile, skeleton: Skeleton3D, entries: Array) -> Dictionary:
	var toggles := {}
	if vmd == null or skeleton == null:
		return toggles
	for entry in entries:
		if int(entry["ik_target"]) < 0:
			continue
		var controller: int = int(entry["bone"])
		if controller >= 0 and controller < skeleton.get_bone_count():
			toggles[skeleton.get_bone_name(controller)] = {
				"states": [],
				"index": 0,
				"enabled": true,
			}
	for property_key in vmd.m_propertyKeyFrames:
		for ik_state in property_key.m_ikStates:
			var name: String = ik_state[0]
			if toggles.has(name):
				toggles[name]["states"].append([property_key.m_frame, ik_state[1]])
	for name in toggles:
		toggles[name]["states"].sort_custom(_sort_toggle_states)
	return toggles


static func _sort_toggle_states(a: Array, b: Array) -> bool:
	return int(a[0]) < int(b[0])


static func _is_ik_enabled(toggles: Dictionary, controller_name: String, frame: int) -> bool:
	if not toggles.has(controller_name):
		return true
	var state: Dictionary = toggles[controller_name]
	var states: Array = state["states"]
	var index: int = state["index"]
	while index < states.size() and int(states[index][0]) <= frame:
		state["enabled"] = states[index][1]
		index += 1
	state["index"] = index
	return state["enabled"]
