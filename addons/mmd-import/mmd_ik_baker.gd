class_name MMDIKBaker

## Bakes the IK chains of a PMX model into ordinary animation tracks.
##
## MMD solves IK at playback time, but the result only depends on the motion and
## the rest pose, so it can be precomputed: the VMD importer samples the pose of
## every frame, runs the PMX CCD solver on the reference skeleton and writes the
## resulting chain rotations as normal rotation keys. Playing the animation no
## longer needs the IK data, only the 付与 rules handled by [MMDPoseSolver].
##
## The IK definitions are read from the [Skeleton3D] metadata written by
## [PMXImporter]. The solver mirrors the MMD behaviour: links are rotated around
## their own origin, the PMX angle limits are applied per local axis and the
## per-bone transform order of the PMX is respected.

const DATA_META := "mmd_bone_data"

const ROTATION_EPSILON := 0.0001
const IK_TOLERANCE := 0.0005
const MAX_IK_ITERATIONS := 255
const MIN_VECTOR_LENGTH_SQUARED := 1e-10


## Writes baked rotation keys for every IK driven bone of [param skeleton] into
## [param animation]. Returns the number of bones that were baked.
static func bake(vmd: VMDFile, skeleton: Skeleton3D, skeleton_node_path: String, scale: float, fps: float, animation: Animation) -> int:
	if vmd == null or skeleton == null or animation == null or skeleton_node_path.is_empty():
		return 0
	if not skeleton.has_meta(DATA_META):
		return 0
	var data: Variant = skeleton.get_meta(DATA_META)
	if not data is Array:
		return 0

	var entries: Array = []
	for entry in data:
		if entry is Dictionary:
			entries.append(entry)
	if entries.is_empty():
		return 0
	entries.sort_custom(MMDPoseSolver.sort_entries)

	var ik_entries: Array = []
	for entry in entries:
		if int(entry["ik_target"]) >= 0 and not (entry["ik_links"] as Array).is_empty():
			ik_entries.append(entry)
	if ik_entries.is_empty():
		return 0

	var frame_rate := fps if fps > 0.0 else 30.0
	var bone_count := skeleton.get_bone_count()
	var bone_indices := {}
	for i in range(bone_count):
		bone_indices[skeleton.get_bone_name(i)] = i

	# Make sure every IK driven bone has a rotation track, even when the motion
	# has no key of its own (for example a knee moved by the leg IK only).
	var rotation_tracks := {}
	for track in range(animation.get_track_count()):
		if animation.track_get_type(track) != Animation.TYPE_ROTATION_3D:
			continue
		rotation_tracks[String(animation.track_get_path(track).get_concatenated_subnames())] = track

	var baked: Dictionary = {}
	for entry in ik_entries:
		for link in entry["ik_links"]:
			var link_bone: int = int(link["bone"])
			if link_bone < 0 or link_bone >= bone_count or baked.has(link_bone):
				continue
			var link_name := skeleton.get_bone_name(link_bone)
			var track: int = rotation_tracks.get(link_name, -1)
			if track < 0:
				track = animation.add_track(Animation.TYPE_ROTATION_3D)
				animation.track_set_path(track, NodePath("%s:%s" % [skeleton_node_path, link_name]))
				animation.track_set_interpolation_type(track, Animation.INTERPOLATION_LINEAR)
				rotation_tracks[link_name] = track
			baked[link_bone] = track
	if baked.is_empty():
		return 0

	# Sampling state of every bone animated by the file. Keys are walked with a
	# cursor because the frames are processed in ascending order.
	var samples := {}
	var last_frame := 0
	for bone_name in vmd.m_boneKeyFrames:
		if not bone_indices.has(bone_name):
			continue
		var keys: Array = vmd.m_boneKeyFrames[bone_name]
		if keys.is_empty():
			continue
		keys.sort_custom(_sort_keys)
		samples[int(bone_indices[bone_name])] = {"keys": keys, "index": 0}
		last_frame = maxi(last_frame, int(keys[keys.size() - 1].m_frame))

	# IK can be toggled in the property section of the motion.
	var toggles := _collect_ik_toggles(vmd, skeleton, ik_entries)
	for controller_name in toggles:
		var states: Array = toggles[controller_name]["states"]
		if not states.is_empty():
			last_frame = maxi(last_frame, int(states[states.size() - 1][0]))

	# Bones whose pose is modified by the solver and therefore has to be
	# restored from the motion before every frame is evaluated.
	var modified_bones := {}
	for bone in baked:
		modified_bones[bone] = true
	for entry in entries:
		if int(entry["append"]) >= 0:
			modified_bones[int(entry["bone"])] = true

	var global_cache := {}
	for frame in range(last_frame + 1):
		for bone in samples:
			_apply_sample(skeleton, bone, samples[bone], frame, scale)
		for bone in modified_bones:
			if not samples.has(bone):
				_reset_to_rest(skeleton, bone)
		global_cache.clear()

		for entry in entries:
			if int(entry["append"]) >= 0:
				MMDPoseSolver.apply_append(skeleton, entry)
				global_cache.clear()
			var target: int = int(entry["ik_target"])
			if target < 0 or target >= bone_count:
				continue
			var controller_name := skeleton.get_bone_name(int(entry["bone"]))
			if _is_ik_enabled(toggles, controller_name, frame):
				_apply_ik(skeleton, entry, global_cache)

		var time := float(frame) / frame_rate
		for bone in baked:
			animation.rotation_track_insert_key(baked[bone], time, skeleton.get_bone_pose_rotation(bone))

	animation.length = maxf(animation.length, float(last_frame) / frame_rate)
	print("MMD: baked IK of %d bones over %d frames" % [baked.size(), last_frame + 1])
	return baked.size()


static func _sort_keys(a: VMDFile.VMDBoneKeyFrame, b: VMDFile.VMDBoneKeyFrame) -> bool:
	return a.m_frame < b.m_frame


## Sets the pose of [param bone] from its motion keys, using the linear
## interpolation of the generated animation tracks.
static func _apply_sample(skeleton: Skeleton3D, bone: int, state: Dictionary, frame: int, scale: float) -> void:
	var keys: Array = state["keys"]
	var index: int = state["index"]
	while index + 1 < keys.size() and int(keys[index + 1].m_frame) <= frame:
		index += 1
	state["index"] = index

	var key: VMDFile.VMDBoneKeyFrame = keys[index]
	var position := key.m_position
	var rotation := key.m_rotation
	if index + 1 < keys.size():
		var next: VMDFile.VMDBoneKeyFrame = keys[index + 1]
		var start := int(key.m_frame)
		var span := int(next.m_frame) - start
		if span > 0:
			var weight := clampf(float(frame - start) / float(span), 0.0, 1.0)
			position = position.lerp(next.m_position, weight)
			rotation = rotation.slerp(next.m_rotation, weight)

	var rest := skeleton.get_bone_rest(bone)
	skeleton.set_bone_pose_position(bone, rest.origin + VMDAnimationBuilder.convert_position(position, scale))
	skeleton.set_bone_pose_rotation(bone, VMDAnimationBuilder.convert_rotation(rotation))


static func _reset_to_rest(skeleton: Skeleton3D, bone: int) -> void:
	var rest := skeleton.get_bone_rest(bone)
	skeleton.set_bone_pose_position(bone, rest.origin)
	skeleton.set_bone_pose_rotation(bone, rest.basis.get_rotation_quaternion())


# --------------------------------------------------------------------------
# IK toggles (VMD property key frames)
# --------------------------------------------------------------------------

## Returns controller bone name -> { "states": [[frame, enabled], ...],
## "index": int, "enabled": bool }. The IK is enabled until a property key says
## otherwise.
static func _collect_ik_toggles(vmd: VMDFile, skeleton: Skeleton3D, ik_entries: Array) -> Dictionary:
	var toggles := {}
	for entry in ik_entries:
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


# --------------------------------------------------------------------------
# IK solver (CCD)
# --------------------------------------------------------------------------

static func _apply_ik(skeleton: Skeleton3D, entry: Dictionary, global_cache: Dictionary) -> void:
	var target: int = entry["ik_target"]
	var links: Array = entry["ik_links"]
	if target < 0 or target >= skeleton.get_bone_count() or links.is_empty():
		return
	var goal := _global_pose(skeleton, entry["bone"], global_cache).origin
	var iterations: int = clampi(int(entry["ik_iter"]), 1, MAX_IK_ITERATIONS)
	for _iteration in range(iterations):
		for link in links:
			var link_bone: int = link["bone"]
			if link_bone < 0 or link_bone >= skeleton.get_bone_count():
				continue
			if not _is_ancestor(skeleton, link_bone, target):
				continue
			_rotate_towards(skeleton, link_bone, target, goal, link, global_cache)
		if _global_pose(skeleton, target, global_cache).origin.distance_to(goal) <= IK_TOLERANCE:
			break


static func _rotate_towards(skeleton: Skeleton3D, link_bone: int, target: int, goal: Vector3, link: Dictionary, global_cache: Dictionary) -> void:
	var link_position := _global_pose(skeleton, link_bone, global_cache).origin
	var to_target := _global_pose(skeleton, target, global_cache).origin - link_position
	var to_goal := goal - link_position
	if to_target.length_squared() < MIN_VECTOR_LENGTH_SQUARED or to_goal.length_squared() < MIN_VECTOR_LENGTH_SQUARED:
		return
	var angle := to_target.angle_to(to_goal)
	if angle < ROTATION_EPSILON:
		return
	var axis := to_target.cross(to_goal)
	if axis.length_squared() < MIN_VECTOR_LENGTH_SQUARED:
		return
	var delta := Quaternion(axis.normalized(), angle)

	# Rotate in skeleton space, then convert back to the local pose.
	var parent := skeleton.get_bone_parent(link_bone)
	var parent_basis := Basis.IDENTITY
	if parent >= 0:
		parent_basis = _global_pose(skeleton, parent, global_cache).basis
	var rotation := parent_basis.inverse() * Basis(delta) * parent_basis * Basis(skeleton.get_bone_pose_rotation(link_bone))
	var local_rotation := rotation.get_rotation_quaternion()
	if link["limit"]:
		local_rotation = _clamp_rotation(local_rotation, link["min"], link["max"])
	skeleton.set_bone_pose_rotation(link_bone, local_rotation.normalized())
	global_cache.clear()


## Clamps the local rotation of an IK link to the limits of the PMX file. The
## limits are per local axis, so the rotation is converted to euler angles.
static func _clamp_rotation(rotation: Quaternion, limit_min: Vector3, limit_max: Vector3) -> Quaternion:
	var euler := Basis(rotation).get_euler(EULER_ORDER_XYZ)
	euler.x = _clamp_angle(euler.x, limit_min.x, limit_max.x)
	euler.y = _clamp_angle(euler.y, limit_min.y, limit_max.y)
	euler.z = _clamp_angle(euler.z, limit_min.z, limit_max.z)
	return Basis.from_euler(euler, EULER_ORDER_XYZ).get_rotation_quaternion()


## Clamps an angle to [param minimum, param maximum] while keeping the value
## closest to the interval, so limits like [-180, -0.5] still catch +170.
static func _clamp_angle(value: float, minimum: float, maximum: float) -> float:
	var center := (minimum + maximum) * 0.5
	return clampf(center + wrapf(value - center, -PI, PI), minimum, maximum)


static func _is_ancestor(skeleton: Skeleton3D, ancestor: int, bone: int) -> bool:
	var current := bone
	var depth := 0
	while current >= 0 and depth < skeleton.get_bone_count():
		if current == ancestor:
			return true
		current = skeleton.get_bone_parent(current)
		depth += 1
	return false


## Returns the skeleton space pose of [param bone] including the modifications
## done so far. The engine only refreshes its global poses after all modifiers
## ran, so the chain is accumulated here.
static func _global_pose(skeleton: Skeleton3D, bone: int, global_cache: Dictionary) -> Transform3D:
	if global_cache.has(bone):
		return global_cache[bone]
	var local := skeleton.get_bone_pose(bone)
	var parent := skeleton.get_bone_parent(bone)
	var result := local
	if parent >= 0:
		result = _global_pose(skeleton, parent, global_cache) * local
	global_cache[bone] = result
	return result
