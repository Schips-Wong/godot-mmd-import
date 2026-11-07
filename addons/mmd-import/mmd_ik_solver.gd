@tool
class_name MMDIKSolver

## Solves one IK chain of a PMX model on a live [Skeleton3D].
##
## Shared by [MMDPoseSolver], which solves the chains at runtime after the
## [AnimationMixer] (the way MMD does), and by [MMDPoseBaker], which samples the
## same solver into keys when a self contained animation is wanted.
##
## The chain comes from the "mmd_bone_data" metadata of the skeleton (see
## [PMXImporter]): the goal bone (the IK controller, e.g. 左足ＩＫ), the tip bone
## it has to reach (足首) and the links it may rotate (ひざ, 足). The limits are
## already converted to Godot space by the importer.
##
## Two solvers are used: a closed form two bone solution for the chains that
## have the shape of a limb (thigh + hinged knee, upper arm + elbow), and CCD for
## everything else. The closed form matches MMD far better than CCD, which
## stalls near a straight limb, but it only applies to a two link chain whose
## hinge is locked to a single axis.


const ROTATION_EPSILON := 0.0001
const IK_TOLERANCE := 0.0005
const MAX_IK_ITERATIONS := 255
const MIN_VECTOR_LENGTH_SQUARED := 1e-10


## Rotates the links of [param entry] so the tip bone reaches the goal bone.
##
## [param global_cache] holds skeleton space poses of the current frame and is
## cleared by the solver whenever it changes a pose: the engine only refreshes
## its global poses after every modifier ran, so the chain is accumulated here.
## It is an in/out parameter and has to be passed in by the caller.
static func solve(skeleton: Skeleton3D, entry: Dictionary, global_cache: Dictionary) -> void:
	var target: int = entry["ik_target"]
	var links: Array = entry["ik_links"]
	if skeleton == null or target < 0 or target >= skeleton.get_bone_count() or links.is_empty():
		return
	if _solve_two_bone(skeleton, entry, links, target, global_cache):
		return
	var goal := global_pose(skeleton, int(entry["bone"]), global_cache).origin
	var iterations: int = clampi(int(entry["ik_iter"]), 1, MAX_IK_ITERATIONS)
	for _iteration in range(iterations):
		for link in links:
			var link_bone: int = link["bone"]
			if link_bone < 0 or link_bone >= skeleton.get_bone_count():
				continue
			if not is_ancestor(skeleton, link_bone, target):
				continue
			_rotate_towards(skeleton, link_bone, target, goal, link, global_cache)
		if global_pose(skeleton, target, global_cache).origin.distance_to(goal) <= IK_TOLERANCE:
			break


## Returns the skeleton space pose of [param bone] including the modifications
## done so far. The engine only refreshes its global poses after all modifiers
## ran, so the chain is accumulated here.
static func global_pose(skeleton: Skeleton3D, bone: int, global_cache: Dictionary) -> Transform3D:
	if global_cache.has(bone):
		return global_cache[bone]
	var local := skeleton.get_bone_pose(bone)
	var parent := skeleton.get_bone_parent(bone)
	var result := local
	if parent >= 0:
		result = global_pose(skeleton, parent, global_cache) * local
	global_cache[bone] = result
	return result


## Returns true when [param ancestor] is [param bone] or one of its parents.
static func is_ancestor(skeleton: Skeleton3D, ancestor: int, bone: int) -> bool:
	var current := bone
	var depth := 0
	while current >= 0 and depth < skeleton.get_bone_count():
		if current == ancestor:
			return true
		current = skeleton.get_bone_parent(current)
		depth += 1
	return false


## Solves a two bone chain (thigh + hinged knee, elbow...) in closed form. The
## knee/elbow angle only depends on the hip to goal distance, so this matches
## MMD better than CCD and does not get stuck near a straight leg. Returns false
## when the chain does not have that shape and the CCD fallback has to run.
static func _solve_two_bone(skeleton: Skeleton3D, entry: Dictionary, links: Array, target: int, global_cache: Dictionary) -> bool:
	if links.size() != 2:
		return false
	var hinge: Dictionary = links[0]
	var root_link: Dictionary = links[1]
	if not hinge["limit"]:
		return false
	var hinge_bone: int = int(hinge["bone"])
	var root_bone: int = int(root_link["bone"])
	if hinge_bone < 0 or root_bone < 0 or hinge_bone >= skeleton.get_bone_count() or root_bone >= skeleton.get_bone_count():
		return false
	# The hinge has to be a child of the first link, and the target a child of
	# the hinge, otherwise the closed form does not apply.
	if skeleton.get_bone_parent(hinge_bone) != root_bone or skeleton.get_bone_parent(target) != hinge_bone:
		return false
	var axis := _free_axis(hinge)
	if axis == Vector3.ZERO:
		return false
	var l1 := skeleton.get_bone_rest(hinge_bone).origin.length()
	var l2 := skeleton.get_bone_rest(target).origin.length()
	if l1 <= 0.0 or l2 <= 0.0:
		return false

	var hip := global_pose(skeleton, root_bone, global_cache).origin
	var goal := global_pose(skeleton, int(entry["bone"]), global_cache).origin
	var distance := clampf(hip.distance_to(goal), absf(l1 - l2), l1 + l2)
	var cos_angle := clampf((l1 * l1 + l2 * l2 - distance * distance) / (2.0 * l1 * l2), -1.0, 1.0)
	var limits_min: Vector3 = hinge["min"]
	var limits_max: Vector3 = hinge["max"]
	var bend := clampf(PI - acos(cos_angle), _axis_value(limits_min, axis), _axis_value(limits_max, axis))
	# The locked axes stay on their limit, which is what the PMX limits say.
	skeleton.set_bone_pose_rotation(hinge_bone, Quaternion(axis, bend))
	global_cache.clear()

	# Rotating the whole chain around the hip keeps the solved knee angle and
	# puts the target exactly on the goal.
	var reached := global_pose(skeleton, target, global_cache).origin
	var to_target := reached - hip
	var to_goal := goal - hip
	if to_target.length_squared() < MIN_VECTOR_LENGTH_SQUARED or to_goal.length_squared() < MIN_VECTOR_LENGTH_SQUARED:
		return true
	var align_axis := to_target.cross(to_goal)
	if align_axis.length_squared() < MIN_VECTOR_LENGTH_SQUARED:
		return true
	var align := Quaternion(align_axis.normalized(), to_target.angle_to(to_goal))
	var root_global := global_pose(skeleton, root_bone, global_cache).basis
	var parent := skeleton.get_bone_parent(root_bone)
	var parent_basis := Basis.IDENTITY
	if parent >= 0:
		parent_basis = global_pose(skeleton, parent, global_cache).basis
	var local_basis := parent_basis.inverse() * (Basis(align) * root_global)
	skeleton.set_bone_pose_rotation(root_bone, local_basis.get_rotation_quaternion().normalized())
	global_cache.clear()
	return true


## Returns the only axis a limited link may rotate around, or [constant
## Vector3.ZERO] when the link locks more than one axis.
static func _free_axis(link: Dictionary) -> Vector3:
	var limits_min: Vector3 = link["min"]
	var limits_max: Vector3 = link["max"]
	var free := 0
	var axis := Vector3.ZERO
	if not is_equal_approx(limits_min.x, limits_max.x):
		free += 1
		axis = Vector3.RIGHT
	if not is_equal_approx(limits_min.y, limits_max.y):
		free += 1
		axis = Vector3.UP
	if not is_equal_approx(limits_min.z, limits_max.z):
		free += 1
		axis = Vector3.BACK
	return axis if free == 1 else Vector3.ZERO


static func _axis_value(v: Vector3, axis: Vector3) -> float:
	if axis == Vector3.RIGHT:
		return v.x
	if axis == Vector3.UP:
		return v.y
	return v.z


static func _rotate_towards(skeleton: Skeleton3D, link_bone: int, target: int, goal: Vector3, link: Dictionary, global_cache: Dictionary) -> void:
	var link_position := global_pose(skeleton, link_bone, global_cache).origin
	var to_target := global_pose(skeleton, target, global_cache).origin - link_position
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
		parent_basis = global_pose(skeleton, parent, global_cache).basis
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
