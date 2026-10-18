@tool
class_name MMDPoseSolver extends SkeletonModifier3D

## Applies the two MMD bone rules that plain [Animation] tracks cannot express.
##
## [b]付与 (append / grant)[/b]: a bone inherits a weighted part of another bone's
## animation transform (twist bones, D bones, 肩C, 目戻...). Negative weights
## invert the inherited rotation. Bones are processed in the PMX transform order.
##
## [b]IK[/b]: an IK bone drives its chain (足ＩＫ -> 足/ひざ/足首) with CCD and the
## per-bone angle limits stored in the PMX.
##
## The bone data is written by [PMXImporter] into the [Skeleton3D] metadata
## "mmd_bone_data". The engine runs this node after the [AnimationMixer] and
## restores the unmodified poses afterwards, so the skin uses the solved pose
## while the animation keeps working on a clean pose every frame.

const DATA_META := "mmd_bone_data"

const ROTATION_EPSILON := 0.0001
const IK_TOLERANCE := 0.0005
const MAX_IK_ITERATIONS := 255
const MIN_VECTOR_LENGTH_SQUARED := 1e-10

var _entries: Array = []
var _bone_count := -1

## Cached global transforms, only used while solving (see [method _global_pose]).
var _global_cache: Dictionary = {}


func _process_modification() -> void:
	var skeleton := get_skeleton()
	if skeleton == null:
		return
	if _bone_count != skeleton.get_bone_count():
		_build(skeleton)
	if _entries.is_empty():
		return
	_global_cache.clear()
	for entry in _entries:
		if entry["append"] >= 0:
			_apply_append(skeleton, entry)
		if entry["ik_target"] >= 0:
			_apply_ik(skeleton, entry)


func _skeleton_changed(_old_skeleton: Skeleton3D, new_skeleton: Skeleton3D) -> void:
	_bone_count = -1
	_entries.clear()
	if new_skeleton != null:
		_build(new_skeleton)


func _build(skeleton: Skeleton3D) -> void:
	_bone_count = skeleton.get_bone_count()
	_entries.clear()
	_global_cache.clear()
	if not skeleton.has_meta(DATA_META):
		return
	var data: Variant = skeleton.get_meta(DATA_META)
	if not data is Array:
		return
	for entry in data:
		if entry is Dictionary:
			_entries.append(entry)
	_entries.sort_custom(_sort_entries)


static func _sort_entries(a: Dictionary, b: Dictionary) -> bool:
	var order_a: int = a["order"]
	var order_b: int = b["order"]
	if order_a != order_b:
		return order_a < order_b
	return int(a["bone"]) < int(b["bone"])


# --------------------------------------------------------------------------
# 付与 (append / grant)
# --------------------------------------------------------------------------

func _apply_append(skeleton: Skeleton3D, entry: Dictionary) -> void:
	var bone: int = entry["bone"]
	var target: int = entry["append"]
	if target < 0 or target == bone or target >= skeleton.get_bone_count():
		return
	var weight: float = entry["weight"]
	var rest_target := skeleton.get_bone_rest(target)

	if entry["rot"]:
		var rest_rotation := rest_target.basis.get_rotation_quaternion()
		var target_rotation := skeleton.get_bone_pose_rotation(target)
		var delta := _quaternion_power(rest_rotation.inverse() * target_rotation, weight)
		if not delta.is_equal_approx(Quaternion.IDENTITY):
			var rotation := skeleton.get_bone_pose_rotation(bone) * delta
			skeleton.set_bone_pose_rotation(bone, rotation.normalized())
			_global_cache.clear()

	if entry["pos"]:
		var delta_position := skeleton.get_bone_pose_position(target) - rest_target.origin
		if not delta_position.is_zero_approx():
			var position := skeleton.get_bone_pose_position(bone) + delta_position * weight
			skeleton.set_bone_pose_position(bone, position)
			_global_cache.clear()


## Returns [param q] raised to the power of [param weight]: the rotation angle is
## scaled, so a negative weight produces the inverse rotation.
static func _quaternion_power(q: Quaternion, weight: float) -> Quaternion:
	if is_zero_approx(weight):
		return Quaternion.IDENTITY
	var angle := q.get_angle()
	if is_zero_approx(angle):
		return Quaternion.IDENTITY
	return Quaternion(q.get_axis(), angle * weight)


# --------------------------------------------------------------------------
# IK
# --------------------------------------------------------------------------

func _apply_ik(skeleton: Skeleton3D, entry: Dictionary) -> void:
	var target: int = entry["ik_target"]
	var links: Array = entry["ik_links"]
	if target < 0 or links.is_empty():
		return
	var goal := _global_pose(skeleton, entry["bone"]).origin
	var iterations: int = clampi(int(entry["ik_iter"]), 1, MAX_IK_ITERATIONS)
	for _iteration in range(iterations):
		for link in links:
			var link_bone: int = link["bone"]
			if not _is_ancestor(skeleton, link_bone, target):
				continue
			_rotate_towards(skeleton, link_bone, target, goal, link)
		if _global_pose(skeleton, target).origin.distance_to(goal) <= IK_TOLERANCE:
			break


func _rotate_towards(skeleton: Skeleton3D, link_bone: int, target: int, goal: Vector3, link: Dictionary) -> void:
	var link_position := _global_pose(skeleton, link_bone).origin
	var to_target := _global_pose(skeleton, target).origin - link_position
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
		parent_basis = _global_pose(skeleton, parent).basis
	var rotation := parent_basis.inverse() * Basis(delta) * parent_basis * Basis(skeleton.get_bone_pose_rotation(link_bone))
	var local_rotation := rotation.get_rotation_quaternion()
	if link["limit"]:
		local_rotation = _clamp_rotation(local_rotation, link["min"], link["max"])
	skeleton.set_bone_pose_rotation(link_bone, local_rotation.normalized())
	_global_cache.clear()


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


func _is_ancestor(skeleton: Skeleton3D, ancestor: int, bone: int) -> bool:
	var current := bone
	var depth := 0
	while current >= 0 and depth < skeleton.get_bone_count():
		if current == ancestor:
			return true
		current = skeleton.get_bone_parent(current)
		depth += 1
	return false


# --------------------------------------------------------------------------
# Pose helpers
# --------------------------------------------------------------------------

## Returns the skeleton space pose of [param bone] including the modifications
## done so far. The engine only refreshes the global poses after all modifiers
## ran, so the chain is accumulated here.
func _global_pose(skeleton: Skeleton3D, bone: int) -> Transform3D:
	if _global_cache.has(bone):
		return _global_cache[bone]
	var local := skeleton.get_bone_pose(bone)
	var parent := skeleton.get_bone_parent(bone)
	var result := local
	if parent >= 0:
		result = _global_pose(skeleton, parent) * local
	_global_cache[bone] = result
	return result
