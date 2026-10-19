@tool
class_name MMDPoseSolver extends SkeletonModifier3D

## Applies the 付与 (append / grant) bone rule that plain [Animation] tracks
## cannot express.
##
## A bone inherits a weighted part of another bone's animation transform (twist
## bones, D bones, 肩C, 目戻...). Negative weights invert the inherited rotation.
## Bones are processed in the PMX transform order.
##
## IK is not solved at runtime: [VMDImporter] bakes the IK chains into ordinary
## rotation tracks, see [MMDIKBaker].
##
## The bone data is written by [PMXImporter] into the [Skeleton3D] metadata
## "mmd_bone_data". The engine runs this node after the [AnimationMixer] and
## restores the unmodified poses afterwards, so the skin uses the solved pose
## while the animation keeps working on a clean pose every frame.

const DATA_META := "mmd_bone_data"

var _entries: Array = []
var _bone_count := -1


func _process_modification() -> void:
	var skeleton := get_skeleton()
	if skeleton == null:
		return
	if _bone_count != skeleton.get_bone_count():
		_build(skeleton)
	for entry in _entries:
		apply_append(skeleton, entry)


func _skeleton_changed(_old_skeleton: Skeleton3D, new_skeleton: Skeleton3D) -> void:
	_bone_count = -1
	_entries.clear()
	if new_skeleton != null:
		_build(new_skeleton)


func _build(skeleton: Skeleton3D) -> void:
	_bone_count = skeleton.get_bone_count()
	_entries.clear()
	if not skeleton.has_meta(DATA_META):
		return
	var data: Variant = skeleton.get_meta(DATA_META)
	if not data is Array:
		return
	for entry in data:
		if entry is Dictionary and int(entry["append"]) >= 0:
			_entries.append(entry)
	_entries.sort_custom(sort_entries)


static func sort_entries(a: Dictionary, b: Dictionary) -> bool:
	var order_a: int = a["order"]
	var order_b: int = b["order"]
	if order_a != order_b:
		return order_a < order_b
	return int(a["bone"]) < int(b["bone"])


# --------------------------------------------------------------------------
# 付与 (append / grant)
# --------------------------------------------------------------------------

## Applies the append rule of one bone. Also used by [MMDIKBaker] to recreate
## the runtime pose while solving IK at import time.
static func apply_append(skeleton: Skeleton3D, entry: Dictionary) -> void:
	var bone: int = entry["bone"]
	var target: int = entry["append"]
	if target < 0 or target == bone or target >= skeleton.get_bone_count() or bone >= skeleton.get_bone_count():
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

	if entry["pos"]:
		var delta_position := skeleton.get_bone_pose_position(target) - rest_target.origin
		if not delta_position.is_zero_approx():
			var position := skeleton.get_bone_pose_position(bone) + delta_position * weight
			skeleton.set_bone_pose_position(bone, position)


## Returns [param q] raised to the power of [param weight]: the rotation angle is
## scaled, so a negative weight produces the inverse rotation.
static func _quaternion_power(q: Quaternion, weight: float) -> Quaternion:
	if is_zero_approx(weight):
		return Quaternion.IDENTITY
	var angle := q.get_angle()
	if is_zero_approx(angle):
		return Quaternion.IDENTITY
	return Quaternion(q.get_axis(), angle * weight)
