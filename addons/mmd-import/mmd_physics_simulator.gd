@tool
class_name MMDPhysicsSimulator extends SkeletonModifier3D

## Runs the MMD physics (剛体 + ジョイント) of the model.
##
## MMD models carry their own physics: hair, skirts, breasts and accessories are
## rigid bodies connected by 6 DOF spring joints. Some bodies are static and only
## act as colliders for the others, the rest are simulated and then move the bone
## they belong to. This node reproduces that with [MMDPhysicsWorld]: static
## bodies follow their bone, the world simulates, and the result is written back
## into the pose of the simulated bones.
##
## The data is written by [PMXImporter] into the [Skeleton3D] metadata
## "mmd_physics_data", which holds the bodies and the joints already scaled and
## mirrored like the mesh.
##
## The node has to run after [MMDPoseSolver]: the bones moved by 付与 have to be
## final before the static bodies are placed. [PMXImporter] adds it in that
## order, and like every [SkeletonModifier3D] it runs after the
## [AnimationMixer], so the animation of a simulated bone is replaced by the
## physics result.
##
## [b]Tip[/b]: the physics is only simulated while the scene runs, not in the
## editor, where the [PhysicsServer3D] does not step. Use [method reset] to place
## the bodies on the current pose again (teleport, scene reload, cut to a
## different motion).
## [codeblock]
## $Model/Bone/MMDPhysics.reset()
## [/codeblock]

const DATA_META := "mmd_physics_data"

## Gravity of the model, in the units of the scene. Tune it together with the
## import scale of the model and the spring scales below: MMD itself has no such
## setting, which is why its models are tuned for a 0.1 import scale.
@export_range(0.0, 98.0, 0.1) var gravity := 9.8
## Multiplier of the linear springs of the joints. The stiffness of a PMX joint
## is a Bullet parameter; the solvers of Godot weight gravity, mass and
## stiffness differently, so a value of 1.0 is a starting point, not a law.
@export_range(0.0, 100.0, 0.01) var linear_spring_scale := 1.0
## Multiplier of the angular springs of the joints, the ones that keep the hair
## and the skirt hanging in place.
@export_range(0.0, 100.0, 0.01) var angular_spring_scale := 1.0
## Damping ratio of every joint spring (Bullet, the engine of MMD, uses 0.5).
@export_range(0.0, 1.0, 0.01) var spring_damping := 0.75

var _world: MMDPhysicsWorld = null
var _bone_count := -1
## Bone of every body, -1 when the body only collides.
var _bones := PackedInt32Array()
## Body -> bone offset, in the space of the skeleton (see [method _build]).
var _offsets: Array[Transform3D] = []
## Bodies that follow a bone, and bodies that move a bone, both sorted by bone
## depth so parents are handled before their children.
var _kinematic: Array[int] = []
var _dynamic: Array[int] = []
## Bone -> depth in the hierarchy, used to sort the bodies.
var _depths := {}
var _global_cache := {}

func _process_modification() -> void:
	# The editor does not step the PhysicsServer3D: simulating there would only
	# park the bones on the rest pose in the viewport.
	if Engine.is_editor_hint():
		return
	var skeleton := get_skeleton()
	if skeleton == null:
		return
	if _bone_count != skeleton.get_bone_count():
		_build(skeleton)
	if _world == null:
		return

	var to_world := skeleton.global_transform
	_global_cache.clear()
	_sync_kinematic(skeleton, to_world)
	_apply_dynamic(skeleton, to_world.affine_inverse())


## Puts every body back on the pose of its bone and stops it. Call it after
## teleporting the model or when switching to a motion that does not continue
## the previous pose, otherwise the hair and the skirt snap into place.
func reset() -> void:
	var skeleton := get_skeleton()
	if skeleton == null or _world == null:
		return
	var to_world := skeleton.global_transform
	_global_cache.clear()
	for index in range(_world.body_count()):
		var bone: int = _bones[index]
		if bone < 0:
			continue
		_world.set_body_transform(index, to_world * (_offsets[index] * _global_pose(skeleton, bone)))
		_world.reset_body_velocity(index)
	_global_cache.clear()


## Returns the world that simulates the model, for impulses (morphs, wind) and
## for debugging. Body indices follow the "mmd_physics_data" metadata.
func get_world() -> MMDPhysicsWorld:
	return _world


## Returns the index of the rigid body attached to [param bone], or -1.
func find_body_of_bone(bone: int) -> int:
	for index in range(_bones.size()):
		if _bones[index] == bone:
			return index
	return -1


func _skeleton_changed(_old_skeleton: Skeleton3D, _new_skeleton: Skeleton3D) -> void:
	_release()
	_bone_count = -1


func _exit_tree() -> void:
	_release()
	_bone_count = -1


# --------------------------------------------------------------------------
# Build
# --------------------------------------------------------------------------

func _build(skeleton: Skeleton3D) -> void:
	_release()
	_bone_count = skeleton.get_bone_count()
	var data := _physics_data(skeleton)
	if data.is_empty():
		return
	var bodies: Array = data.get("rigid_bodies", [])
	var joints: Array = data.get("joints", [])
	if bodies.is_empty():
		return

	_world = MMDPhysicsWorld.new()
	_world.gravity = gravity
	_world.linear_spring_scale = linear_spring_scale
	_world.angular_spring_scale = angular_spring_scale
	_world.spring_damping = spring_damping
	_world.open(gravity)

	# The metadata is in the space of the skeleton, the physics runs in the
	# space of the scene.
	var to_world := skeleton.global_transform
	_depths.clear()
	for body in bodies:
		var bone := _valid_bone(skeleton, int(body.get("bone", -1)))
		_bones.append(bone)
		_offsets.append(_offset(skeleton, body, bone))
		if bone >= 0 and not _depths.has(bone):
			_depths[bone] = _depth_of(skeleton, bone)
		# The body keeps the offset to its bone for the whole simulation: the
		# rigid body of a PMX file is placed once, then follows the bone.
		var placed: Dictionary = body.duplicate()
		placed["transform"] = to_world * (body.get("transform", Transform3D.IDENTITY) as Transform3D)
		var index := _world.add_rigid_body(placed)
		if bone < 0:
			continue
		if int(body.get("mode", MMDPhysicsWorld.Mode.DYNAMIC)) == MMDPhysicsWorld.Mode.STATIC:
			_kinematic.append(index)
		else:
			_dynamic.append(index)

	_kinematic.sort_custom(_sort_by_bone)
	_dynamic.sort_custom(_sort_by_bone)

	for joint in joints:
		var placed: Dictionary = joint.duplicate()
		placed["transform"] = to_world * (joint.get("transform", Transform3D.IDENTITY) as Transform3D)
		_world.add_joint(placed)


func _release() -> void:
	if _world != null:
		_world.close()
		_world = null
	_bones.clear()
	_offsets.clear()
	_kinematic.clear()
	_dynamic.clear()
	_depths.clear()
	_global_cache.clear()


## Returns the physics data of the skeleton.
func _physics_data(skeleton: Skeleton3D) -> Dictionary:
	if not skeleton.has_meta(DATA_META):
		return {}
	var data: Variant = skeleton.get_meta(DATA_META)
	if not data is Dictionary:
		return {}
	return data


## Returns the bone of a body, or -1 when the index of the file is unusable.
func _valid_bone(skeleton: Skeleton3D, bone: int) -> int:
	if bone < 0 or bone >= skeleton.get_bone_count():
		return -1
	return bone


## Returns how many bones are above [param bone].
func _depth_of(skeleton: Skeleton3D, bone: int) -> int:
	var depth := 0
	var current := bone
	while current >= 0 and depth < skeleton.get_bone_count():
		current = skeleton.get_bone_parent(current)
		depth += 1
	return depth


## Returns the transform that moves a body with its bone: the body sits at
## [code]offset * bone_global_pose[/code] and the bone ends up at
## [code]offset⁻¹ * body[/code], the same relation MMD uses.
func _offset(skeleton: Skeleton3D, body: Dictionary, bone: int) -> Transform3D:
	if bone < 0:
		return Transform3D.IDENTITY
	var rest: Transform3D = body.get("transform", Transform3D.IDENTITY)
	return rest * skeleton.get_bone_global_rest(bone).affine_inverse()


## Orders the bodies so a bone is always written before the bones below it: the
## parent pose the local pose is derived from has to be final.
func _sort_by_bone(a: int, b: int) -> bool:
	var depth_a: int = _depths.get(_bones[a], 0)
	var depth_b: int = _depths.get(_bones[b], 0)
	if depth_a != depth_b:
		return depth_a < depth_b
	return _bones[a] < _bones[b]


# --------------------------------------------------------------------------
# Bone <-> body
# --------------------------------------------------------------------------

## Moves the bodies that only collide with the animated pose of their bone
## (付与 and IK included, because this node runs after [MMDPoseSolver]).
func _sync_kinematic(skeleton: Skeleton3D, to_world: Transform3D) -> void:
	for index in _kinematic:
		var bone: int = _bones[index]
		_world.set_body_transform(index, to_world * (_offsets[index] * _global_pose(skeleton, bone)))


## Writes the simulated bodies back into the pose of their bone.
##
## The bodies are processed top down, so a bone that moves another one (the hair,
## the skirt, a joint chain) is handled first and its result is remembered: the
## pose of the bones that are not moved by the physics is the animated one, which
## [method _global_pose] resolves once per frame.
func _apply_dynamic(skeleton: Skeleton3D, to_skeleton: Transform3D) -> void:
	var written := {}
	for index in _dynamic:
		var bone: int = _bones[index]
		var body := to_skeleton * _world.get_body_transform(index)
		var bone_global := _offsets[index].affine_inverse() * body
		# A body that escaped the simulation (huge impulse, degenerate shape)
		# would otherwise write NaN into the pose and break the whole model.
		if not bone_global.origin.is_finite() or not _is_finite(bone_global.basis):
			continue
		var parent := skeleton.get_bone_parent(bone)
		var parent_global := Transform3D.IDENTITY
		if parent >= 0:
			parent_global = written[parent] if written.has(parent) else _global_pose(skeleton, parent)
		var local := parent_global.affine_inverse() * bone_global
		if not local.origin.is_finite() or not _is_finite(local.basis):
			continue
		skeleton.set_bone_pose_position(bone, local.origin)
		skeleton.set_bone_pose_rotation(bone, local.basis.get_rotation_quaternion().normalized())
		written[bone] = bone_global


## Returns the pose of [param bone] in the space of the skeleton, as the
## animation (and the modifiers that ran before this one) left it. The engine
## only refreshes the global poses after all modifiers ran, so the chain is
## accumulated here.
func _global_pose(skeleton: Skeleton3D, bone: int) -> Transform3D:
	if _global_cache.has(bone):
		return _global_cache[bone]
	var pose := skeleton.get_bone_pose(bone)
	var parent := skeleton.get_bone_parent(bone)
	var result := pose
	if parent >= 0:
		result = _global_pose(skeleton, parent) * pose
	_global_cache[bone] = result
	return result


static func _is_finite(basis: Basis) -> bool:
	return basis.x.is_finite() and basis.y.is_finite() and basis.z.is_finite()
