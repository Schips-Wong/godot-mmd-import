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
## mirrored like the mesh. It also carries the scale of the import, which is what
## the gravity and the springs of the file have to be converted with: MMD runs
## Bullet in the units of the PMX file, so simulating a model imported at 0.1
## with the raw values would make it ten times too stiff and too heavy.
##
## The three modes of [enum MMDPhysicsWorld.Mode] are reproduced: a static body
## follows its bone and only collides, a dynamic body is simulated and replaces
## the pose of its bone, and a "dynamic and bone merge" body
## (物理演算とボーン位置合わせ) only supplies the rotation while the bone keeps
## the position the animation gave it.
##
## The bodies live in the space of the scene, so they follow the node of the
## model when it moves; a jump large enough to be a teleport rebuilds them on the
## new pose, see [method reset].
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

## Distance, in the units of the scene, beyond which a movement of the model is
## read as a teleport instead of as the motion of the scene around it.
const TELEPORT_DISTANCE := 0.5
## Rotation, in degrees, beyond which a movement of the model is read as a
## teleport.
const TELEPORT_ANGLE := 45.0
## Number of modifications after the world is built for which the bodies are
## placed on the pose of the scene again, see [method reset].
const ALIGN_FRAMES := 4

## Gravity of the model, in the units of the PMX file: MMD uses 9.8 there, which
## is why its hair and its skirts are tuned for a scale of 0.1. The value is
## converted to the units of the scene with [member physics_scale], so the motion
## keeps the timing of MMD whatever scale the model was imported with.
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

## Length scale of the model relative to the units of the PMX file. The importer
## writes the scale it used in the physics metadata; a model that keeps the
## units of the file leaves it at 1.0.
var physics_scale := 1.0

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
## Whether the body only supplies the rotation of its bone, the position being
## the one of the animation (物理演算とボーン位置合わせ).
var _merge: Array[bool] = []
## Bone -> depth in the hierarchy, used to sort the bodies.
var _depths := {}
var _global_cache := {}
## Transform of the skeleton the bodies were placed in, to follow the model when
## the node moves.
var _last_root := Transform3D.IDENTITY
## Number of modifications left to place the bodies on the pose of the scene
## (see [method reset]).
var _align_frames := 0

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
	_follow_root(skeleton, to_world)
	if _world == null:
		return
	# The world is built when the node enters the tree, before the animation of
	# the scene is applied: the bodies sit on the rest pose of the file there, and
	# a chain would be torn away from the bones it belongs to. Align them on the
	# pose the scene has, which the first frames bring in.
	if _align_frames > 0:
		_align_frames -= 1
		reset()
	# The bones the physics moves are written first: a body that follows a bone
	# may hang on a bone that is itself moved by the physics, and it has to be
	# placed on the final pose.
	_global_cache.clear()
	_apply_dynamic(skeleton, to_world.affine_inverse())
	_global_cache.clear()
	_sync_kinematic(skeleton, to_world)


## Puts every body back on the pose of its bone and stops it. Call it after
## teleporting the model or when switching to a motion that does not continue
## the previous pose, otherwise the hair and the skirt snap into place.
func reset() -> void:
	var skeleton := get_skeleton()
	if skeleton == null or _world == null:
		return
	var to_world := skeleton.global_transform
	_last_root = to_world
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

	physics_scale = maxf(float(data.get("scale", physics_scale)), 0.0001)
	_world = MMDPhysicsWorld.new()
	# The gravity of the file is in the units of the file: convert it, so the
	# hair falls as slowly as in MMD instead of ten times faster on a model
	# imported at 0.1.
	var world_gravity := gravity * physics_scale
	_world.gravity = world_gravity
	_world.physics_scale = physics_scale
	_world.linear_spring_scale = linear_spring_scale
	_world.angular_spring_scale = angular_spring_scale
	_world.spring_damping = spring_damping
	_world.open(world_gravity)

	# The metadata is in the space of the skeleton, the physics runs in the
	# space of the scene.
	var to_world := skeleton.global_transform
	_last_root = to_world
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
		var mode := int(body.get("mode", MMDPhysicsWorld.Mode.DYNAMIC))
		_merge.append(mode == MMDPhysicsWorld.Mode.DYNAMIC_AND_BONE_MERGE)
		if bone < 0:
			continue
		if mode == MMDPhysicsWorld.Mode.STATIC:
			_kinematic.append(index)
		else:
			_dynamic.append(index)

	_kinematic.sort_custom(_sort_by_bone)
	_dynamic.sort_custom(_sort_by_bone)

	for joint in joints:
		var placed: Dictionary = joint.duplicate()
		placed["transform"] = to_world * (joint.get("transform", Transform3D.IDENTITY) as Transform3D)
		_world.add_joint(placed)

	# The bodies were created on the rest pose of the file, which is not the pose
	# the model has when the physics starts (a motion that plays, a pose that was
	# set by hand). Without this the chains would be torn away from their bones
	# and would have to be dragged back, joint by joint, from the rest pose. The
	# animation of the scene only lands on the skeleton a frame or two later, so
	# the alignment is repeated on the next modifications as well.
	_align_frames = ALIGN_FRAMES
	reset()


func _release() -> void:
	if _world != null:
		_world.close()
		_world = null
	_bones.clear()
	_offsets.clear()
	_kinematic.clear()
	_dynamic.clear()
	_merge.clear()
	_depths.clear()
	_global_cache.clear()
	_last_root = Transform3D.IDENTITY
	_align_frames = 0


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


## Moves the bodies with the node of the model. The world of the physics is the
## space of the scene and the bodies are not children of the skeleton, so
## moving the model has to be applied to them by hand, otherwise the hair and
## the skirt stay behind.
##
## [b]Teleport[/b]: the engine cannot place a kinematic body instantly, it moves
## it to the given transform by giving it the speed that goes with it (Jolt:
## [code]MoveKinematic[/code]), so a jump of the model would fire the joints with
## that speed. A movement this large is handled by rebuilding the world on the
## new pose instead.
func _follow_root(skeleton: Skeleton3D, to_world: Transform3D) -> void:
	if to_world == _last_root:
		return
	if not _is_finite(to_world.basis) or not to_world.origin.is_finite():
		return
	var delta := to_world * _last_root.affine_inverse()
	_last_root = to_world
	if not _is_finite(delta.basis) or not delta.origin.is_finite():
		return
	var turned := rad_to_deg(delta.basis.get_rotation_quaternion().get_angle())
	if delta.origin.length() > TELEPORT_DISTANCE or turned > TELEPORT_ANGLE:
		_build(skeleton)
		return
	_world.translate_bodies(delta)


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
		if _merge[index]:
			bone_global = _merge_pose(_global_pose(skeleton, bone), bone_global)
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


## Combines the simulated body with the pose the animation left on the bone for
## the "physics and bone position" mode of the PMX file
## (物理演算とボーン位置合わせ): the body only supplies the rotation, the bone
## keeps the position the animation gave it. That is how MMD drives the parts
## that have to follow the motion and the simulation at the same time.
static func _merge_pose(pose: Transform3D, physics: Transform3D) -> Transform3D:
	return Transform3D(Basis(physics.basis.get_rotation_quaternion().normalized()), pose.origin)


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
