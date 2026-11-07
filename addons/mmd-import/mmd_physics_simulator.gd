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
## [b]Tuning[/b]: the physics of a PMX file is authored for Bullet and is often
## broken on its own (masses of hundreds of thousands of kilos per cubic unit,
## joints free to slide, no spring at all); see [MMDPhysicsStrategy] for the
## switches that repair it and [member preset] for the combinations that work on
## real models. The preset is layered over the parameters exported below, so a
## model whose hair was hand tuned keeps its tuning.
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
## Distance, in the units of the scene, a body may travel away from its bone
## before it is read as lost and put back. A chain of hair or a skirt swings well
## inside of it; a body the solver threw out is hundreds of units away.
const MAX_DRIFT := 3.0
## Speed, in units per second, and spin, in radians per second, a body on its way
## out is held to. Hair and skirts of a model stay far below.
const MAX_SPEED := 20.0
const MAX_SPIN := 40.0

## Gravity of the model, in the units of the PMX file: MMD uses 9.8 there, which
## is why its hair and its skirts are tuned for a scale of 0.1. The value is
## converted to the units of the scene with [member physics_scale], so the motion
## keeps the timing of MMD whatever scale the model was imported with.
@export_range(0.0, 98.0, 0.1) var gravity := 9.8

@export_group("优化策略")
## 一套修好 PMX 文件常见缺陷的开关，见 [MMDPhysicsStrategy]。预设会叠在下面
## 导出的参数之上，所以已经手工调好的场景（弹簧倍率等）不会因为换策略而丢失；
## 预设只在它自己有意见的开关上覆盖导出值。选
## [constant MMDPhysicsStrategy.Preset.CUSTOM] 时只用下面导出的参数。
##
## 改完这里（或下面的参数）要重新构建世界才生效，见 [method rebuild]；场景
## 启动时自然就是新值。
@export var preset: MMDPhysicsStrategy.Preset = MMDPhysicsStrategy.Preset.CUSTOM
## 把关节的平移限位锁死在 0（不论文件怎么写）。裙子、头发在文件里常常允许
## 滑动几毫米到几厘米，解算器会把每一步的误差留在那里，链子就会越来越长。
@export var lock_joint_translation := false
## 关闭关节的线位移弹簧。平移锁死之后它已经没有作用，而一个很硬的线弹簧会
## 和锁死角力。
@export var disable_linear_spring := false
## 把刚体的弹性（反弹）全部设为 0，不让碰撞把速度还给链子。
@export var zero_restitution := false
## 刚体质量的计算方式，见 [enum MMDPhysicsStrategy.MassMode]。文件里的质量
## 常常是错的（测试模型里有 3cm 的头发 15 公斤），同一条链上的质量不一致是
## 抖动和炸开的主要原因。
@export var mass_mode: MMDPhysicsStrategy.MassMode = MMDPhysicsStrategy.MassMode.FILE
## [constant MMDPhysicsStrategy.MassMode.UNIFORM] 下每个刚体的质量。
@export_range(0.001, 100.0, 0.001) var uniform_mass := 1.0
## [constant MMDPhysicsStrategy.MassMode.DENSITY] 下的密度（kg/场景单位³，
## 1000 相当于水）。
@export_range(1.0, 100000.0, 1.0) var mass_density := 1000.0
## 乘在刚体阻尼（文件值和基础阻尼都算）上的倍率。阻尼是把解算器无法收敛的
## 抖动变成会停下来的摆动的唯一手段。
@export_range(0.0, 8.0, 0.01) var damping_scale := 1.0
## 允许刚体休眠。MMD 是关掉的（骨骼一动，链子必须立刻反应），只有几百个刚体
## 的模型为了性能才值得打开。
@export var allow_sleep := false
## 对模拟刚体开启连续碰撞检测。头发刚体只有几厘米却动得很快，没有它会穿进
## 不该进的身体里，然后被接触以当前速度弹开。
@export var continuous_collision := true
## 每帧的保护：跑离骨骼 3 个场景单位以上的刚体被放回骨骼的位置（速度清零），
## 已经跑偏一半的刚体被限速到 [constant MAX_SPEED]。关掉它可以省掉每帧的检查，
## 但一个被甩出链子的刚体就再也回不来了。
@export var stability_guard := true
## 所有模拟刚体的速度上限（场景单位/秒），0 表示不限制。
##
## 一个无法收敛的接触（文件里互相重叠的碰撞体、远比它代表的部位重的刚体）
## 会给出一个速度，关节几步之内就把它传遍整条链，链子就飞了。限速要每帧读
## 一次所有刚体的速度（超限时才写），但能挡住伤害扩散。它治不了太软的关节，
## 只是不让一颗坏刚体拖垮整个模型。按 0.1 导入的模型，5 左右比较合适。
@export_range(0.0, 50.0, 0.1) var max_speed := 0.0

@export_group("关节弹簧")
## Multiplier of the linear springs of the joints. The stiffness of a PMX joint
## is a Bullet parameter; the solvers of Godot weight gravity, mass and
## stiffness differently, so a value of 1.0 is a starting point, not a law.
@export_range(0.0, 100.0, 0.01) var linear_spring_scale := 1.0
## Multiplier of the angular springs of the joints, the ones that keep the hair
## and the skirt hanging in place.
@export_range(0.0, 100.0, 0.01) var angular_spring_scale := 1.0
## Damping ratio of every joint spring (Bullet, the engine of MMD, uses 0.5).
@export_range(0.0, 1.0, 0.01) var spring_damping := 0.75
## Multiplier of the spring that holds the joints of small limits together, 0
## turns it off. A joint of the file that allows only a few degrees and has no
## spring of its own is meant to keep its part in place; the limit alone cannot,
## see [method MMDPhysicsWorld._configure_joint].
@export_range(0.0, 4.0, 0.05) var support_spring_scale := 1.0

## Length scale of the model relative to the units of the PMX file. The importer
## writes the scale it used in the physics metadata; a model that keeps the
## units of the file leaves it at 1.0.
var physics_scale := 1.0

var _world: MMDPhysicsWorld = null
## Tuning the world was built with, see [method _resolve_strategy].
var _strategy := MMDPhysicsStrategy.new()
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
## Bodies put back on their bone, for tuning and for tests.
var _resets := 0

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
	if _strategy.max_speed > 0.0:
		_world.clamp_speeds(_strategy.max_speed, MAX_SPIN)


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


## Returns the tuning the world runs with: the preset of the node applied on top
## of the parameters it exports. Read only, a change needs [method rebuild].
func get_strategy() -> MMDPhysicsStrategy:
	return _strategy


## Builds the bodies and the joints again, picking up the parameters of the node
## (the preset and the switches) and the metadata of the skeleton. The pose is
## kept: the bodies are placed on it, see [method reset]. Call it after changing
## the tuning while the scene runs.
func rebuild() -> void:
	var skeleton := get_skeleton()
	if skeleton == null:
		return
	_global_cache.clear()
	_build(skeleton)


## Returns the tuning to build the world with: the switches of [member preset]
## applied on top of the parameters of the node.
func _resolve_strategy() -> MMDPhysicsStrategy:
	var base := MMDPhysicsStrategy.new()
	base.lock_translation = lock_joint_translation
	base.disable_linear_spring = disable_linear_spring
	base.linear_spring_scale = linear_spring_scale
	base.angular_spring_scale = angular_spring_scale
	base.spring_damping = spring_damping
	base.support_spring_scale = support_spring_scale
	base.mass_mode = mass_mode
	base.uniform_mass = uniform_mass
	base.density = mass_density
	base.damping_scale = damping_scale
	base.zero_restitution = zero_restitution
	base.continuous_collision = continuous_collision
	base.allow_sleep = allow_sleep
	base.stability_guard = stability_guard
	base.max_speed = max_speed
	return MMDPhysicsStrategy.create(preset, base)


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
	_strategy = _resolve_strategy()
	_world.gravity = world_gravity
	_world.physics_scale = physics_scale
	_world.strategy = _strategy
	# The masks of the file are not symmetrical, and Godot reads them as an "or":
	# the pairs are reduced to the ones MMD has, see build_group_rules.
	_world.group_rules = MMDPhysicsWorld.build_group_rules(bodies)
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

	print_verbose("MMD: physics %s (%d bodies, %d joints)" % [_strategy.describe(), bodies.size(), joints.size()])

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
	var to_world := to_skeleton.affine_inverse()
	for index in _dynamic:
		var bone: int = _bones[index]
		var body := to_skeleton * _world.get_body_transform(index)
		var bone_global := _offsets[index].affine_inverse() * body
		# A body of the file can be a few centimeter big and weigh ten kilos, and a
		# contact it should not have pushes it away for good; once it is that far
		# from its bone the model is lost anyway, so it is put back on the pose the
		# animation gives the bone. Normal hair and skirt motion stays well inside.
		if _strategy.stability_guard:
			var animated := _global_pose(skeleton, bone)
			var drift := bone_global.origin.distance_to(animated.origin)
			if drift > MAX_DRIFT:
				_world.set_body_transform(index, to_world * (_offsets[index] * animated))
				_world.reset_body_velocity(index)
				bone_global = animated
				_resets += 1
			elif drift > MAX_DRIFT * 0.5:
				# Already on its way out: brake before it leaves, so the pose is
				# held instead of being put back in one visible jump.
				_world.clamp_body_velocity(index, MAX_SPEED, MAX_SPIN)
		# A body that escaped the simulation (huge impulse, degenerate shape)
		# would otherwise write NaN into the pose and break the whole model.
		if not bone_global.origin.is_finite() or not _is_finite(bone_global.basis):
			continue
		var parent := skeleton.get_bone_parent(bone)
		var parent_global := Transform3D.IDENTITY
		if parent >= 0:
			parent_global = written[parent] if written.has(parent) else _global_pose(skeleton, parent)
		var local := parent_global.affine_inverse() * bone_global
		if _merge[index]:
			# 物理演算とボーン位置合わせ: the body only supplies the rotation. The
			# position is the one the animation gives the bone <b>in its parent</b>,
			# not in the model: keeping the world position would cancel the rotation
			# of the parent and pin the part in place, which is what a skirt of that
			# mode looked like.
			local = Transform3D(local.basis, skeleton.get_bone_pose(bone).origin)
		if not local.origin.is_finite() or not _is_finite(local.basis):
			continue
		skeleton.set_bone_pose_position(bone, local.origin)
		skeleton.set_bone_pose_rotation(bone, local.basis.get_rotation_quaternion().normalized())
		written[bone] = parent_global * local


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
