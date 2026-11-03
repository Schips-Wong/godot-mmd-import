class_name MMDPhysicsWorld extends RefCounted

## Physics world of the rigid bodies (剛体) and joints (ジョイント) of one MMD
## model.
##
## MMD simulates every model in its own Bullet world: the bodies collide with the
## bodies of the same model only, following the collision groups of the PMX file,
## and nothing else in the scene is affected. The same is done here with a
## private [PhysicsServer3D] space owned by this class. The engine steps it with
## the configured physics tick rate, but no viewport, body or physics query of
## the scene can see it.
##
## The bodies are created from the "mmd_physics_data" metadata written by
## [PMXImporter] and are driven by [MMDPhysicsSimulator], which connects them
## with the bones of the skeleton:
## [codeblock]
## var world := MMDPhysicsWorld.new()
## world.open(9.8)
## world.add_rigid_body(body_data)
## world.add_joint(joint_data)
## world.set_body_transform(0, transform)
## [/codeblock]
##
## [b]Collision groups[/b]: a body of PMX group N is placed on collision layer
## N and collides with the groups listed in its collision mask, which is the
## same meaning the field has in MMD. Godot collides two bodies when either mask
## lists the layer of the other, while Bullet requires both, so a model with
## one-way masks collides a bit more than in MMD.
##
## [b]Scale[/b]: the data is already scaled and mirrored by the importer, so the
## bodies are created in the same space as the bones of the [Skeleton3D].

## Shape ids, the same order as [enum PMXFile.PMXShape].
enum Shape {
	SPHERE,
	BOX,
	CAPSULE,
}

## Same meaning as in the PMX file: 0 is static (a bone moves it), the other
## values are simulated by the world. [constant DYNAMIC_AND_BONE_MERGE] is
## simulated exactly like [constant DYNAMIC]: the blend MMD does with the bone
## transform is not applied, which is also what the reference implementation
## does.
enum Mode {
	STATIC,
	DYNAMIC,
	DYNAMIC_AND_BONE_MERGE,
}

const MIN_SIZE := 0.0001
const MIN_MASS := 0.001
## PMX files use 16 collision groups.
const GROUP_COUNT := 16

## Stiffness multiplier applied to the linear springs of the joints, see
## [member MMDPhysicsSimulator.linear_spring_scale].
var linear_spring_scale := 1.0
## Stiffness multiplier applied to the angular springs of the joints.
var angular_spring_scale := 1.0
## Damping ratio of every joint spring; Bullet, the engine MMD uses, applies
## 0.5 by default.
var spring_damping := 0.5
## Gravity of the space, in the units of the imported model.
var gravity := 9.8
## Length scale of the model relative to the units of the PMX file (the importer
## uses 0.1). MMD runs Bullet in the units of the file, so a value that carries a
## length has to follow the scale of the model: the stiffness of an angular
## spring is a torque per radian, a torque scales with a length squared, so the
## angular springs are converted with the square of this value. Linear springs
## and damping have no length dimension and are used as they are.
var physics_scale := 1.0
## Multiplier of the support spring, 0 turns it off.
var support_spring_scale := 1.0
## Sum of the angular limits, in radians, below which a joint without a spring
## counts as "hold this part in place" and gets the support spring.
const LIMIT_SPRING_RANGE := 0.6
## Stiffness and damping of that support spring, in the units of the PMX file
## (scaled with the model like every other torque, see [member physics_scale]).
## A stiff spring needs a lot of damping: a light spring on a long feather is
## under damped and feeds the wobble it is there to stop.
const LIMIT_SPRING_STIFFNESS := 100000.0
const LIMIT_SPRING_DAMPING := 10000.0

var _space := RID()
var _bodies: Array[RID] = []
var _joints: Array[RID] = []
var _dynamic: Array[bool] = []
## Last transform written to each body, kept on this side because reading it
## back is deferred when the physics runs on its own thread.
var _transforms: Array[Transform3D] = []
var _shape_cache := {}
var _open := false


## Creates the private physics space and starts stepping it.
func open(gravity_value: float = 9.8) -> void:
	if _open:
		return
	gravity = gravity_value
	_space = PhysicsServer3D.space_create()
	set_gravity(gravity)
	PhysicsServer3D.space_set_active(_space, true)
	_open = true


## Sets the gravity of the space. Godot applies the gravity of a space through
## its default area, which [method PhysicsServer3D.area_set_param] also accepts
## by space RID.
func set_gravity(value: float) -> void:
	gravity = value
	if not _space.is_valid():
		return
	PhysicsServer3D.area_set_param(_space, PhysicsServer3D.AREA_PARAM_GRAVITY, absf(value))
	PhysicsServer3D.area_set_param(_space, PhysicsServer3D.AREA_PARAM_GRAVITY_VECTOR, Vector3.DOWN)


## Frees every body, joint and shape of the world as well as the space itself.
func close() -> void:
	if not _open:
		return
	for joint in _joints:
		if joint.is_valid():
			PhysicsServer3D.free_rid(joint)
	_joints.clear()
	for body in _bodies:
		if body.is_valid():
			PhysicsServer3D.free_rid(body)
	_bodies.clear()
	_dynamic.clear()
	_transforms.clear()
	for key in _shape_cache:
		PhysicsServer3D.free_rid(_shape_cache[key])
	_shape_cache.clear()
	PhysicsServer3D.free_rid(_space)
	_space = RID()
	_open = false


func is_open() -> bool:
	return _open


func body_count() -> int:
	return _bodies.size()


func joint_count() -> int:
	return _joints.size()


## Returns the index of the rigid body described by [param data], a dictionary
## of the "mmd_physics_data" metadata. Bodies added while the world is closed are
## skipped, so the indices always match the metadata.
func add_rigid_body(data: Dictionary) -> int:
	var index := _bodies.size()
	if not _open:
		return index
	var mode := int(data.get("mode", Mode.DYNAMIC))
	var dynamic := mode != Mode.STATIC
	var group := clampi(int(data.get("group", 0)), 0, GROUP_COUNT - 1)
	var mask := int(data.get("mask", 0)) & ((1 << GROUP_COUNT) - 1)

	var body := PhysicsServer3D.body_create()
	PhysicsServer3D.body_set_mode(body, PhysicsServer3D.BODY_MODE_RIGID if dynamic else PhysicsServer3D.BODY_MODE_KINEMATIC)
	PhysicsServer3D.body_set_param(body, PhysicsServer3D.BODY_PARAM_MASS, maxf(float(data.get("mass", 1.0)), MIN_MASS))
	PhysicsServer3D.body_set_param(body, PhysicsServer3D.BODY_PARAM_FRICTION, maxf(float(data.get("friction", 0.5)), 0.0))
	PhysicsServer3D.body_set_param(body, PhysicsServer3D.BODY_PARAM_BOUNCE, clampf(float(data.get("restitution", 0.0)), 0.0, 1.0))
	PhysicsServer3D.body_set_param(body, PhysicsServer3D.BODY_PARAM_LINEAR_DAMP, maxf(float(data.get("linear_damp", 0.0)), 0.0))
	PhysicsServer3D.body_set_param(body, PhysicsServer3D.BODY_PARAM_ANGULAR_DAMP, maxf(float(data.get("angular_damp", 0.0)), 0.0))
	# MMD disables deactivation (Bullet: DisableDeactivation, zero sleep
	# thresholds): a hair or skirt chain has to react the moment the body moves,
	# not after it woke up again.
	PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_CAN_SLEEP, false)

	PhysicsServer3D.body_set_collision_layer(body, 1 << group)
	PhysicsServer3D.body_set_collision_mask(body, mask)
	PhysicsServer3D.body_add_shape(body, _shape_for(int(data.get("shape", Shape.SPHERE)), data.get("size", Vector3.ONE)))
	var transform: Transform3D = data.get("transform", Transform3D.IDENTITY)
	PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_TRANSFORM, transform)
	PhysicsServer3D.body_set_space(body, _space)

	_bodies.append(body)
	_dynamic.append(dynamic)
	_transforms.append(transform)
	return index


## Adds the joint described by [param data] and returns its index. A joint that
## references a missing body is skipped.
func add_joint(data: Dictionary) -> int:
	var index := _joints.size()
	var body_a := int(data.get("body_a", -1))
	var body_b := int(data.get("body_b", -1))
	if not _open:
		return index
	if body_a < 0 or body_b < 0 or body_a >= _bodies.size() or body_b >= _bodies.size():
		return index

	var frame: Transform3D = data.get("transform", Transform3D.IDENTITY)
	var joint := PhysicsServer3D.joint_create()
	# Like Bullet, the frames are given in the local space of each body, so the
	# joint stays valid when the whole model moves.
	PhysicsServer3D.joint_make_generic_6dof(joint, _bodies[body_a], _local_frame(body_a, frame), _bodies[body_b], _local_frame(body_b, frame))
	_configure_joint(joint, data)
	# A chain of bodies overlaps its neighbour at every joint on purpose, so the
	# pair a joint holds together must not collide with itself: the solver would
	# spend every step pushing the two halves of the joint apart. This is how a 6
	# DOF chain is built in Bullet (disableCollisionsBetweenLinkedBodies).
	PhysicsServer3D.joint_disable_collisions_between_bodies(joint, true)
	_joints.append(joint)
	return index


## Returns the transform of the body, in the space of the world. The value comes
## from the last physics step.
func get_body_transform(index: int) -> Transform3D:
	if index < 0 or index >= _bodies.size():
		return Transform3D.IDENTITY
	return PhysicsServer3D.body_get_state(_bodies[index], PhysicsServer3D.BODY_STATE_TRANSFORM)


## Places the body, in the space of the world. Static (kinematic) bodies are
## moved this way every frame to follow their bone, dynamic bodies are only moved
## by [method MMDPhysicsSimulator.reset].
func set_body_transform(index: int, transform: Transform3D) -> void:
	if index < 0 or index >= _bodies.size():
		return
	_transforms[index] = transform
	PhysicsServer3D.body_set_state(_bodies[index], PhysicsServer3D.BODY_STATE_TRANSFORM, transform)


## Moves every body by [param delta], velocities included: the world of the
## model is the space of the scene, so the bodies have to follow when the node
## of the model moves. The joint frames are stored in the local space of the
## bodies, so they stay valid.
func translate_bodies(delta: Transform3D) -> void:
	if not _open:
		return
	for index in range(_bodies.size()):
		var body: RID = _bodies[index]
		var moved: Transform3D = delta * PhysicsServer3D.body_get_state(body, PhysicsServer3D.BODY_STATE_TRANSFORM)
		PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_TRANSFORM, moved)
		if _dynamic[index]:
			var linear: Vector3 = PhysicsServer3D.body_get_state(body, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY)
			var angular: Vector3 = PhysicsServer3D.body_get_state(body, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY)
			PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY, delta.basis * linear)
			PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY, delta.basis * angular)
		else:
			# The kinematic bodies are placed from their bone again next frame;
			# keeping the last written transform in sync keeps the joint frames
			# meaningful if one is ever added to the world after this call.
			_transforms[index] = moved


## Stops the body and removes the rotation it gathered until now.
func reset_body_velocity(index: int) -> void:
	if index < 0 or index >= _bodies.size() or not _dynamic[index]:
		return
	PhysicsServer3D.body_set_state(_bodies[index], PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY, Vector3.ZERO)
	PhysicsServer3D.body_set_state(_bodies[index], PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY, Vector3.ZERO)


## Applies an impulse at the body origin, for gameplay code and for the impulse
## morphs (物理演算) of the file.
func apply_impulse(index: int, impulse: Vector3) -> void:
	if index < 0 or index >= _bodies.size() or not _dynamic[index]:
		return
	PhysicsServer3D.body_apply_impulse(_bodies[index], impulse)


# --------------------------------------------------------------------------
# Internals
# --------------------------------------------------------------------------

## Returns the joint frame in the space of body [param index], which is what
## [method PhysicsServer3D.joint_make_generic_6dof] expects. The transform the
## body was created with is used instead of reading it back, so the frames are
## right even when the physics server does not answer before its next step.
func _local_frame(index: int, frame: Transform3D) -> Transform3D:
	return _transforms[index].affine_inverse() * frame


## Sets the limits and the springs of a PMX joint. Every axis carries a linear
## and an angular limit, plus an optional spring with stiffness
## (spring_translate_factor / spring_rotate_factor of the file), the same
## combination Bullet's btGeneric6DofSpringConstraint offers.
func _configure_joint(joint: RID, data: Dictionary) -> void:
	var linear_min: Vector3 = data.get("linear_min", Vector3.ZERO)
	var linear_max: Vector3 = data.get("linear_max", Vector3.ZERO)
	var linear_spring: Vector3 = data.get("linear_spring", Vector3.ZERO)
	var angular_min: Vector3 = data.get("angular_min", Vector3.ZERO)
	var angular_max: Vector3 = data.get("angular_max", Vector3.ZERO)
	var angular_spring: Vector3 = data.get("angular_spring", Vector3.ZERO)

	# A joint of the file can store a lower limit above its upper one, which is
	# an authoring mistake MMD survives by clamping somewhere inside the range.
	# The solvers of Godot read that as an invalid interval and drop the limit
	# altogether (the axis becomes free), so the ends are sorted here: losing the
	# limit is the one outcome no model wants.
	var linear_low := Vector3(minf(linear_min.x, linear_max.x), minf(linear_min.y, linear_max.y), minf(linear_min.z, linear_max.z))
	var linear_high := Vector3(maxf(linear_min.x, linear_max.x), maxf(linear_min.y, linear_max.y), maxf(linear_min.z, linear_max.z))
	var angular_low := Vector3(minf(angular_min.x, angular_max.x), minf(angular_min.y, angular_max.y), minf(angular_min.z, angular_max.z))
	var angular_high := Vector3(maxf(angular_min.x, angular_max.x), maxf(angular_min.y, angular_max.y), maxf(angular_min.z, angular_max.z))

	# A narrow angular limit without a spring is a joint the author wants to stay
	# where it is. A limit alone is not enough: a limit can only push back once the
	# joint is already out, and with a chain hanging on it the solver can leave it
	# out by tens of degrees. A spring on the same axis pulls the joint to the pose
	# of the file instead, which is what keeps a wing of small limits together.
	var support := support_spring_scale > 0.0 and angular_spring.is_zero_approx() and (angular_high - angular_low).length() < LIMIT_SPRING_RANGE
	var area := physics_scale * physics_scale * support_spring_scale

	for axis in range(3):
		PhysicsServer3D.generic_6dof_joint_set_flag(joint, axis, PhysicsServer3D.G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT, true)
		PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, PhysicsServer3D.G6DOF_JOINT_LINEAR_LOWER_LIMIT, linear_low[axis])
		PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, PhysicsServer3D.G6DOF_JOINT_LINEAR_UPPER_LIMIT, linear_high[axis])
		PhysicsServer3D.generic_6dof_joint_set_flag(joint, axis, PhysicsServer3D.G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT, true)
		PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, PhysicsServer3D.G6DOF_JOINT_ANGULAR_LOWER_LIMIT, angular_low[axis])
		PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, PhysicsServer3D.G6DOF_JOINT_ANGULAR_UPPER_LIMIT, angular_high[axis])

		_set_spring(joint, axis, linear_spring[axis] * linear_spring_scale, true)
		if support:
			_set_spring(joint, axis, LIMIT_SPRING_STIFFNESS * area, false, LIMIT_SPRING_DAMPING * area)
		else:
			_set_spring(joint, axis, angular_spring[axis] * angular_spring_scale * physics_scale * physics_scale, false)


## Enables the spring of one axis when it has a stiffness. A zero stiffness is
## how the PMX file says "no spring on this axis". [param damping] overrides
## [member spring_damping] for the springs the importer adds itself.
func _set_spring(joint: RID, axis: int, stiffness: float, linear: bool, damping := -1.0) -> void:
	var flag := PhysicsServer3D.G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING if linear else PhysicsServer3D.G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING
	var stiffness_param := PhysicsServer3D.G6DOF_JOINT_LINEAR_SPRING_STIFFNESS if linear else PhysicsServer3D.G6DOF_JOINT_ANGULAR_SPRING_STIFFNESS
	var damping_param := PhysicsServer3D.G6DOF_JOINT_LINEAR_SPRING_DAMPING if linear else PhysicsServer3D.G6DOF_JOINT_ANGULAR_SPRING_DAMPING
	var equilibrium_param := PhysicsServer3D.G6DOF_JOINT_LINEAR_SPRING_EQUILIBRIUM_POINT if linear else PhysicsServer3D.G6DOF_JOINT_ANGULAR_SPRING_EQUILIBRIUM_POINT
	if is_zero_approx(stiffness):
		PhysicsServer3D.generic_6dof_joint_set_flag(joint, axis, flag, false)
		return
	PhysicsServer3D.generic_6dof_joint_set_flag(joint, axis, flag, true)
	PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, stiffness_param, absf(stiffness))
	PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, damping_param, spring_damping if damping < 0.0 else damping)
	PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, equilibrium_param, 0.0)


## Returns the collision shape of the PMX shape and size, shared by the bodies
## that use the same combination (a model has many identical hair spheres).
func _shape_for(shape: int, size: Vector3) -> RID:
	var key := "%d:%s" % [shape, size]
	if _shape_cache.has(key):
		return _shape_cache[key]

	var radius := maxf(size.x, MIN_SIZE)
	var rid: RID
	match shape:
		Shape.BOX:
			# PMX and Godot both store the half extents of a box.
			rid = PhysicsServer3D.box_shape_create()
			PhysicsServer3D.shape_set_data(rid, Vector3(maxf(size.x, MIN_SIZE), maxf(size.y, MIN_SIZE), maxf(size.z, MIN_SIZE)))
		Shape.CAPSULE:
			# Bullet (and PMX) measure the height of a capsule between the
			# centers of its two hemispheres, Godot measures the whole shape.
			rid = PhysicsServer3D.capsule_shape_create()
			PhysicsServer3D.shape_set_data(rid, {"radius": radius, "height": maxf(size.y, 0.0) + radius * 2.0})
		_:
			rid = PhysicsServer3D.sphere_shape_create()
			PhysicsServer3D.shape_set_data(rid, radius)
	_shape_cache[key] = rid
	return rid
