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
##
## [b]Tuning[/b]: the masses, the damping, the restitution, the swept test, the
## sleeping and the limits of the joints are all read from [member strategy],
## see [MMDPhysicsStrategy]. The values of the file alone are rarely usable: the
## test models of this project carry masses up to three hundred thousand kilos
## per cubic unit, joints free to slide that should not be, and, in most files,
## no spring at all.

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
## Density, in kg per cubic unit of the scene, a body is allowed to have. MMD
## files are authored with the units of the file and no density in mind, so a
## mass that is far above what the volume can hold is a typo of the author that
## only shows up as a body the solver throws.
const MAX_DENSITY := 10000.0
## Damping every body gets, even when the file asks for none. A contact that the
## solver keeps pushing (a lock of hair caught in the body, a shape that overlaps
## its neighbour) adds up until the chain leaves the model, and the damping is
## what turns that into a wobble that stays.
var BASE_DAMP := 0.5
## PMX files use 16 collision groups.
const GROUP_COUNT := 16

## Tuning of the simulation, see [MMDPhysicsStrategy]. It is read while the
## bodies and the joints are created, so changing it needs the world to be built
## again ([method MMDPhysicsSimulator.rebuild]).
var strategy := MMDPhysicsStrategy.new()
## Gravity of the space, in the units of the imported model.
var gravity := 9.8
## Length scale of the model relative to the units of the PMX file (the importer
## uses 0.1). MMD runs Bullet in the units of the file, so a value that carries a
## length has to follow the scale of the model: the stiffness of an angular
## spring is a torque per radian, a torque scales with a length squared, so the
## angular springs are converted with the square of this value. Linear springs
## and damping have no length dimension and are used as they are.
var physics_scale := 1.0
## Collision mask of every group, worked out by [method build_group_rules]. When
## it is set, it replaces the mask each body carries.
var group_rules := PackedInt32Array()
## Sum of the angular limits, in radians, below which a joint without a spring
## counts as "hold this part in place" and gets the support spring.
const LIMIT_SPRING_RANGE := 0.6
## Angular frequency (rad/s) the support spring wants the joint to return with,
## and the damping ratio that goes with it. The stiffness is derived from the
## inertia of the body the joint drives, so a 7 cm lock of hair and a one meter
## feather are held with the same response instead of one being loose and the
## other shaking. Over damped on purpose: a limit that is fought by a light spring
## is a limit that wobbles.
const SUPPORT_OMEGA := 100.0
const SUPPORT_ZETA := 5.0

var _space := RID()
var _bodies: Array[RID] = []
var _joints: Array[RID] = []
var _dynamic: Array[bool] = []
## Rough moment of inertia of every body (mass times the longest extent, squared),
## used to give the support spring the same response on every body.
var _inertia: Array[float] = []
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
	_inertia.clear()
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


## Works out the collision mask of every group, so the pairs that collide are the
## pairs MMD collides.
##
## A body of the file collides with another one when <b>both</b> masks list the
## group of the other, which is what Bullet, the engine of MMD, checks. Godot
## collides when <b>either</b> mask lists the group of the other, so a file whose
## masks are not symmetrical (very common: a lock of hair lists the body group,
## the body does not list the hair one) collides far more than MMD does. That
## extra contact is what pumps energy into a chain until it flies away, so the
## masks are replaced by one per group pair. A pair is kept only when every body
## on both sides lists the group of the other: with masks that are not uniform
## inside a group, the stricter reading is the one that cannot add a contact MMD
## does not have.
static func build_group_rules(bodies: Array) -> PackedInt32Array:
	var grouped: Array = []
	grouped.resize(GROUP_COUNT)
	for g in range(GROUP_COUNT):
		grouped[g] = []
	for i in range(bodies.size()):
		var g := clampi(int(bodies[i].get("group", 0)), 0, GROUP_COUNT - 1)
		grouped[g].append(i)
	var rules := PackedInt32Array()
	rules.resize(GROUP_COUNT)
	for group_a in range(GROUP_COUNT):
		var row := 0
		for group_b in range(GROUP_COUNT):
			if grouped[group_a].is_empty() or grouped[group_b].is_empty():
				continue
			if not _group_sees(bodies, grouped[group_a], group_b):
				continue
			if not _group_sees(bodies, grouped[group_b], group_a):
				continue
			row |= 1 << group_b
		rules[group_a] = row
	return rules


## Whether every body of [param list] lists [param group] in its mask.
static func _group_sees(bodies: Array, list: Array, group: int) -> bool:
	for i in list:
		if (int(bodies[i].get("mask", 0)) >> group) & 1 == 0:
			return false
	return true


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
	if group_rules.size() == GROUP_COUNT:
		mask = group_rules[group]

	var shape := int(data.get("shape", Shape.SPHERE))
	var size: Vector3 = data.get("size", Vector3.ONE)
	var mass := _mass_of(data, shape, size)

	var body := PhysicsServer3D.body_create()
	PhysicsServer3D.body_set_mode(body, _body_mode(dynamic, int(data.get("bone", -1))))
	PhysicsServer3D.body_set_param(body, PhysicsServer3D.BODY_PARAM_MASS, mass)
	PhysicsServer3D.body_set_param(body, PhysicsServer3D.BODY_PARAM_FRICTION, maxf(float(data.get("friction", 0.5)), 0.0))
	PhysicsServer3D.body_set_param(body, PhysicsServer3D.BODY_PARAM_BOUNCE, _bounce_of(data))
	var damping := _damping_of(data)
	PhysicsServer3D.body_set_param(body, PhysicsServer3D.BODY_PARAM_LINEAR_DAMP, damping.x)
	PhysicsServer3D.body_set_param(body, PhysicsServer3D.BODY_PARAM_ANGULAR_DAMP, damping.y)
	# MMD disables deactivation (Bullet: DisableDeactivation, zero sleep
	# thresholds): a hair or skirt chain has to react the moment the body moves,
	# not after it woke up again. It is only worth letting a body sleep when the
	# strategy asks for the cheapest simulation, and then the chain it belongs to
	# wakes up as a whole, because the bodies are held by joints.
	PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_CAN_SLEEP, strategy.allow_sleep)

	PhysicsServer3D.body_set_collision_layer(body, 1 << group)
	PhysicsServer3D.body_set_collision_mask(body, mask)
	# A hair body is a few centimeter big and moves fast: without a swept test it
	# passes through the body it should not enter, and the contact that finally
	# finds it pushes it away with the speed it had.
	if dynamic:
		PhysicsServer3D.body_set_enable_continuous_collision_detection(body, strategy.continuous_collision)
	PhysicsServer3D.body_add_shape(body, _shape_for(shape, size))
	var transform: Transform3D = data.get("transform", Transform3D.IDENTITY)
	PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_TRANSFORM, transform)
	PhysicsServer3D.body_set_space(body, _space)

	_bodies.append(body)
	_dynamic.append(dynamic)
	_transforms.append(transform)
	var extent := maxf(size.x, maxf(size.y, size.z))
	_inertia.append(mass * extent * extent)
	return index


## Returns the mode the physics server needs for a body.
##
## A body the file marks as static is a collider the bones drag around, which is
## a kinematic body. A static body that belongs to no bone never moves at all, so
## it is created as a plain static body: the server can skip it in the broad
## phase, and nothing else in the model can push it away.
func _body_mode(dynamic: bool, bone: int) -> int:
	if dynamic:
		return PhysicsServer3D.BODY_MODE_RIGID
	return PhysicsServer3D.BODY_MODE_STATIC if bone < 0 else PhysicsServer3D.BODY_MODE_KINEMATIC


## Returns the mass of a body, following [member MMDPhysicsStrategy.mass_mode].
##
## The masses of a PMX file are usually meaningless: the test models of this
## project go up to fifteen kilos for a lock of hair of three centimeters, a
## density of three hundred thousand kilos per cubic unit. A body like that
## answers the lightest contact with an impulse that no chain of joints
## survives, and it drags the bodies it is joint to along with it.
func _mass_of(data: Dictionary, shape: int, size: Vector3) -> float:
	var volume := volume_of(shape, size)
	match strategy.mass_mode:
		MMDPhysicsStrategy.MassMode.UNIFORM:
			return maxf(strategy.uniform_mass, MIN_MASS)
		MMDPhysicsStrategy.MassMode.DENSITY:
			if volume > 0.0:
				return maxf(volume * strategy.density, MIN_MASS)
	# The mass of the file, cut down to what the volume can reasonably hold: a
	# typo of the author is the one thing that must not reach the solver.
	var mass := maxf(float(data.get("mass", 1.0)), MIN_MASS)
	if volume > 0.0:
		mass = minf(mass, maxf(volume * MAX_DENSITY, MIN_MASS))
	return mass


## Returns the restitution of a body, in the units of the physics server.
func _bounce_of(data: Dictionary) -> float:
	if strategy.zero_restitution:
		return 0.0
	return clampf(float(data.get("restitution", 0.0)), 0.0, 1.0)


## Returns the linear and the angular damping of a body, in (x, y).
##
## The file gives a damping of 0.5 to 1.0 to almost every body, so what the
## strategy adds on top is what decides how long a chain keeps swinging. The
## value of the file is never lowered below [constant BASE_DAMP]: a solver that
## cannot settle a contact needs the floor.
func _damping_of(data: Dictionary) -> Vector2:
	var scale := maxf(strategy.damping_scale, 0.0)
	return Vector2(
		maxf(float(data.get("linear_damp", 0.0)), BASE_DAMP) * scale,
		maxf(float(data.get("angular_damp", 0.0)), BASE_DAMP) * scale)


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
	# The support spring is sized on the body the joint drives, and the static
	# side of the joint is the one that carries no inertia.
	var driven := body_b if _dynamic[body_b] else body_a
	_configure_joint(joint, data, _inertia[driven])
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


## Cuts the speeds of the body down to the given limits. A contact between two
## bodies that overlap (the shape of the file is generous, and a chain of hair
## hangs inside the body) can hand out a speed that no chain survives; a body of a
## real model never comes near these limits, so this only ever touches a body
## that is already running away.
func clamp_body_velocity(index: int, max_linear: float, max_angular: float) -> void:
	if index < 0 or index >= _bodies.size() or not _dynamic[index]:
		return
	var body: RID = _bodies[index]
	var linear: Vector3 = PhysicsServer3D.body_get_state(body, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY)
	if linear.length_squared() > max_linear * max_linear:
		PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY, linear.normalized() * max_linear)
	var angular: Vector3 = PhysicsServer3D.body_get_state(body, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY)
	if angular.length_squared() > max_angular * max_angular:
		PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY, angular.normalized() * max_angular)


## Cuts the speed of every simulated body down to [param max_linear] and
## [param max_spin].
##
## A single contact that the solver cannot resolve (a body of the file that
## overlaps another one, a shape far bigger than the part it stands for) hands
## out a speed that the joints then spread over the whole model in a few steps. A
## limit on the speed of the bodies stops that cascade: the part keeps moving,
## but it cannot leave with a speed the animation never had.
func clamp_speeds(max_linear: float, max_spin: float) -> void:
	if not _open:
		return
	for index in range(_bodies.size()):
		if not _dynamic[index]:
			continue
		clamp_body_velocity(index, max_linear, max_spin)


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
func _configure_joint(joint: RID, data: Dictionary, inertia: float) -> void:
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

	# A joint of a hair or a skirt is meant to keep its parts at a fixed distance:
	# a translation that slides is what lets the chain stretch, step by step,
	# until the hair hangs below the mesh it belongs to (in the files of the test
	# folder a third of the joints allow the slide). The strategy pins it.
	if strategy.lock_translation:
		linear_low = Vector3.ZERO
		linear_high = Vector3.ZERO

	# A narrow angular limit without a spring is a joint the author wants to stay
	# where it is. A limit alone is not enough: a limit can only push back once the
	# joint is already out, and with a chain hanging on it the solver can leave it
	# out by tens of degrees. A spring on the same axis pulls the joint to the pose
	# of the file instead, which is what keeps a wing of small limits together.
	var support := strategy.support_spring_scale > 0.0 and angular_spring.is_zero_approx() and (angular_high - angular_low).length() < LIMIT_SPRING_RANGE
	# A spring that returns a body of inertia I with the wanted frequency and
	# damping: k = I * w2 and c = 2 * z * I * w.
	var inertia_scaled := inertia * strategy.support_spring_scale
	var support_stiffness := inertia_scaled * SUPPORT_OMEGA * SUPPORT_OMEGA
	var support_damping := 2.0 * SUPPORT_ZETA * inertia_scaled * SUPPORT_OMEGA

	for axis in range(3):
		PhysicsServer3D.generic_6dof_joint_set_flag(joint, axis, PhysicsServer3D.G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT, true)
		PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, PhysicsServer3D.G6DOF_JOINT_LINEAR_LOWER_LIMIT, linear_low[axis])
		PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, PhysicsServer3D.G6DOF_JOINT_LINEAR_UPPER_LIMIT, linear_high[axis])
		PhysicsServer3D.generic_6dof_joint_set_flag(joint, axis, PhysicsServer3D.G6DOF_JOINT_FLAG_ENABLE_ANGULAR_LIMIT, true)
		PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, PhysicsServer3D.G6DOF_JOINT_ANGULAR_LOWER_LIMIT, angular_low[axis])
		PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, PhysicsServer3D.G6DOF_JOINT_ANGULAR_UPPER_LIMIT, angular_high[axis])

		if strategy.disable_linear_spring:
			_set_spring(joint, axis, 0.0, true)
		else:
			_set_spring(joint, axis, linear_spring[axis] * strategy.linear_spring_scale, true)
		if support:
			_set_spring(joint, axis, support_stiffness, false, support_damping)
		else:
			_set_spring(joint, axis, angular_spring[axis] * strategy.angular_spring_scale * physics_scale * physics_scale, false)


## Enables the spring of one axis when it has a stiffness. A zero stiffness is
## how the PMX file says "no spring on this axis". [param damping] overrides
## [member MMDPhysicsStrategy.spring_damping] for the springs the importer adds
## itself.
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
	PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, damping_param, strategy.spring_damping if damping < 0.0 else damping)
	PhysicsServer3D.generic_6dof_joint_set_param(joint, axis, equilibrium_param, 0.0)


## Volume of the collision shape, in units of the scene.
static func volume_of(shape: int, size: Vector3) -> float:
	match shape:
		Shape.BOX:
			return 8.0 * size.x * size.y * size.z
		Shape.CAPSULE:
			var radius := size.x
			return PI * radius * radius * size.y + (4.0 / 3.0) * PI * radius * radius * radius
		_:
			return (4.0 / 3.0) * PI * size.x * size.x * size.x


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
