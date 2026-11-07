class_name MMDPhysicsStrategy extends RefCounted

## The switches that repair the physics of one MMD model, and the presets that
## combine them.
##
## A PMX file carries physics authored for Bullet, at the scale and with the
## tolerances of MMD: chains of thin boxes and capsules (hair, skirt, ribbon)
## held by 6 DOF joints. What the files contain is often wrong on top of that:
## a lock of hair of three centimeters weighing fifteen kilos (a density of
## 300000 kg per cubic unit), a joint whose translation is free while the chain
## is meant to keep its length, and, in most files, no spring at all, so a
## limit that is a few degrees wide is the only thing that holds the part.
##
## A solver with a larger step, a different inertia and a different notion of a
## spring does not reproduce that: the chain stretches at the joints that are
## free to translate, the heavy body at the end of a light chain pulls it apart
## and a contact that should be gentle throws the whole chain away. That is the
## classic MMD bug: hair that shakes, stretches or explodes.
##
## This class holds one switch per defect. The presets are the combinations
## that work on the models of the test folder:
## [codeblock]
## ACCURATE     the values of the file, nothing else
## STABLE       translation locked, one mass per body, more damping
## SOFT         weaker springs: hair and skirt swing further and slower
## PERFORMANCE  no CCD, sleeping allowed, fewer checks per frame
## CUSTOM       the switches of [MMDPhysicsSimulator] as they are
## [/codeblock]
##
## A preset is applied [b]on top[/b] of the values the node exports, so the
## tuning of a scene (the spring scales of a model whose hair was hand tuned,
## for example) survives a change of preset: only the switches a preset has an
## opinion about are replaced.
##
## The object is read by [MMDPhysicsWorld] while it creates the bodies, and by
## [MMDPhysicsSimulator] for the checks it runs every frame.

## Named combinations of the switches, see the class description.
enum Preset {
	ACCURATE,
	STABLE,
	SOFT,
	PERFORMANCE,
	CUSTOM,
}

## How the mass of the bodies is worked out. PMX files are usually authored
## with meaningless masses, and a chain is only stable when its parts agree
## with each other.
enum MassMode {
	## The mass of the file, cut down to what the volume can hold (see
	## [constant MMDPhysicsWorld.MAX_DENSITY]).
	FILE,
	## Every simulated body weighs [member uniform_mass]. The fix the Blender
	## plugin MMD-Physics uses, and the one that makes almost every file behave.
	UNIFORM,
	## The mass follows the volume, with [member density] (kg per cubic unit of
	## the scene, 1000 being water). Keeps the weight ratio of a big and a small
	## body of the same chain physical instead of equal.
	DENSITY,
}

## Damping of the STABLE preset, as a multiplier of the damping of the node.
const STABLE_DAMPING := 1.5
## Support spring of the STABLE preset, as a multiplier of the one of the node.
const STABLE_SUPPORT := 1.5
## Stiffness of the springs of the SOFT preset, linear and angular.
const SOFT_SPRING := 0.5
## Damping ratio of the springs of the SOFT preset, as a multiplier of the one
## of the node. A weaker spring that is not damped more is a spring that swings
## on its own.
const SOFT_DAMPING := 1.25
## Damping of the bodies of the SOFT preset.
const SOFT_BODY_DAMPING := 1.2
## Speed, in units of the scene per second, the STABLE and SOFT presets hold the
## bodies to. A model imported at 0.1 is about two units tall, and a lock of hair
## of a real motion stays below a unit per second; a body of the test folder that
## a contact throws around reaches fifty.
const GUARDED_SPEED := 5.0

## Preset this strategy was built from.
var preset := Preset.CUSTOM

# ---------------------------------------------------------------- joints

## Sets the translation of every joint to zero, whatever the file asks for.
##
## A PMX joint can allow its bodies to slide (a skirt of the file very often
## does, by a few millimeters to a few centimeters). Under Bullet the joint
## returns exactly to where it started; under a solver that does not, every
## contact leaves a little displacement behind and the chain stretches until it
## no longer matches the mesh. Locking the translation is the single most
## effective repair of a skirt that grows.
var lock_translation := false
## Drops the linear springs of the joints. With the translation locked they
## have nothing left to do, and a stiff linear spring on a body of a few grams
## is a spring that fights the lock.
var disable_linear_spring := false
## Multiplier of the linear spring stiffness of the file.
var linear_spring_scale := 1.0
## Multiplier of the angular spring stiffness of the file.
var angular_spring_scale := 1.0
## Damping ratio given to every joint spring.
var spring_damping := 0.5
## Multiplier of the support spring [MMDPhysicsWorld] adds to the joints whose
## limit is narrow and that the file leaves without one.
var support_spring_scale := 1.0

# ---------------------------------------------------------------- bodies

## How the mass of the simulated bodies is worked out.
var mass_mode := MassMode.FILE
## Mass of every simulated body under [constant MassMode.UNIFORM].
var uniform_mass := 1.0
## Density under [constant MassMode.DENSITY], in kg per cubic unit of the
## scene. The importer works at 0.1, so a unit of the scene is ten centimeters
## and the units are close enough to meters for 1000 to mean water.
var density := 1000.0
## Multiplier of the linear and angular damping of the bodies, the one of the
## file included. Damping is what turns an oscillation the solver cannot settle
## into a motion that stays.
var damping_scale := 1.0
## Forces the restitution of every body to zero. A body that bounces is a body
## that hands the speed of a contact back to the chain, and a chain of hair or
## of a skirt never wants that.
var zero_restitution := false
## Swept collision of the simulated bodies. A hair body is a few centimeters
## big and moves fast; without it the hair passes through the body it should
## not enter, and the contact that finally finds it throws it away.
var continuous_collision := true
## Lets the solver put a body to sleep when it stops. MMD disables it (a chain
## has to answer the moment its bone moves), so it is only worth turning on for
## models with hundreds of bodies that nobody looks at closely.
var allow_sleep := false

# ---------------------------------------------------------------- runtime

## Keeps the per frame checks of [MMDPhysicsSimulator] (a body that drifts away
## from its bone is put back, one that runs away is braked).
## [constant Preset.PERFORMANCE] turns it off: it is the price of the cheapest
## simulation, a body that leaves its chain then never comes back.
var stability_guard := true
## Speed, in units of the scene per second, every simulated body is held to, 0
## turns it off.
##
## One unresolvable contact (a shape of the file that overlaps a hundred others,
## a body far heavier than the part it stands for) hands out a speed that the
## joints spread over the whole chain in a few steps, and the chain is gone. A
## cap costs one read and, only for the bodies that are over it, one write per
## body and per frame, and it keeps a part from leaving at a speed no animation
## ever had. It does not repair a joint that is too soft, it only stops the
## damage from spreading.
var max_speed := 0.0


## Returns the strategy of [param preset_value], applied on top of [param base].
## [param base] is the tuning the node exports; it is copied, so the caller
## keeps its own object untouched. A preset only replaces the switches it has
## an opinion about, everything else keeps the value of [param base].
static func create(preset_value: int, base: MMDPhysicsStrategy = null) -> MMDPhysicsStrategy:
	var strategy := base.copy() if base != null else MMDPhysicsStrategy.new()
	strategy.preset = preset_value
	match preset_value:
		Preset.STABLE:
			# The combination of the Blender plugin MMD-Physics, plus the
			# translation lock: with it, a skirt neither stretches nor pumps
			# energy into the chain, and a uniform mass keeps the heaviest body
			# of a chain from dragging the lightest ones away.
			strategy.lock_translation = true
			strategy.disable_linear_spring = true
			strategy.zero_restitution = true
			strategy.mass_mode = MassMode.UNIFORM
			strategy.damping_scale *= STABLE_DAMPING
			strategy.support_spring_scale *= STABLE_SUPPORT
			strategy.allow_sleep = false
			strategy.continuous_collision = true
			strategy.stability_guard = true
			strategy.max_speed = maxf(strategy.max_speed, GUARDED_SPEED)
		Preset.SOFT:
			# Same repairs, but the springs are weaker and more damped, so the
			# hair and the skirt follow the motion instead of snapping back.
			strategy.lock_translation = true
			strategy.disable_linear_spring = true
			strategy.zero_restitution = true
			strategy.linear_spring_scale *= SOFT_SPRING
			strategy.angular_spring_scale *= SOFT_SPRING
			strategy.support_spring_scale *= SOFT_SPRING
			strategy.spring_damping = minf(strategy.spring_damping * SOFT_DAMPING, 1.0)
			strategy.damping_scale *= SOFT_BODY_DAMPING
			strategy.allow_sleep = false
			strategy.continuous_collision = true
			strategy.stability_guard = true
			strategy.max_speed = maxf(strategy.max_speed, GUARDED_SPEED)
		Preset.PERFORMANCE:
			# Every repair that costs nothing, and none that costs a step: no
			# swept test, sleeping allowed and no per frame check. The
			# translation lock stays because it is the one that prevents the
			# chain from being torn apart in the first place.
			strategy.lock_translation = true
			strategy.disable_linear_spring = true
			strategy.zero_restitution = true
			strategy.allow_sleep = true
			strategy.continuous_collision = false
			strategy.stability_guard = false
			strategy.max_speed = 0.0
		_:
			# ACCURATE and CUSTOM: what the node exports, unchanged.
			pass
	return strategy


## Returns a copy of the strategy, so a preset never writes into the object of
## another node.
func copy() -> MMDPhysicsStrategy:
	var strategy := MMDPhysicsStrategy.new()
	strategy.preset = preset
	strategy.lock_translation = lock_translation
	strategy.disable_linear_spring = disable_linear_spring
	strategy.linear_spring_scale = linear_spring_scale
	strategy.angular_spring_scale = angular_spring_scale
	strategy.spring_damping = spring_damping
	strategy.support_spring_scale = support_spring_scale
	strategy.mass_mode = mass_mode
	strategy.uniform_mass = uniform_mass
	strategy.density = density
	strategy.damping_scale = damping_scale
	strategy.zero_restitution = zero_restitution
	strategy.continuous_collision = continuous_collision
	strategy.allow_sleep = allow_sleep
	strategy.stability_guard = stability_guard
	strategy.max_speed = max_speed
	return strategy


## Returns the name of [param preset], for the inspector and the log.
static func preset_name(preset_value: int) -> String:
	match preset_value:
		Preset.ACCURATE:
			return "Accurate"
		Preset.STABLE:
			return "Stable"
		Preset.SOFT:
			return "Soft"
		Preset.PERFORMANCE:
			return "Performance"
		_:
			return "Custom"


## Returns a one line summary of the switches that differ from the reference
## behaviour, for the log of [MMDPhysicsSimulator].
func describe() -> String:
	var parts: Array[String] = []
	if lock_translation:
		parts.append("translation locked")
	if disable_linear_spring:
		parts.append("no linear spring")
	if zero_restitution:
		parts.append("no restitution")
	match mass_mode:
		MassMode.UNIFORM:
			parts.append("uniform mass %s" % uniform_mass)
		MassMode.DENSITY:
			parts.append("density %s" % density)
	if not continuous_collision:
		parts.append("no CCD")
	if allow_sleep:
		parts.append("sleeping allowed")
	if not stability_guard:
		parts.append("no guards")
	if max_speed > 0.0:
		parts.append("speed capped at %s" % max_speed)
	if not is_equal_approx(linear_spring_scale, 1.0) or not is_equal_approx(angular_spring_scale, 1.0):
		parts.append("spring x%s/%s" % [linear_spring_scale, angular_spring_scale])
	if not is_equal_approx(spring_damping, 0.5):
		parts.append("spring damping %s" % spring_damping)
	if not is_equal_approx(support_spring_scale, 1.0):
		parts.append("support x%s" % support_spring_scale)
	if not is_equal_approx(damping_scale, 1.0):
		parts.append("damping x%s" % damping_scale)
	return "%s: %s" % [preset_name(preset), ", ".join(parts) if not parts.is_empty() else "file values"]
