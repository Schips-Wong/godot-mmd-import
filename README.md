# MMD-For-Godot
About Godot MMD Plugin with gdscript


- [x] Parse PMX File
- [ ] Parse PMX File Exception handling
- [x] Generate Godot Skeleton3D Tree
- [x] Generate Godot Skin
- [x] Generate Godot Mesh
- [x] Generate Godot Morph (vertex morphs as blend shapes)
- [x] Generate Godot Rigidbody (MMDPhysicsSimulator + mmd_physics_data)
- [x] Generate Godot Joint (Generic6DOF, limits and springs of the PMX file)
- [x] Physics optimization strategies (see below)
- [ ] Generate Godot Softbody
- [x] Parse VMD File
- [x] Generate Godot Animation (bone)
- [x] Solve IK and 付与 at runtime (MMDPoseSolver, motions stay blendable)
- [x] Optional bake of the rules (import option "mmd/bake", self contained motions)
- [ ] Generate Godot Animation (morph)
- [ ] Generate Godot Animation (camera / light)
- [ ] More Powerful


## Physics

Every imported model gets a `MMDPhysicsSimulator` node on its `Skeleton3D`. It
simulates the rigid bodies (剛体) and joints (ジョイント) of the file in a private
physics space and writes the result back into the bones, which is what moves the
hair, the skirt, the ribbon and the accessories.

### Strategies

The file alone is rarely usable. `MMDPhysicsStrategy` holds one switch per
defect that a PMX file (or the change of engine) brings, and the presets are the
combinations that work on the models of `test/`:

| Preset | What it does | When |
| --- | --- | --- |
| `Accurate` | The values of the file, nothing else | Comparing with MMD, debugging a model |
| `Stable` | Locks the translation of the joints, drops the linear springs and the restitution, gives every body the same mass, more damping, and caps the speed | **Recommended starting point** |
| `Soft` | Same repairs, weaker and more damped springs | Hair and skirt that should swing further |
| `Performance` | Translation lock and no restitution only, no CCD, sleeping allowed, no per frame checks | Models with hundreds of bodies |
| `Custom` | The parameters the node exports, as they are | A model whose physics was hand tuned |

Pick one in the import dock (`mmd/physics_strategy`) or on the node
(`preset`). A preset is layered **on top** of the parameters the node exports,
so a scene whose hair was tuned by hand (spring scales) keeps its tuning when
the preset changes.

The switches themselves are available on the node under *优化策略*:

- `lock_joint_translation` — pin the translation of every joint to zero. A third
  of the joints of the files in `test/` let the bodies slide, which is what makes
  a skirt grow longer frame after frame.
- `disable_linear_spring` — drop the linear springs, which do nothing once the
  translation is locked.
- `zero_restitution` — no bounce. A body that bounces hands the speed of a
  contact back to the chain.
- `mass_mode` — `File` (the mass of the file, cut down to what its volume can
  hold), `Uniform` (one mass for every body) or `Density` (mass from the volume).
  The files in `test/` go up to three hundred thousand kilos per cubic unit, and
  a chain is only stable when its parts agree with each other.
- `damping_scale` — damping of the bodies, the one of the file included.
- `allow_sleep`, `continuous_collision`, `stability_guard`, `max_speed` — the
  cost/stability trade off of the simulation.

Changing a parameter while the scene runs needs `rebuild()` on the node.

### Tuning a model

1. Start with `Stable`. If the hair is too stiff or too loose, tune
   `angular_spring_scale` / `support_spring_scale` in *关节弹簧*.
2. The hair that lags behind the body: raise the `influence` of the node (it is
   the blend Skeleton3D applies between the previous pose and the physics
   result; 1.0 is exact, 0.25 is smooth but late).
3. A part that flies away: `max_speed` caps the damage, `mass_mode = Density` or
   `Uniform` removes its cause.

### Known limits

- Jolt (the physics engine of Godot 4.6) has 10 velocity steps and 2 position
  steps, and does not expose them per space. A model with a thousand joints and
  bodies far heavier than the part they stand for will still stretch its joints
  under a fast motion. `ProjectSettings > physics/jolt_physics_3d/simulation`
  (`velocity_steps`, `position_steps`, `penetration_slop`) is the global knob
  that helps the most; `penetration_slop` defaults to 2 cm, which is a lot for a
  body of three centimeters.
- MMD runs Bullet in the units of the PMX file. The importer works at 0.1, so
  the values that carry a length (gravity, angular stiffness) are converted, but
  the tolerances of the engine itself are not.
