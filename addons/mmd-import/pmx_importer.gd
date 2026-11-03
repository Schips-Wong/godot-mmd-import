class_name PMXImporter extends EditorSceneFormatImporter

## Imports a PMX model as a scene with a [Skeleton3D], one skinned mesh per
## material and, when the file has vertex morphs (表情), a "Morph" mesh whose
## [ArrayMesh] holds one blend shape per morph.
##
## The morph mesh only contains the triangles the morphs touch, and the blend
## shapes store absolute vertex positions (like the glTF morph targets), so
## driving them costs the face vertices only:
## [codeblock]
## var face: MeshInstance3D = $Model/Mesh/Morph
## var smile := face.mesh.find_blend_shape_by_name("笑い")
## face.set_blend_shape_value(smile, 1.0)
## [/codeblock]
##
## The other morph types (UV, bone, material, group) are not converted.
##
## The rigid bodies (剛体) and joints (ジョイント) become a [MMDPhysicsSimulator]
## node plus the "mmd_physics_data" metadata: they are what moves the hair, the
## skirt and the accessories at runtime.
##
## Skeleton3D metadata consumed by MMDPoseSolver and MMDPoseBaker.
const BONE_DATA_META := "mmd_bone_data"
const SOLVER_NODE_NAME := "MMDPoseSolver"

## Skeleton3D metadata consumed by [MMDPhysicsSimulator]: the rigid bodies
## (剛体) and joints (ジョイント) of the file, scaled and mirrored like the mesh.
const PHYSICS_DATA_META := "mmd_physics_data"
const PHYSICS_NODE_NAME := "MMDPhysics"

## The influence value of the physics bones.
const PHYSICS_INFLUENCE_VALUE := 0.25

## Node holding the surfaces deformed by the vertex morphs (表情). Its
## [ArrayMesh] carries one blend shape per Position morph of the PMX file.
const MORPH_NODE_NAME := "Morph"

## Adds the runtime [MMDPoseSolver] node that applies 付与 (append) to motions
## that do not contain it yet. Imported VMD motions are baked by [MMDPoseBaker]
## and skipped by the solver, so the node is only needed for motions authored
## elsewhere; disable it for models that only play baked motions. The bone data
## metadata is written either way.
const OPTION_RUNTIME_SOLVER := "mmd/runtime_pose_solver"

## Adds the runtime [MMDPhysicsSimulator] node that simulates the rigid bodies
## and joints of the file (hair, skirt, breasts...). Without it the metadata is
## still written, so the node can be added by hand later.
const OPTION_PHYSICS := "mmd/physics"


func _get_extensions() -> PackedStringArray:
	return PackedStringArray(["pmx"])

func _get_import_options(_path: String) -> void:
	add_import_option(OPTION_RUNTIME_SOLVER, true)
	add_import_option(OPTION_PHYSICS, true)


func _import_scene(path: String, _flags: int, options: Dictionary) -> Object:
	var pmx = PMXFile.new()
	pmx.setup(path)
	pmx.scale(0.1)
	pmx_transform_to_godot(pmx)
	var runtime_solver := bool(options.get(OPTION_RUNTIME_SOLVER, true))
	var physics := bool(options.get(OPTION_PHYSICS, true))
	return create_mesh_node(pmx, 0.1, path.get_base_dir(), runtime_solver, physics)


func pmx_transform_to_godot(pmx):
	for v in pmx.m_vertices:
		v.m_position.z *= -1
		v.m_normal.z *= -1
	for b in pmx.m_bones:
		b.m_position.z *= -1
		b.m_positionOffset.z *= -1
	# Morph offsets live in the same space as the vertices, so the axis mirror
	# has to be applied to them as well. Mirroring is improper, which turns the
	# rotation axis of a bone morph into a pseudovector: (x, y, z) -> (-x, -y, z).
	for morph in pmx.m_morphs:
		for position in morph.m_positionMorph:
			position.m_position.z *= -1
		for bone in morph.m_boneMorph:
			bone.m_position.z *= -1
			bone.m_quaternion = Quaternion(-bone.m_quaternion.x, -bone.m_quaternion.y, bone.m_quaternion.z, bone.m_quaternion.w)
	pass


func create_mesh_node(pmx: PMXFile, scale: float, base_dir: String, runtime_solver: bool = true, physics: bool = true) -> Object:
	var scene = Node3D.new()
	var root_node = Node3D.new()
	var bone_node = Skeleton3D.new()
	var skin = Skin.new()
	scene.name = pmx.m_info.m_modelName
	scene.set_meta("description", pmx.m_info.m_comment)
	scene.set_meta("description_en", pmx.m_info.m_englishComment)
	scene.set_meta("model_scale", scale)
	bone_node.name = "Bone"
	root_node.name = "Mesh"
	
	scene.add_child(bone_node)
	bone_node.set_owner(scene)

	# The mesh group is nested under the skeleton on purpose: Godot's retarget
	# import options (retarget/bone_map + unique_node/make_unique) rename the
	# Skeleton3D node *after* this importer ran, without fixing the skeleton
	# paths of the meshes. A sibling path like "../../Bone" then points to a
	# node that no longer exists, the Skin is never applied and the mesh stays
	# frozen while the bones animate. "../../" resolves through the hierarchy,
	# so it survives any rename of the skeleton node.
	bone_node.add_child(root_node)
	root_node.set_owner(scene)

	# Create Bones
	for bone in pmx.m_bones:
		bone_node.add_bone(bone.m_name)

	for i in range(pmx.m_bones.size()):
		var bone = pmx.m_bones[i]
		var parent
		var position: Vector3
		if bone.m_parentBoneIndex >= 0:
			parent = pmx.m_bones[bone.m_parentBoneIndex]
			position = bone.m_position - parent.m_position
			bone_node.set_bone_parent(i, bone.m_parentBoneIndex)
			bone_node.set_bone_pose_position(i, position)
		else:
			position = bone.m_position
		bone_node.set_bone_rest(i, Transform3D(Basis.IDENTITY, position))

	_create_pose_solver(pmx, bone_node, scene, runtime_solver)
	_create_physics(pmx, bone_node, scene, physics)

	# Create Skin
	for bone in pmx.m_bones:
		skin.add_named_bind(bone.m_name, Transform3D(Basis.IDENTITY, -bone.m_position))
	
	var textureMap: Dictionary[int, StandardMaterial3D] = {}
	for i in range(pmx.m_textures.size()):
		var textureName: String = pmx.m_textures[i].m_textureName
		var texturePath: String = textureName.replace('\\', '/')
		var resourcePath: String = base_dir + "/" + texturePath
		var mt3d := StandardMaterial3D.new()
		mt3d.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS
		mt3d.cull_mode = BaseMaterial3D.CULL_BACK
		mt3d.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		mt3d.depth_draw_mode = BaseMaterial3D.DEPTH_DRAW_ALWAYS
		mt3d.resource_name = texturePath.get_file()
		mt3d.render_priority = i
		# "0Nub" is MMD's placeholder for "no texture"; the file usually does
		# not exist, but the index still has to resolve for the materials.
		if not textureName.ends_with("0Nub") and FileAccess.file_exists(resourcePath):
			mt3d.albedo_texture = load(resourcePath)
		textureMap[i] = mt3d

	# Create Mesh
	var vertices = PackedVector3Array()
	var normals = PackedVector3Array()
	var uvs = PackedVector2Array()
	var bones = PackedInt32Array()
	var weights = PackedFloat32Array()
	for vertex in pmx.m_vertices:
		vertices.append(vertex.m_position)
		normals.append(vertex.m_normal)
		uvs.append(vertex.m_uv)
		bones.append(vertex.m_boneId0)
		bones.append(vertex.m_boneId1)
		bones.append(vertex.m_boneId2)
		bones.append(vertex.m_boneId3)
		weights.append(vertex.m_boneWeight0)
		weights.append(vertex.m_boneWeight1)
		weights.append(vertex.m_boneWeight2)
		weights.append(vertex.m_boneWeight3)
	
	# Vertex morphs (表情) become blend shapes on a dedicated mesh, the way MMD
	# uses them: the morph moves vertices, so only the triangles it touches have
	# to be part of that mesh. The remaining triangles stay in the plain meshes.
	var morphs := _collect_vertex_morphs(pmx)
	var morph_vertices := _collect_morph_vertices(morphs)
	var morph_mesh := ImporterMesh.new()
	if not morphs.is_empty():
		for morph in morphs:
			morph_mesh.add_blend_shape(morph.m_name)
		# NORMALIZED stores absolute vertex positions, the same convention the
		# glTF importer uses for its morph targets (and the default of [Mesh]),
		# so the shapes stay correct even when the mode is not carried over by
		# the importer or the resource serializer.
		morph_mesh.set_blend_shape_mode(Mesh.BLEND_SHAPE_MODE_NORMALIZED)
	var morph_mesh_used := false

	var faceIndex = 0
	for material in pmx.m_materials:
		var indexs = PackedInt32Array()
		var morph_indexs = PackedInt32Array()
		var faceNum = material.m_numFaceVertices / 3
		for i in range(faceIndex, faceIndex+faceNum):
			var face = pmx.m_faces[i]
			if _face_uses_morph(face, morph_vertices):
				morph_indexs.append(face.m_vertice0)
				morph_indexs.append(face.m_vertice1)
				morph_indexs.append(face.m_vertice2)
			else:
				indexs.append(face.m_vertice0)
				indexs.append(face.m_vertice1)
				indexs.append(face.m_vertice2)
		faceIndex += faceNum

		# PMX stores -1 for materials without a texture.
		var surface_material: Material = null
		if textureMap.has(material.m_textureIndex):
			surface_material = textureMap[material.m_textureIndex]

		if not morph_indexs.is_empty():
			_add_morph_surface(morph_mesh, pmx.m_vertices, morphs, morph_indexs, surface_material)
			morph_mesh_used = true

		if indexs.is_empty():
			continue

		var surfaces: Array = []
		surfaces.resize(ArrayMesh.ARRAY_MAX)
		surfaces[ArrayMesh.ARRAY_VERTEX] = vertices
		surfaces[ArrayMesh.ARRAY_NORMAL] = normals
		surfaces[ArrayMesh.ARRAY_TEX_UV] = uvs
		surfaces[ArrayMesh.ARRAY_BONES] = bones
		surfaces[ArrayMesh.ARRAY_WEIGHTS] = weights
		surfaces[ArrayMesh.ARRAY_INDEX] = indexs
		var mesh = ImporterMesh.new()
		mesh.add_surface(Mesh.PRIMITIVE_TRIANGLES, surfaces, [], {}, surface_material)
		mesh.set_name(material.m_name)

		var mesh_node = ImporterMeshInstance3D.new()
		root_node.add_child(mesh_node, true)
		mesh_node.set_owner(root_node.get_owner())

		mesh_node.mesh = mesh
		mesh_node.skin = skin
		mesh_node.name = material.m_name
		mesh_node.skeleton_path = mesh_node.get_path_to(bone_node)

	if morph_mesh_used:
		var morph_node = ImporterMeshInstance3D.new()
		root_node.add_child(morph_node, true)
		morph_node.set_owner(root_node.get_owner())
		morph_node.mesh = morph_mesh
		morph_node.skin = skin
		morph_node.name = MORPH_NODE_NAME
		morph_node.skeleton_path = morph_node.get_path_to(bone_node)
		# The morph mesh is built after the material meshes, but it has to come
		# first: the node order is the draw order, and the face (the part the
		# morphs move) should be drawn with the same priority as before.
		root_node.move_child(morph_node, 0)

	return scene


## Returns the vertex (Position) morphs of the model, in file order. They are the
## ones Godot can drive as blend shapes; the other morph types (UV, bone,
## material, group) are not converted.
func _collect_vertex_morphs(pmx: PMXFile) -> Array:
	var morphs: Array = []
	for morph in pmx.m_morphs:
		if morph.m_morphType == PMXFile.PMXMorphType.Position and not morph.m_positionMorph.is_empty():
			morphs.append(morph)
	return morphs


## Returns the set of vertices moved by any of [param morphs].
func _collect_morph_vertices(morphs: Array) -> Dictionary:
	var indices := {}
	for morph in morphs:
		for position in morph.m_positionMorph:
			indices[position.m_vertexIndex] = true
	return indices


func _face_uses_morph(face: PMXFile.PMXFace, morph_vertices: Dictionary) -> bool:
	if morph_vertices.is_empty():
		return false
	return morph_vertices.has(face.m_vertice0) or morph_vertices.has(face.m_vertice1) or morph_vertices.has(face.m_vertice2)


## Adds one surface of the morph mesh. Only the vertices used by [param indexs]
## are written, and every vertex morph becomes a blend shape of absolute vertex
## positions, so [method MeshInstance3D.set_blend_shape_value] deforms the mesh
## exactly like the MMD morph does.
func _add_morph_surface(mesh: ImporterMesh, vertices: Array, morphs: Array, indexs: PackedInt32Array, material: Material) -> void:
	var remap := {}
	var order: Array[int] = []
	for index in indexs:
		if not remap.has(index):
			remap[index] = order.size()
			order.append(index)

	var positions := PackedVector3Array()
	var normals := PackedVector3Array()
	var uvs := PackedVector2Array()
	var bones := PackedInt32Array()
	var weights := PackedFloat32Array()
	for index in order:
		var vertex: PMXFile.PMXVertex = vertices[index]
		positions.append(vertex.m_position)
		normals.append(vertex.m_normal)
		uvs.append(vertex.m_uv)
		bones.append(vertex.m_boneId0)
		bones.append(vertex.m_boneId1)
		bones.append(vertex.m_boneId2)
		bones.append(vertex.m_boneId3)
		weights.append(vertex.m_boneWeight0)
		weights.append(vertex.m_boneWeight1)
		weights.append(vertex.m_boneWeight2)
		weights.append(vertex.m_boneWeight3)

	var remapped := PackedInt32Array()
	for index in indexs:
		remapped.append(remap[index])

	# The blend shape holds absolute positions (BLEND_SHAPE_MODE_NORMALIZED):
	# the PMX morph stores an offset, so it is added to the base vertex. Every
	# other vertex keeps its base position; with zeroed entries the whole surface
	# would collapse towards the morph offsets instead. MMD vertex morphs do not
	# change the normals, so the base normals are repeated.
	var blend_shapes: Array[Array] = []
	for morph in morphs:
		var targets := positions.duplicate()
		for position in morph.m_positionMorph:
			var target: int = remap.get(position.m_vertexIndex, -1)
			if target >= 0:
				targets[target] = positions[target] + position.m_position
		var shape: Array = []
		shape.resize(ArrayMesh.ARRAY_MAX)
		shape[ArrayMesh.ARRAY_VERTEX] = targets
		shape[ArrayMesh.ARRAY_NORMAL] = normals
		blend_shapes.append(shape)

	var surfaces: Array = []
	surfaces.resize(ArrayMesh.ARRAY_MAX)
	surfaces[ArrayMesh.ARRAY_VERTEX] = positions
	surfaces[ArrayMesh.ARRAY_NORMAL] = normals
	surfaces[ArrayMesh.ARRAY_TEX_UV] = uvs
	surfaces[ArrayMesh.ARRAY_BONES] = bones
	surfaces[ArrayMesh.ARRAY_WEIGHTS] = weights
	surfaces[ArrayMesh.ARRAY_INDEX] = remapped
	mesh.add_surface(Mesh.PRIMITIVE_TRIANGLES, surfaces, blend_shapes, {}, material)


## Stores the bone rules in the skeleton metadata and, unless the project works
## with baked motions, adds a [MMDPoseSolver] that applies 付与 (append) at
## runtime. The IK definitions in the same data are consumed by [MMDPoseBaker]
## to bake the IK chains and, when the solver is absent, the 付与 result into
## VMD animations.
func _create_pose_solver(pmx: PMXFile, bone_node: Skeleton3D, scene: Node3D, runtime_solver: bool) -> void:
	var data := _build_bone_data(pmx)
	if data.is_empty():
		return
	bone_node.set_meta(BONE_DATA_META, data)
	if not runtime_solver:
		return
	var solver := MMDPoseSolver.new()
	solver.name = SOLVER_NODE_NAME
	bone_node.add_child(solver)
	solver.set_owner(scene)


## Collects the MMD specific rules of every bone: the append (付与) parent with
## its weight and the IK chain with the per link angle limits. The runtime
## [MMDPoseSolver] consumes the append rule, [MMDPoseBaker] consumes both.
## The PMX transform order is kept so both can process parents first.
func _build_bone_data(pmx: PMXFile) -> Array:
	var data: Array = []
	for i in range(pmx.m_bones.size()):
		var bone: PMXFile.PMXBone = pmx.m_bones[i]
		var entry := {
			"bone": i,
			"order": bone.m_deformDepth,
			"append": -1,
			"weight": 0.0,
			"rot": false,
			"pos": false,
			"ik_target": -1,
			"ik_iter": 0,
			"ik_links": [],
		}

		var has_append := false
		if bone.m_appendBoneIndex >= 0 and bone.m_appendBoneIndex < pmx.m_bones.size() and not is_zero_approx(bone.m_appendWeight):
			if bone.m_boneFlag & PMXFile.PMXBoneFlags.AppendRotate:
				entry["rot"] = true
				has_append = true
			if bone.m_boneFlag & PMXFile.PMXBoneFlags.AppendTranslate:
				entry["pos"] = true
				has_append = true
			if has_append:
				entry["append"] = bone.m_appendBoneIndex
				entry["weight"] = bone.m_appendWeight

		var has_ik := false
		if (bone.m_boneFlag & PMXFile.PMXBoneFlags.IK) and not bone.m_ikLinks.is_empty() and bone.m_ikTargetBoneIndex >= 0:
			var links: Array = []
			for link in bone.m_ikLinks:
				# PMX limits are angles in MMD space. The MMD to Godot rotation
				# conversion mirrors the X and Y axes, so the intervals flip:
				# a knee limit of [-180, -0.5] becomes [+0.5, +180] in Godot.
				links.append({
					"bone": link.m_ikBoneIndex,
					"limit": link.m_enableLimit != 0,
					"min": Vector3(-link.m_limitMax.x, -link.m_limitMax.y, link.m_limitMin.z),
					"max": Vector3(-link.m_limitMin.x, -link.m_limitMin.y, link.m_limitMax.z),
				})
			entry["ik_target"] = bone.m_ikTargetBoneIndex
			entry["ik_iter"] = bone.m_ikIterationCount
			entry["ik_links"] = links
			has_ik = true

		if has_append or has_ik:
			data.append(entry)
	return data


## Stores the rigid bodies and joints of the file in the skeleton metadata and
## adds the runtime [MMDPhysicsSimulator] that drives them, unless the physics is
## disabled in the import dock. The bodies are what makes hair, skirts and breasts
## move: they are simulated and the result is written back into the bones.
func _create_physics(pmx: PMXFile, bone_node: Skeleton3D, scene: Node3D, enabled: bool) -> void:
	if not enabled or pmx.m_rigidbodies.is_empty():
		return
	var data := _build_physics_data(pmx)
	if data["rigid_bodies"].is_empty():
		return
	bone_node.set_meta(PHYSICS_DATA_META, data)
	var simulator := MMDPhysicsSimulator.new()
	simulator.name = PHYSICS_NODE_NAME
	simulator.set_influence(PHYSICS_INFLUENCE_VALUE)
	bone_node.add_child(simulator)
	simulator.set_owner(scene)
	print("MMD: %d rigid bodies, %d joints" % [data["rigid_bodies"].size(), data["joints"].size()])


## Collects the rigid bodies (剛体) and the joints (ジョイント) of the file.
## Transforms are converted to Godot space (scaled by the importer, mirrored like
## the mesh), so the runtime node only has to place them.
func _build_physics_data(pmx: PMXFile) -> Dictionary:
	var bone_count := pmx.m_bones.size()
	var rigid_bodies: Array = []
	for rigid_body in pmx.m_rigidbodies:
		var bone: int = rigid_body.m_boneIndex
		if bone < 0 or bone >= bone_count:
			bone = -1
		rigid_bodies.append({
			"name": rigid_body.m_name,
			"bone": bone,
			"group": rigid_body.m_group,
			"mask": rigid_body.m_collisionGroup,
			"shape": rigid_body.m_shape,
			"size": rigid_body.m_shapeSize,
			"transform": mmd_transform(rigid_body.m_translate, rigid_body.m_rotate),
			"mass": rigid_body.m_mass,
			"linear_damp": rigid_body.m_translateDimmer,
			"angular_damp": rigid_body.m_rotateDimmer,
			"friction": rigid_body.m_friction,
			"restitution": rigid_body.m_repulsion,
			"mode": rigid_body.m_op,
		})

	var joints: Array = []
	for joint in pmx.m_joints:
		var body_a: int = joint.m_rigidbodyAIndex
		var body_b: int = joint.m_rigidbodyBIndex
		if body_a < 0 or body_b < 0 or body_a >= rigid_bodies.size() or body_b >= rigid_bodies.size():
			push_warning("MMD: joint '%s' references a missing rigid body, skipped" % joint.m_name)
			continue
		joints.append({
			"name": joint.m_name,
			"body_a": body_a,
			"body_b": body_b,
			"transform": mmd_transform(joint.m_position, joint.m_rotate),
			# The limits are given in the frame of the joint, which is mirrored
			# with the model: the X and Y intervals flip (see _build_bone_data).
			"linear_min": joint.m_positionLowerLimit,
			"linear_max": joint.m_positionUpperLimit,
			"linear_spring": joint.m_springTranslateFactor,
			"angular_min": Vector3(-joint.m_rotateUpperLimit.x, -joint.m_rotateUpperLimit.y, joint.m_rotateLowerLimit.z),
			"angular_max": Vector3(-joint.m_rotateLowerLimit.x, -joint.m_rotateLowerLimit.y, joint.m_rotateUpperLimit.z),
			"angular_spring": joint.m_springRotateFactor,
		})
	return {"rigid_bodies": rigid_bodies, "joints": joints}


## Converts a transform of the PMX file to a Godot [Transform3D].
##
## PMX positions and rotations live in MMD space; the importer mirrors the model
## along Z like the vertex positions, so the basis is conjugated by the mirror.
## For the YXZ euler angles of the file that is the same as negating X and Y,
## which also turns a rotation axis into a pseudovector (x, y, z) -> (-x, -y, z).
static func mmd_transform(position: Vector3, rotation: Vector3) -> Transform3D:
	var euler := Vector3(-rotation.x, -rotation.y, rotation.z)
	return Transform3D(Basis.from_euler(euler, EULER_ORDER_YXZ), Vector3(position.x, position.y, -position.z))
