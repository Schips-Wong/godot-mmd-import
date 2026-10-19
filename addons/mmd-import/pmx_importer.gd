class_name PMXImporter extends EditorSceneFormatImporter

## Skeleton3D metadata consumed by MMDPoseSolver and MMDPoseBaker.
const BONE_DATA_META := "mmd_bone_data"
const SOLVER_NODE_NAME := "MMDPoseSolver"

## Adds the runtime [MMDPoseSolver] node that applies 付与 (append). Disable it
## when the motions are baked by [MMDPoseBaker], which writes the 付与 result
## into the animation instead. The bone data metadata is written either way.
const OPTION_RUNTIME_SOLVER := "mmd/runtime_pose_solver"


func _get_extensions() -> PackedStringArray:
	return PackedStringArray(["pmx"])

func _get_import_options(_path: String) -> void:
	add_import_option(OPTION_RUNTIME_SOLVER, true)


func _import_scene(path: String, _flags: int, options: Dictionary) -> Object:
	var pmx = PMXFile.new()
	pmx.setup(path)
	pmx.scale(0.1)
	pmx_transform_to_godot(pmx)
	var runtime_solver := bool(options.get(OPTION_RUNTIME_SOLVER, true))
	return create_mesh_node(pmx, 0.1, path.get_base_dir(), runtime_solver)


func pmx_transform_to_godot(pmx):
	for v in pmx.m_vertices:
		v.m_position.z *= -1
		v.m_normal.z *= -1
	for b in pmx.m_bones:
		b.m_position.z *= -1
		b.m_positionOffset.z *= -1
	pass


func create_mesh_node(pmx: PMXFile, scale: float, base_dir: String, runtime_solver: bool = true) -> Object:
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
	
	scene.add_child(root_node)
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
	
	var faceIndex = 0
	for material in pmx.m_materials:
		var indexs = PackedInt32Array()
		var faceNum = material.m_numFaceVertices / 3
		for i in range(faceIndex, faceIndex+faceNum):
			var face = pmx.m_faces[i]
			indexs.append(face.m_vertice0)
			indexs.append(face.m_vertice1)
			indexs.append(face.m_vertice2)
		faceIndex += faceNum

		var surfaces: Array = []
		surfaces.resize(ArrayMesh.ARRAY_MAX)
		surfaces[ArrayMesh.ARRAY_VERTEX] = vertices
		surfaces[ArrayMesh.ARRAY_NORMAL] = normals
		surfaces[ArrayMesh.ARRAY_TEX_UV] = uvs
		surfaces[ArrayMesh.ARRAY_BONES] = bones
		surfaces[ArrayMesh.ARRAY_WEIGHTS] = weights
		surfaces[ArrayMesh.ARRAY_INDEX] = indexs
		var mesh = ImporterMesh.new()
		# PMX stores -1 for materials without a texture.
		var surface_material: Material = null
		if textureMap.has(material.m_textureIndex):
			surface_material = textureMap[material.m_textureIndex]
		mesh.add_surface(Mesh.PRIMITIVE_TRIANGLES, surfaces, [], {}, surface_material)
		mesh.set_name(material.m_name)

		var mesh_node = ImporterMeshInstance3D.new()
		root_node.add_child(mesh_node, true)
		mesh_node.set_owner(root_node.get_owner())

		mesh_node.mesh = mesh
		mesh_node.skin = skin
		mesh_node.name = material.m_name
		mesh_node.skeleton_path = mesh_node.get_path_to(bone_node)

	return scene


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
