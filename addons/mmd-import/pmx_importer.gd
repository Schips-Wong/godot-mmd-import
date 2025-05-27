class_name PMXImporter extends EditorSceneFormatImporter


func _get_extensions() -> PackedStringArray:
	return PackedStringArray(["pmx"])

func _import_scene(path: String, _flags: int, _options: Dictionary) -> Object:
	var pmx = PMXFile.new()
	pmx.setup(path)
	pmx.scale(0.1)
	pmx_transform_to_godot(pmx)
	return create_mesh_node(pmx, 0.1, path.get_base_dir())


func pmx_transform_to_godot(pmx):
	for v in pmx.m_vertices:
		v.m_position.z *= -1
		v.m_normal.z *= -1
	for b in pmx.m_bones:
		b.m_position.z *= -1
		b.m_positionOffset.z *= -1
	pass


func create_mesh_node(pmx: PMXFile, scale: float, base_dir: String) -> Object:
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

	# Create Skin
	for bone in pmx.m_bones:
		skin.add_named_bind(bone.m_name, Transform3D(Basis.IDENTITY, -bone.m_position))
	
	var textureMap: Dictionary[int, StandardMaterial3D] = {}
	for i in range(pmx.m_textures.size()):
		var textureName = pmx.m_textures[i].m_textureName
		if textureName.ends_with("0Nub"):
			continue
		var mt3d = StandardMaterial3D.new()
		var texturePath:String = textureName.replace('\\', '/')
		var resourcePath:String = base_dir + "/" + texturePath

		mt3d.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR_WITH_MIPMAPS
		mt3d.cull_mode = BaseMaterial3D.CULL_BACK
		mt3d.albedo_texture = load(resourcePath)
		mt3d.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		mt3d.depth_draw_mode = BaseMaterial3D.DEPTH_DRAW_ALWAYS
		mt3d.resource_name = texturePath.get_file()
		mt3d.render_priority = i
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
		
		mesh.add_surface(Mesh.PRIMITIVE_TRIANGLES, surfaces, [], {}, textureMap[material.m_textureIndex])
		mesh.set_name(material.m_name)

		var mesh_node = ImporterMeshInstance3D.new()
		root_node.add_child(mesh_node, true)
		mesh_node.set_owner(root_node.get_owner())

		mesh_node.mesh = mesh
		mesh_node.skin = skin
		mesh_node.name = material.m_name
		mesh_node.skeleton_path = mesh_node.get_path_to(bone_node)

	return scene
