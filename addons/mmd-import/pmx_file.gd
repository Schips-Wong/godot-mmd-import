class_name PMXFile

# Thanks for https://www.cnblogs.com/ifwz/p/17544729.html

var errorString: String

enum PMXVertexWeight {
	BDEF1 = 0,
	BDEF2 = 1,
	BDEF4 = 2,
	SDEF = 3,
	QDEF = 4,
}

enum PMXDrawModeFlags {
	BothFace = 0x01,
	GroundShadow = 0x02,
	CastSelfShadow = 0x04,
	RecieveSelfShadow = 0x08,
	DrawEdge = 0x10,
	VertexColor = 0x20,
	DrawPoint = 0x40,
	DrawLine = 0x80,
}

enum PMXSphereMode {
	None,
	Mul,
	Add,
	SubTexture,
}

enum PMXToonMode {
	Separate,
	Common,
}

enum PMXBoneFlags {
	TargetShowMode = 0x0001,
	AllowRotate = 0x0002,
	AllowTranslate = 0x0004,
	Visible = 0x0008,
	AllowControl = 0x0010,
	IK = 0x0020,
	AppendLocal = 0x0080,
	AppendRotate = 0x0100,
	AppendTranslate = 0x0200,
	FixedAxis = 0x0400,
	LocalAxis = 0x800,
	DeformAfterPhysics = 0x1000,
	DeformOuterParent = 0x2000,
}

enum PMXMorphType {
	Group,
	Position,
	Bone,
	UV,
	AddUV1,
	AddUV2,
	AddUV3,
	AddUV4,
	kMaterial,
	Flip,
	Impluse,
}

enum PMXDisplayFrameTargetType {
	BoneIndex,
	MorphIndex,
}

enum PMXFrameType {
	DefaultFrame,
	SpecialFrame,
}

enum PMXShape {
	Sphere,
	Box,
	Capsule,
}

enum PMXOperation {
	Static,
	Dynamic,
	DynamicAndBoneMerge
}

enum PMXJointType {
	SpringDOF6,
	DOF6,
	P2P,
	ConeTwist,
	kSlider,
	Hinge,
}

enum PMXSoftbodyType {
	TriMesh,
	Rope,
}

enum PMXAeroModel {
	kAeroModelV_TwoSided,
	kAeroModelV_OneSided,
	kAeroModelF_TwoSided,
	kAeroModelF_OneSided,
}

static func check_header(head: PackedByteArray) -> bool:
	return (head.get(0) != 0x50
	and head.get(1) != 0x4D
	and head.get(2) != 0x58
	and head.get(3) != 0x20)
	
class PMXHeader:
	var m_magic: PackedByteArray
	var m_version: float
	var m_dataSize: int
	var m_encode: int # 0:UTF16 1:UTF8
	var m_addUVNum: int
	var m_vertexIndexSize: int
	var m_textureIndexSize: int
	var m_materialIndexSize: int
	var m_boneIndexSize: int
	var m_morphIndexSize: int
	var m_rigidbodyIndexSize: int

class PMXFileReader extends PMXHeader:
	var f: FileAccess
	func setup_header() -> bool:
		m_magic = f.get_buffer(4)
		m_version = f.get_float()
		m_dataSize = f.get_8()
		m_encode = f.get_8()
		m_addUVNum = f.get_8()
		m_vertexIndexSize = f.get_8()
		m_textureIndexSize = f.get_8()
		m_materialIndexSize = f.get_8()
		m_boneIndexSize = f.get_8()
		m_morphIndexSize = f.get_8()
		m_rigidbodyIndexSize = f.get_8()
		return PMXFile.check_header(m_magic)

	static func create(path: String) -> PMXFileReader:
		var r = PMXFileReader.new()
		r.f = FileAccess.open(path, FileAccess.READ)
		r.setup_header()
		return r

	func get_8():
		return f.get_8()
	func get_16():
		return f.get_16()
	func get_32():
		return f.get_32()
	func get_float():
		return f.get_float()

	func get_vec2() -> Vector2:
		var v: Vector2
		v.x = f.get_float()
		v.y = f.get_float()
		return v

	func get_vec3() -> Vector3:
		var v: Vector3 
		v.x = f.get_float()
		v.y = f.get_float()
		v.z = f.get_float()
		return v

	func get_vec4() -> Vector4:
		var v: Vector4
		v.x = f.get_float()
		v.y = f.get_float()
		v.z = f.get_float()
		v.w = f.get_float()
		return v

	func get_string() -> String:
		var strLen = f.get_32()
		var str = f.get_buffer(strLen)
		if m_encode == 1:
			return str.get_string_from_utf8()
		else:
			return str.get_string_from_utf16()

	func get_index(size: int) -> int:
		if 1 == size:
			var idx = f.get_8()
			if idx == 0xFF:
				return -1
			else:
				return idx
		if 2 == size:
			var idx = f.get_16()
			if idx == 0xFFFF:
				return -1
			else:
				return idx
		return f.get_32()
		
	func get_vertex_index():
		return get_index(m_vertexIndexSize)
		
	func get_texture_index() -> int:
		return get_index(m_textureIndexSize)
	
	func get_material_index() -> int:
		return get_index(m_materialIndexSize)

	func get_bone_index() -> int:
		return get_index(m_boneIndexSize)
	
	func get_morph_index() -> int:
		return get_index(m_morphIndexSize)

	func get_rigidbody_index() -> int:
		return get_index(m_rigidbodyIndexSize)

	func get_quaternion() -> Quaternion:
		var q = Quaternion()
		q.x = f.get_float()
		q.y = f.get_float()
		q.z = f.get_float()
		q.w = f.get_float()
		return q

class PMXInfo:
	var m_modelName: String
	var m_englishModelName: String
	var m_comment: String
	var m_englishComment: String

	func setup(f: PMXFileReader):
		m_modelName = f.get_string()
		m_englishModelName = f.get_string()
		m_comment = f.get_string()
		m_englishComment = f.get_string()
	pass

class PMXVertex:
	var m_position: Vector3
	var m_normal: Vector3
	var m_uv: Vector2
	var m_addUV: Array[Vector4] = []
	var m_weightType: int
	var m_boneId0: int = 0
	var m_boneId1: int = 0
	var m_boneId2: int = 0
	var m_boneId3: int = 0
	var m_boneWeight0: float = 0
	var m_boneWeight1: float = 0
	var m_boneWeight2: float = 0
	var m_boneWeight3: float = 0
	var m_sdefC: Vector3
	var m_sdefR0: Vector3
	var m_sdefR1: Vector3
	var m_edgeMag: float
	
	func setup(f: PMXFileReader) -> bool:
		m_position = f.get_vec3()
		m_normal = f.get_vec3()
		m_uv = f.get_vec2()
		m_addUV.resize(f.m_addUVNum)
		for i in range(f.m_addUVNum):
			m_addUV[i] = f.get_vec4()
		m_weightType = f.get_8()
		
		match m_weightType:
			PMXVertexWeight.BDEF1:
				m_boneId0 = f.get_bone_index()
				m_boneWeight0 = 1
				pass
			PMXVertexWeight.BDEF2:
				m_boneId0 = f.get_bone_index()
				m_boneId1 = f.get_bone_index()
				m_boneWeight0 = f.get_float()
				m_boneWeight1 = 1.0 - m_boneWeight0
				pass
			PMXVertexWeight.BDEF4, PMXVertexWeight.QDEF:
				m_boneId0 = f.get_bone_index()
				m_boneId1 = f.get_bone_index()
				m_boneId2 = f.get_bone_index()
				m_boneId3 = f.get_bone_index()
				m_boneWeight0 = f.get_float()
				m_boneWeight1 = f.get_float()
				m_boneWeight2 = f.get_float()
				m_boneWeight3 = f.get_float()
				pass
			PMXVertexWeight.SDEF:
				m_boneId0 = f.get_bone_index()
				m_boneId1 = f.get_bone_index()
				m_boneWeight0 = f.get_float()
				m_boneWeight1 = 1.0 - m_boneWeight0
				m_sdefC = f.get_vec3()
				m_sdefR0 = f.get_vec3()
				m_sdefR1 = f.get_vec3()
				pass
		pass
		m_edgeMag = f.get_float()
		return true
	pass
	
class PMXFace:
	var m_vertice0: int
	var m_vertice1: int
	var m_vertice2: int
	func setup(f: PMXFileReader) -> bool:
		match f.m_vertexIndexSize:
			1:
				m_vertice0 = f.get_8()
				m_vertice1 = f.get_8()
				m_vertice2 = f.get_8()
			2:
				m_vertice0 = f.get_16()
				m_vertice1 = f.get_16()
				m_vertice2 = f.get_16()
			4:
				m_vertice0 = f.get_32()
				m_vertice1 = f.get_32()
				m_vertice2 = f.get_32()
			_:
				return false
		pass
		return true
	pass

class PMXTexture:
	var m_textureName: String
	func setup(f: PMXFileReader) -> bool:
		m_textureName = f.get_string()
		return true
	pass
	
class PMXMaterial:
	var m_name: String
	var m_englishName: String
	var m_diffuse: Vector4
	var m_specular: Vector3
	var m_specularPower: float
	var m_ambient: Vector3
	var m_drawMode: int
	var m_edgeColor: Vector4
	var m_edgeScale: float
	var m_textureIndex: int
	var m_sphereTextureIndex: int
	var m_sphereMode: int
	var m_toonMode: int
	var m_toonTextureIndex: int
	var m_memo: String
	var m_numFaceVertices: int
	func setup(f: PMXFileReader) -> bool:
		m_name = f.get_string()
		m_englishName = f.get_string()
		m_diffuse = f.get_vec4()
		m_specular = f.get_vec3()
		m_specularPower = f.get_float()
		m_ambient = f.get_vec3()
		m_drawMode = f.get_8()
		m_edgeColor = f.get_vec4()
		m_edgeScale = f.get_float()
		m_textureIndex = f.get_texture_index()
		m_sphereTextureIndex = f.get_texture_index()
		m_sphereMode = f.get_8()
		m_toonMode = f.get_8()
		match m_toonMode:
			PMXToonMode.Separate:
				m_toonTextureIndex = f.get_texture_index()
			PMXToonMode.Common:
				m_toonTextureIndex = f.get_8()
			_:
				return false
		m_memo = f.get_string()
		m_numFaceVertices = f.get_32()
		return true
	pass
	
class PMXIKLink:
	var m_ikBoneIndex: int
	var m_enableLimit: int
	var m_limitMin: Vector3
	var m_limitMax: Vector3
	
	func setup(f: PMXFileReader) -> bool:
		m_ikBoneIndex = f.get_bone_index()
		m_enableLimit = f.get_8()
		if m_enableLimit != 0:
			m_limitMin = f.get_vec3()
			m_limitMax = f.get_vec3()
		return true
	pass
	
class PMXBone:
	var m_name: String
	var m_englishName: String
	var m_position: Vector3
	var m_parentBoneIndex: int
	var m_deformDepth: int
	var m_boneFlag: int
	var m_positionOffset: Vector3
	var m_linkBoneIndex: int
	var m_appendBoneIndex: int
	var m_appendWeight: float
	var m_fixedAxis: Vector3
	var m_localXAxis: Vector3
	var m_localZAxis: Vector3
	var m_keyValue: int
	var m_ikTargetBoneIndex: int
	var m_ikIterationCount: int
	var m_ikLimit: float
	var m_ikLinks: Array[PMXIKLink] = []

	func setup(f: PMXFileReader) -> bool:
		m_name = f.get_string()
		m_englishName = f.get_string()
		m_position = f.get_vec3()
		m_parentBoneIndex = f.get_bone_index()
		m_deformDepth = f.get_32()
		m_boneFlag = f.get_16()
		
		if (m_boneFlag & PMXBoneFlags.TargetShowMode) == 0:
			m_positionOffset = f.get_vec3()
		else:
			m_linkBoneIndex = f.get_bone_index()
		if ((m_boneFlag & PMXBoneFlags.AppendRotate) or (m_boneFlag & PMXBoneFlags.AppendTranslate)):
			m_appendBoneIndex = f.get_bone_index()
			m_appendWeight = f.get_float()
			pass
		if m_boneFlag & PMXBoneFlags.FixedAxis:
			m_fixedAxis = f.get_vec3()
			pass
		if m_boneFlag & PMXBoneFlags.LocalAxis:
			m_localXAxis = f.get_vec3()
			m_localZAxis = f.get_vec3()
			pass
		if m_boneFlag & PMXBoneFlags.DeformOuterParent:
			m_keyValue = f.get_32()
			pass
		if m_boneFlag & PMXBoneFlags.IK:
			m_ikTargetBoneIndex = f.get_bone_index()
			m_ikIterationCount = f.get_32()
			m_ikLimit = f.get_float()
			m_ikLinks.resize(f.get_32())
			for i in range(m_ikLinks.size()):
				m_ikLinks[i] = PMXIKLink.new()
				m_ikLinks[i].setup(f)
			pass
		return true
	func print():
		print(" m_name = ", m_name)
		print(" m_englishName = ", m_englishName)
		print(" m_position = ", m_position)
		print(" m_parentBoneIndex = ", m_parentBoneIndex)
	pass

class PMXMorphPosition:
	var m_vertexIndex: int
	var m_position: Vector3
	pass

class PMXMorphUV:
	var m_vertexIndex: int
	var m_uv: Vector4
	pass

class PMXMorphBone:
	var m_boneIndex: int
	var m_position: Vector3
	var m_quaternion: Quaternion
	pass

class PMXMorphMaterial:
	var m_materialIndex: int
	var m_opType: int
	var m_diffuse: Vector4
	var m_specular: Vector3
	var m_specularPower: float
	var m_ambient: Vector3
	var m_edgeColor: Vector4
	var m_edgeSize: float
	var m_textureFactor: Vector4
	var m_sphereTextureFactor: Vector4
	var m_toonTextureFactor: Vector4
	pass

class PMXMorphGroup:
	var m_morphIndex: int
	var m_weight: float
	pass
	
class PMXMorphFlip:
	var m_morphIndex: int
	var m_weight: float
	pass
	
class PMXMorphImpulse:
	var m_rigidbodyIndex: int
	var m_localFlag: int
	var m_translateVelocity: Vector3
	var m_rotateTorque: Vector3
	pass

class PMXMorph:
	var m_name: String
	var m_englishName: String
	var m_controlPanel: int
	var m_morphType: int
	var m_positionMorph: Array[PMXMorphPosition] = []
	var m_uvMorph: Array[PMXMorphUV] = []
	var m_boneMorph: Array[PMXMorphBone] = []
	var m_materialMorph: Array[PMXMorphMaterial] = []
	var m_groupMorph: Array[PMXMorphGroup] = []
	var m_flipMorph: Array[PMXMorphFlip] = []
	var m_impulseMorph: Array[PMXMorphImpulse] = []
	func setup(f: PMXFileReader) -> bool:
		m_name = f.get_string()
		m_englishName = f.get_string()
		m_controlPanel = f.get_8()
		m_morphType = f.get_8()
		var dataCount = f.get_32()
		match m_morphType:
			PMXMorphType.Position:
				m_positionMorph.resize(dataCount)
				for i in range(dataCount):
					var tmp = PMXMorphPosition.new()
					tmp.m_vertexIndex = f.get_vertex_index()
					tmp.m_position = f.get_vec3()
					m_positionMorph[i] = tmp
				pass
			PMXMorphType.UV, PMXMorphType.AddUV1, PMXMorphType.AddUV2, PMXMorphType.AddUV3, PMXMorphType.AddUV4:
				m_uvMorph.resize(dataCount)
				for i in range(dataCount):
					var tmp = PMXMorphUV.new()
					tmp.m_vertexIndex = f.get_vertex_index()
					tmp.m_uv = f.get_vec4()
					m_uvMorph[i] = tmp
				pass
			PMXMorphType.Bone:
				m_boneMorph.resize(dataCount)
				for i in range(dataCount):
					var tmp = PMXMorphBone.new()
					tmp.m_boneIndex = f.get_bone_index()
					tmp.m_position = f.get_vec3()
					tmp.m_quaternion = f.get_quaternion()
					m_boneMorph[i] = tmp
				pass
			PMXMorphType.kMaterial:
				m_materialMorph.resize(dataCount)
				for i in range(dataCount):
					var tmp = PMXMorphMaterial.new()
					tmp.m_materialIndex = f.get_material_index()
					tmp.m_opType = f.get_8()
					tmp.m_diffuse = f.get_vec4()
					tmp.m_specular = f.get_vec3()
					tmp.m_specularPower = f.get_float()
					tmp.m_ambient = f.get_vec3()
					tmp.m_edgeColor = f.get_vec4()
					tmp.m_edgeSize = f.get_float()
					tmp.m_textureFactor = f.get_vec4()
					tmp.m_sphereTextureFactor = f.get_vec4()
					tmp.m_toonTextureFactor = f.get_vec4()
					m_materialMorph[i] = tmp
				pass
			PMXMorphType.Group:
				m_groupMorph.resize(dataCount)
				for i in range(dataCount):
					var tmp = PMXMorphGroup.new()
					tmp.m_morphIndex = f.get_morph_index()
					tmp.m_weight = f.get_float()
					m_groupMorph[i] = tmp
				pass
			PMXMorphType.Flip:
				m_flipMorph.resize(dataCount)
				for i in range(dataCount):
					var tmp = PMXMorphFlip.new()
					tmp.m_morphIndex = f.get_morph_index()
					tmp.m_weight = f.get_float()
					m_flipMorph[i] = tmp
				pass
			PMXMorphType.Impluse:
				m_impulseMorph.resize(dataCount)
				for i in range(dataCount):
					var tmp = PMXMorphImpulse.new()
					tmp.m_rigidbodyIndex = f.get_rigidbody_index()
					tmp.m_localFlag = f.get_8()
					tmp.m_translateVelocity = f.get_vec3()
					tmp.m_rotateTorque = f.get_vec3()
					m_impulseMorph[i] = tmp
				pass
			_:
				return false
		pass
		return true
	pass
	
class PMXDisplayFrameTarget:
	var m_type: int
	var m_index: int
	pass
	
class PMXDisplayFrame:
	var m_name: String
	var m_englishName: String
	var m_flag: int
	var m_targets: Array[PMXDisplayFrameTarget] = []
	func setup(f: PMXFileReader) -> bool:
		m_name = f.get_string()
		m_englishName = f.get_string()
		m_flag = f.get_8()
		m_targets.resize(f.get_32())
		for i in range(m_targets.size()):
			var tmp = PMXDisplayFrameTarget.new()
			tmp.m_type = f.get_8()
			match tmp.m_type:
				PMXDisplayFrameTargetType.BoneIndex:
					tmp.m_index = f.get_bone_index()
					pass
				PMXDisplayFrameTargetType.MorphIndex:
					tmp.m_index = f.get_morph_index()
					pass
				_:
					return false
			m_targets[i] = tmp
		pass
		return true
	pass
	
class PMXRigidbody:
	var m_name: String
	var m_englishName: String
	var m_boneIndex: int
	var m_group: int
	var m_collisionGroup: int
	var m_shape: int
	var m_shapeSize: Vector3
	var m_translate: Vector3
	var m_rotate: Vector3
	var m_mass: float
	var m_translateDimmer: float
	var m_rotateDimmer: float
	var m_repulsion: float
	var m_friction: float
	var m_op: int
	func setup(f: PMXFileReader):
		m_name = f.get_string()
		m_englishName = f.get_string()
		m_boneIndex = f.get_bone_index()
		m_group = f.get_8()
		m_collisionGroup = f.get_16()
		m_shape = f.get_8()
		m_shapeSize = f.get_vec3()
		m_translate = f.get_vec3()
		m_rotate = f.get_vec3()
		m_mass = f.get_float()
		m_translateDimmer = f.get_float()
		m_rotateDimmer = f.get_float()
		m_repulsion = f.get_float()
		m_friction = f.get_float()
		m_op = f.get_8()
		pass
	pass
	
class PMXJoint:
	var m_name: String
	var m_englishName: String
	var m_type: int
	var m_rigidbodyAIndex: int
	var m_rigidbodyBIndex: int
	var m_position: Vector3
	var m_rotate: Vector3
	var m_positionLowerLimit: Vector3
	var m_positionUpperLimit: Vector3
	var m_rotateLowerLimit: Vector3
	var m_rotateUpperLimit: Vector3
	var m_springTranslateFactor: Vector3
	var m_springRotateFactor: Vector3
	func setup(f: PMXFileReader):
		m_name = f.get_string()
		m_englishName = f.get_string()
		m_type = f.get_8()
		m_rigidbodyAIndex = f.get_rigidbody_index()
		m_rigidbodyBIndex = f.get_rigidbody_index()
		m_position = f.get_vec3()
		m_rotate = f.get_vec3()
		m_positionLowerLimit = f.get_vec3()
		m_positionUpperLimit = f.get_vec3()
		m_rotateLowerLimit = f.get_vec3()
		m_rotateUpperLimit = f.get_vec3()
		m_springTranslateFactor = f.get_vec3()
		m_springRotateFactor = f.get_vec3()
		pass
	pass
	
class PMXAnchorRigidbody:
	var m_rigidBodyIndex: int
	var m_vertexIndex: int
	var m_nearMode: int
	func setup(f: PMXFileReader):
		m_rigidBodyIndex = f.get_32()
		m_vertexIndex = f.get_32()
		m_nearMode = f.get_8()
	pass
	
class PMXSoftbody:
	var m_name: String
	var m_englishName: String
	var m_type: int
	var m_materialIndex: int
	var m_group: int
	var m_collisionGroup: int
	var m_flag: int
	var m_BLinkLength: int
	var m_numClusters: int
	var m_totalMass: int
	var m_collisionMargin: int
	var m_aeroModel: int
	# config
	var m_VCF: float
	var m_DP: float
	var m_DG: float
	var m_LF: float
	var m_PR: float
	var m_VC: float
	var m_DF: float
	var m_MT: float
	var m_CHR: float
	var m_KHR: float
	var m_SHR: float
	var m_AHR: float
	# cluster
	var m_SRHR_CL: float
	var m_SKHR_CL: float
	var m_SSHR_CL: float
	var m_SR_SPLT_CL: float
	var m_SK_SPLT_CL: float
	var m_SS_SPLT_CL: float
	# interation
	var m_V_IT: int
	var m_P_IT: int
	var m_D_IT: int
	var m_C_IT: int
	# material
	var m_LST: float
	var m_AST: float
	var m_VST: float
	var m_anchorRigidbodies: Array[PMXAnchorRigidbody] = []
	var m_pinVertexIndices: Array[int] = []
	func setup(f: PMXFileReader):
		m_name = f.get_string()
		m_englishName = f.get_string()
		m_type = f.get_8()
		m_materialIndex = f.get_material_index()
		m_group = f.get_8()
		m_collisionGroup = f.get_32()
		m_flag = f.get_8()
		m_BLinkLength = f.get_32()
		m_numClusters = f.get_32()
		m_totalMass = f.get_float()
		m_collisionMargin = f.get_float()
		m_aeroModel = f.get_32()
		m_VCF = f.get_float()
		m_DP = f.get_float()
		m_DG = f.get_float()
		m_LF = f.get_float()
		m_PR = f.get_float()
		m_VC = f.get_float()
		m_DF = f.get_float()
		m_MT = f.get_float()
		m_CHR = f.get_float()
		m_KHR = f.get_float()
		m_SHR = f.get_float()
		m_AHR = f.get_float()
		m_SRHR_CL = f.get_float()
		m_SKHR_CL = f.get_float()
		m_SSHR_CL = f.get_float()
		m_SR_SPLT_CL = f.get_float()
		m_SK_SPLT_CL = f.get_float()
		m_SS_SPLT_CL = f.get_float()
		m_V_IT = f.get_32()
		m_P_IT = f.get_32()
		m_D_IT = f.get_32()
		m_C_IT = f.get_32()
		m_LST = f.get_float()
		m_AST = f.get_float()
		m_VST = f.get_float()
		m_anchorRigidbodies.resize(f.get_32())
		for i in range(m_anchorRigidbodies.size()):
			var tmp = PMXAnchorRigidbody.new()
			tmp.m_rigidBodyIndex = f.get_rigidbody_index()
			tmp.m_vertexIndex = f.get_vertex_index()
			tmp.m_nearMode = f.get_8()
			m_anchorRigidbodies[i] = tmp
		m_pinVertexIndices.resize(f.get_32())
		for i in range(m_pinVertexIndices.size()):
			m_pinVertexIndices[i] = f.get_vertex_index()
		pass
	pass

var m_head: PMXHeader
var m_info: PMXInfo
var m_vertices: Array[PMXVertex] = []
var m_faces: Array[PMXFace] = []
var m_textures: Array[PMXTexture] = []
var m_materials: Array[PMXMaterial] = []
var m_bones: Array[PMXBone] = []
var m_morphs: Array[PMXMorph] = []
var m_displayFrames: Array[PMXDisplayFrame] = []
var m_rigidbodies: Array[PMXRigidbody] = []
var m_joints: Array[PMXJoint] = []
var m_softbodies: Array[PMXSoftbody] = []

func setup(path: String):
	var r = PMXFileReader.create(path)
	m_head = r
	m_info = PMXInfo.new()
	m_info.setup(r)
	m_vertices.resize(r.get_32())
	for i in range(m_vertices.size()):
		m_vertices[i] = PMXVertex.new()
		m_vertices[i].setup(r)
	m_faces.resize(r.get_32() / 3)
	for i in range(m_faces.size()):
		m_faces[i] = PMXFace.new()
		m_faces[i].setup(r)
	m_textures.resize(r.get_32())
	for i in range(m_textures.size()):
		m_textures[i] = PMXTexture.new()
		m_textures[i].setup(r)
	m_materials.resize(r.get_32())
	for i in range(m_materials.size()):
		m_materials[i] = PMXMaterial.new()
		m_materials[i].setup(r)
	m_bones.resize(r.get_32())
	for i in range(m_bones.size()):
		m_bones[i] = PMXBone.new()
		m_bones[i].setup(r)
	m_morphs.resize(r.get_32())
	for i in range(m_morphs.size()):
		m_morphs[i] = PMXMorph.new()
		m_morphs[i].setup(r)
	m_displayFrames.resize(r.get_32())
	for i in range(m_displayFrames.size()):
		m_displayFrames[i] = PMXDisplayFrame.new()
		m_displayFrames[i].setup(r)
	m_rigidbodies.resize(r.get_32())
	for i in range(m_rigidbodies.size()):
		m_rigidbodies[i] = PMXRigidbody.new()
		m_rigidbodies[i].setup(r)
	m_joints.resize(r.get_32())
	for i in range(m_joints.size()):
		m_joints[i] = PMXJoint.new()
		m_joints[i].setup(r)
	r.f.close()
	
func scale(s: float):
	for v in m_vertices:
		v.m_position *= s
		pass
	for v in m_bones:
		v.m_position *= s
	for v in m_joints:
		v.m_position *= s
		v.m_positionLowerLimit *= s
		v.m_positionUpperLimit *= s
	for v in m_morphs:
		if v.m_positionMorph != null:
			for i in v.m_positionMorph:
				i.m_position *= s
			pass
		if v.m_boneMorph != null:
			for i in v.m_boneMorph:
				i.m_position *= s
	pass
