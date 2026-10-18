class_name VMDFile

## Parser for MMD motion files (.vmd).
##
## Layout of a VMD file:
## [codeblock]
## header
##   signature (30 bytes, "Vocaloid Motion Data 0002")
##   model name (20 bytes in 0002, 10 bytes in the old format)
## bone key frames   : name(15) frame(4) position(12) rotation(16) interpolation(64)
## morph key frames  : name(15) frame(4) weight(4)
## camera key frames : frame(4) distance(4) position(12) rotation(12) interpolation(24) fov(4) perspective(1)
## light key frames  : frame(4) color(12) direction(12)
## shadow key frames : frame(4) mode(1) distance(4)
## ik/visibility keys: frame(4) visible(1) count(4) [name(20) enabled(1)] * count
## [/codeblock]
## All names are Shift-JIS (CP932) encoded and NUL padded, see [MMDShiftJIS].
## Every section after the bone section is optional; older files simply end.

const SIGNATURE_V2 := "Vocaloid Motion Data 0002"
const SIGNATURE_V1 := "Vocaloid Motion Data file"
const SIGNATURE_SIZE := 30
const MODEL_NAME_SIZE_V2 := 20
const MODEL_NAME_SIZE_V1 := 10
const BONE_NAME_SIZE := 15
const IK_NAME_SIZE := 20

## Guards against corrupted frame counts (a bad count would otherwise allocate
## gigabytes before the file runs out).
const MAX_KEY_FRAME_COUNT := 2000000

const BONE_KEY_FRAME_SIZE := BONE_NAME_SIZE + 4 + 12 + 16 + 64
const MORPH_KEY_FRAME_SIZE := BONE_NAME_SIZE + 4 + 4
const CAMERA_KEY_FRAME_SIZE := 4 + 4 + 12 + 12 + 24 + 4 + 1
const LIGHT_KEY_FRAME_SIZE := 4 + 12 + 12
const SHADOW_KEY_FRAME_SIZE := 4 + 1 + 4

## Bezier channel order inside the 64 byte bone interpolation block.
enum BezierChannel {
	X = 0,
	Y = 1,
	Z = 2,
	ROTATION = 3,
}

var errorString: String = ""
var m_header: VMDHeader
## Bone name -> Array[VMDBoneKeyFrame]
var m_boneKeyFrames: Dictionary[String, Array] = {}
## Morph name -> Array[VMDMorphKeyFrame]
var m_morphKeyFrames: Dictionary[String, Array] = {}
var m_cameraKeyFrames: Array[VMDCameraKeyFrame] = []
var m_lightKeyFrames: Array[VMDLightKeyFrame] = []
var m_shadowKeyFrames: Array[VMDShadowKeyFrame] = []
var m_propertyKeyFrames: Array[VMDPropertyKeyFrame] = []


class VMDReader:
	var f: FileAccess
	var m_size: int = 0

	static func create(path: String) -> VMDReader:
		var file: FileAccess = FileAccess.open(path, FileAccess.READ)
		if file == null:
			return null
		var r := VMDReader.new()
		r.f = file
		r.m_size = file.get_length()
		return r

	func close() -> void:
		if f != null:
			f.close()
			f = null

	func remaining() -> int:
		return m_size - f.get_position()

	func has(count: int) -> bool:
		return count >= 0 and remaining() >= count

	func get_bytes(count: int) -> PackedByteArray:
		return f.get_buffer(count)

	func get_string(count: int) -> String:
		return MMDShiftJIS.decode(f.get_buffer(count), count)

	func get_uint8() -> int:
		return f.get_8()

	func get_uint32() -> int:
		return f.get_32()

	func get_float() -> float:
		return f.get_float()

	func get_vec3() -> Vector3:
		var v := Vector3()
		v.x = f.get_float()
		v.y = f.get_float()
		v.z = f.get_float()
		return v

	func get_quaternion() -> Quaternion:
		var q := Quaternion()
		q.x = f.get_float()
		q.y = f.get_float()
		q.z = f.get_float()
		q.w = f.get_float()
		return q


class VMDHeader:
	var m_signature: String
	var m_modelName: String


class VMDBoneKeyFrame:
	var m_frame: int
	var m_position: Vector3
	var m_rotation: Quaternion
	## 64 raw interpolation bytes, see [method VMDFile.get_bezier].
	var m_interpolation: PackedByteArray


class VMDMorphKeyFrame:
	var m_frame: int
	var m_weight: float


class VMDCameraKeyFrame:
	var m_frame: int
	var m_distance: float
	## Look-at target position.
	var m_position: Vector3
	## Euler rotation in MMD space (X, Y, Z).
	var m_rotation: Vector3
	var m_interpolation: PackedByteArray
	## Vertical field of view in degrees.
	var m_angle: float
	var m_perspective: bool = true


class VMDLightKeyFrame:
	var m_frame: int
	var m_color: Color
	## Direction the light points to in MMD space.
	var m_direction: Vector3


class VMDShadowKeyFrame:
	var m_frame: int
	## 0: none, 1: mode 1, 2: mode 2
	var m_mode: int
	## Shadow distance converted to MMD's 0-10000 range.
	var m_distance: float


class VMDPropertyKeyFrame:
	var m_frame: int
	var m_visible: bool = true
	## Array of [name: String, enabled: bool] IK toggles.
	var m_ikStates: Array = []


## Returns the bezier control points of one channel as (x1, y1, x2, y2)
## normalized to 0..1. MMD stores each channel as 4 bytes spread over a
## 16 byte block.
static func get_bezier(interpolation: PackedByteArray, channel: int) -> Vector4:
	if interpolation.size() < 64 or channel < 0 or channel > 3:
		return Vector4(20.0 / 127.0, 20.0 / 127.0, 107.0 / 127.0, 107.0 / 127.0)
	var base := channel * 16
	var v := Vector4()
	v.x = float(interpolation[base]) / 127.0
	v.y = float(interpolation[base + 4]) / 127.0
	v.z = float(interpolation[base + 8]) / 127.0
	v.w = float(interpolation[base + 12]) / 127.0
	return v


func setup(path: String) -> bool:
	errorString = ""
	m_header = VMDHeader.new()
	m_boneKeyFrames.clear()
	m_morphKeyFrames.clear()
	m_cameraKeyFrames.clear()
	m_lightKeyFrames.clear()
	m_shadowKeyFrames.clear()
	m_propertyKeyFrames.clear()

	var r := VMDReader.create(path)
	if r == null:
		errorString = "failed to open %s" % path
		return false

	m_header.m_signature = r.get_string(SIGNATURE_SIZE)
	var model_name_size := 0
	if m_header.m_signature == SIGNATURE_V2:
		model_name_size = MODEL_NAME_SIZE_V2
	elif m_header.m_signature == SIGNATURE_V1:
		model_name_size = MODEL_NAME_SIZE_V1
	else:
		errorString = "invalid VMD signature: %s" % m_header.m_signature
		r.close()
		return false
	m_header.m_modelName = r.get_string(model_name_size)

	var ok := _load_bone_key_frames(r)
	if ok and m_header.m_signature == SIGNATURE_V2:
		ok = _load_morph_key_frames(r)
		if ok:
			ok = _load_camera_key_frames(r)
		if ok:
			ok = _load_light_key_frames(r)
		if ok:
			ok = _load_shadow_key_frames(r)
		if ok:
			ok = _load_property_key_frames(r)
	r.close()
	return ok


func _load_bone_key_frames(r: VMDReader) -> bool:
	var count := r.get_uint32()
	if count < 0 or count > MAX_KEY_FRAME_COUNT or not r.has(count * BONE_KEY_FRAME_SIZE):
		errorString = "invalid bone key frame count: %d" % count
		return false
	for _i in range(count):
		var name := r.get_string(BONE_NAME_SIZE)
		var key := VMDBoneKeyFrame.new()
		key.m_frame = r.get_uint32()
		key.m_position = r.get_vec3()
		key.m_rotation = r.get_quaternion()
		if is_zero_approx(key.m_rotation.length_squared()):
			key.m_rotation = Quaternion.IDENTITY
		key.m_interpolation = r.get_bytes(64)
		if not m_boneKeyFrames.has(name):
			m_boneKeyFrames[name] = []
		m_boneKeyFrames[name].append(key)
	return true


func _load_morph_key_frames(r: VMDReader) -> bool:
	m_morphKeyFrames.clear()
	if not r.has(4):
		return true
	var count := r.get_uint32()
	if count < 0 or count > MAX_KEY_FRAME_COUNT or not r.has(count * MORPH_KEY_FRAME_SIZE):
		errorString = "invalid morph key frame count: %d" % count
		return false
	for _i in range(count):
		var name := r.get_string(BONE_NAME_SIZE)
		var key := VMDMorphKeyFrame.new()
		key.m_frame = r.get_uint32()
		key.m_weight = r.get_float()
		if not m_morphKeyFrames.has(name):
			m_morphKeyFrames[name] = []
		m_morphKeyFrames[name].append(key)
	return true


func _load_camera_key_frames(r: VMDReader) -> bool:
	m_cameraKeyFrames.clear()
	if not r.has(4):
		return true
	var count := r.get_uint32()
	if count < 0 or count > MAX_KEY_FRAME_COUNT or not r.has(count * CAMERA_KEY_FRAME_SIZE):
		# Trailing garbage is common, keep what was parsed so far.
		return true
	for _i in range(count):
		var key := VMDCameraKeyFrame.new()
		key.m_frame = r.get_uint32()
		key.m_distance = r.get_float()
		key.m_position = r.get_vec3()
		key.m_rotation = r.get_vec3()
		key.m_interpolation = r.get_bytes(24)
		key.m_angle = float(r.get_uint32())
		key.m_perspective = r.get_uint8() == 0
		m_cameraKeyFrames.append(key)
	return true


func _load_light_key_frames(r: VMDReader) -> bool:
	m_lightKeyFrames.clear()
	if not r.has(4):
		return true
	var count := r.get_uint32()
	if count < 0 or count > MAX_KEY_FRAME_COUNT or not r.has(count * LIGHT_KEY_FRAME_SIZE):
		return true
	for _i in range(count):
		var key := VMDLightKeyFrame.new()
		key.m_frame = r.get_uint32()
		var color := r.get_vec3()
		key.m_color = Color(color.x, color.y, color.z)
		key.m_direction = r.get_vec3()
		m_lightKeyFrames.append(key)
	return true


func _load_shadow_key_frames(r: VMDReader) -> bool:
	m_shadowKeyFrames.clear()
	if not r.has(4):
		return true
	var count := r.get_uint32()
	if count < 0 or count > MAX_KEY_FRAME_COUNT or not r.has(count * SHADOW_KEY_FRAME_SIZE):
		return true
	for _i in range(count):
		var key := VMDShadowKeyFrame.new()
		key.m_frame = r.get_uint32()
		key.m_mode = r.get_uint8()
		# MMD stores 1.0 for the shortest and 0.0 for the longest distance.
		key.m_distance = 10000.0 - r.get_float() * 100000.0
		m_shadowKeyFrames.append(key)
	return true


func _load_property_key_frames(r: VMDReader) -> bool:
	m_propertyKeyFrames.clear()
	if not r.has(4):
		return true
	var count := r.get_uint32()
	if count < 0 or count > MAX_KEY_FRAME_COUNT:
		return true
	for _i in range(count):
		if not r.has(9):
			return true
		var key := VMDPropertyKeyFrame.new()
		key.m_frame = r.get_uint32()
		key.m_visible = r.get_uint8() != 0
		var ik_count := r.get_uint32()
		if ik_count < 0 or ik_count > 4096:
			return true
		for _j in range(ik_count):
			if not r.has(IK_NAME_SIZE + 1):
				return true
			var name := r.get_string(IK_NAME_SIZE)
			var enabled := r.get_uint8() != 0
			key.m_ikStates.append([name, enabled])
		m_propertyKeyFrames.append(key)
	return true


func get_bone_key_frame_count() -> int:
	var total := 0
	for name in m_boneKeyFrames:
		total += m_boneKeyFrames[name].size()
	return total


## Returns the last frame number used by any key frame, in MMD's 30 fps time base.
func get_last_frame() -> int:
	var last := 0
	for name in m_boneKeyFrames:
		for key in m_boneKeyFrames[name]:
			last = maxi(last, key.m_frame)
	for name in m_morphKeyFrames:
		for key in m_morphKeyFrames[name]:
			last = maxi(last, key.m_frame)
	for key in m_cameraKeyFrames:
		last = maxi(last, key.m_frame)
	for key in m_lightKeyFrames:
		last = maxi(last, key.m_frame)
	return last
