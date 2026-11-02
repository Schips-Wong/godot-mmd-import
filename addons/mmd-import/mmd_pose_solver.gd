@tool
class_name MMDPoseSolver extends SkeletonModifier3D

## Applies the 付与 (append / grant) bone rule that plain [Animation] tracks
## cannot express.
##
## A bone inherits a weighted part of another bone's animation transform (twist
## bones, D bones, 肩C, 目戻...). Negative weights invert the inherited rotation.
## Bones are processed in the PMX transform order.
##
## IK is not solved at runtime: [VMDImporter] bakes the IK chains into ordinary
## rotation tracks, see [MMDPoseBaker]. The 付与 rule is baked as well, so a
## baked motion plays correctly without this node (exported game, glTF export,
## third party tools). Animations carrying [constant MMDPoseBaker.BAKED_META]
## already contain the rule result and are skipped here, otherwise the inherited
## transform would be applied twice.
##
## Baked motions are recognised through the [AnimationPlayer] that drives the
## skeleton. Clips played through an [AnimationTree] leave that player's current
## animation empty and are therefore not recognised: bake those clips and remove
## this node, or keep them unbaked.
##
## The bone data is written by [PMXImporter] into the [Skeleton3D] metadata
## "mmd_bone_data". The engine runs this node after the [AnimationMixer] and
## restores the unmodified poses afterwards, so the skin uses the solved pose
## while the animation keeps working on a clean pose every frame.

const DATA_META := "mmd_bone_data"
## Mirrors [constant MMDPoseBaker.BAKED_META]; kept local so the solver does not
## depend on the baker class being registered in exported projects.
const BAKED_META := "mmd_append_baked"

var _entries: Array = []
var _bone_count := -1
## "instance id/animation name" -> bool, see [method _plays_baked_animation].
var _baked_cache := {}


func _process_modification() -> void:
	var skeleton := get_skeleton()
	if skeleton == null:
		return
	if _plays_baked_animation(skeleton):
		return
	if _bone_count != skeleton.get_bone_count():
		_build(skeleton)
	for entry in _entries:
		apply_append(skeleton, entry)


func _skeleton_changed(_old_skeleton: Skeleton3D, new_skeleton: Skeleton3D) -> void:
	_bone_count = -1
	_entries.clear()
	_baked_cache.clear()
	if new_skeleton != null:
		_build(new_skeleton)


func _build(skeleton: Skeleton3D) -> void:
	_bone_count = skeleton.get_bone_count()
	_entries.clear()
	if not skeleton.has_meta(DATA_META):
		return
	var data: Variant = skeleton.get_meta(DATA_META)
	if not data is Array:
		return
	for entry in data:
		if entry is Dictionary and int(entry["append"]) >= 0:
			_entries.append(entry)
	_entries.sort_custom(sort_entries)


## Returns true when every [AnimationPlayer] driving [param skeleton] plays a
## motion that already contains the 付与 result (see [method MMDPoseBaker.bake]).
## The rule must not run again in that case.
func _plays_baked_animation(skeleton: Skeleton3D) -> bool:
	var driving := false
	for player in _driving_players(skeleton):
		driving = true
		if not _player_plays_baked(player):
			return false
	return driving


## Collects the players that can modify [param skeleton]: the node itself, an
## ancestor or a sibling of any ancestor. A retargeted scene usually keeps the
## [AnimationPlayer] next to the model, not below it.
func _driving_players(skeleton: Skeleton3D) -> Array:
	var players := []
	var seen := {}
	var node: Node = skeleton
	while node != null:
		_add_player(node, skeleton, players, seen)
		for child in node.get_children():
			_add_player(child, skeleton, players, seen)
		node = node.get_parent()
	return players


func _add_player(node: Node, skeleton: Skeleton3D, players: Array, seen: Dictionary) -> void:
	var player := node as AnimationPlayer
	if player == null or not player.is_playing():
		return
	var id := player.get_instance_id()
	if seen.has(id):
		return
	seen[id] = true
	var root := player.get_node_or_null(player.root_node)
	if root == null:
		return
	if root == skeleton or root.is_ancestor_of(skeleton):
		players.append(player)


func _player_plays_baked(player: AnimationPlayer) -> bool:
	var animation_name := String(player.current_animation)
	if animation_name.is_empty():
		return false
	var cache_key := "%d/%s" % [player.get_instance_id(), animation_name]
	if _baked_cache.has(cache_key):
		return _baked_cache[cache_key]
	var animation := _player_animation(player, animation_name)
	var baked := animation != null and animation.has_meta(BAKED_META) and bool(animation.get_meta(BAKED_META))
	_baked_cache[cache_key] = baked
	return baked


static func _player_animation(player: AnimationPlayer, name: String) -> Animation:
	var separator := name.find("/")
	var library_name := ""
	var animation_name := name
	if separator >= 0:
		library_name = name.substr(0, separator)
		animation_name = name.substr(separator + 1)
	if not player.has_animation_library(library_name):
		return null
	var library := player.get_animation_library(library_name)
	if library == null:
		return null
	return library.get_animation(animation_name)


static func sort_entries(a: Dictionary, b: Dictionary) -> bool:
	var order_a: int = a["order"]
	var order_b: int = b["order"]
	if order_a != order_b:
		return order_a < order_b
	return int(a["bone"]) < int(b["bone"])


# --------------------------------------------------------------------------
# 付与 (append / grant)
# --------------------------------------------------------------------------

## Applies the append rule of one bone. Also used by [MMDPoseBaker] to recreate
## the runtime pose while solving IK at import time.
static func apply_append(skeleton: Skeleton3D, entry: Dictionary) -> void:
	var bone: int = entry["bone"]
	var target: int = entry["append"]
	if target < 0 or target == bone or target >= skeleton.get_bone_count() or bone >= skeleton.get_bone_count():
		return
	var weight: float = entry["weight"]
	var rest_target := skeleton.get_bone_rest(target)

	if entry["rot"]:
		var rest_rotation := rest_target.basis.get_rotation_quaternion()
		var target_rotation := skeleton.get_bone_pose_rotation(target)
		var delta := _quaternion_power(rest_rotation.inverse() * target_rotation, weight)
		if not delta.is_equal_approx(Quaternion.IDENTITY):
			var rotation := skeleton.get_bone_pose_rotation(bone) * delta
			skeleton.set_bone_pose_rotation(bone, rotation.normalized())

	if entry["pos"]:
		var delta_position := skeleton.get_bone_pose_position(target) - rest_target.origin
		if not delta_position.is_zero_approx():
			var position := skeleton.get_bone_pose_position(bone) + delta_position * weight
			skeleton.set_bone_pose_position(bone, position)


## Returns [param q] raised to the power of [param weight]: the rotation angle is
## scaled, so a negative weight produces the inverse rotation.
static func _quaternion_power(q: Quaternion, weight: float) -> Quaternion:
	if is_zero_approx(weight):
		return Quaternion.IDENTITY
	var angle := q.get_angle()
	if is_zero_approx(angle):
		return Quaternion.IDENTITY
	# get_axis() of a nearly identity quaternion can be very short; the
	# Quaternion(axis, angle) constructor requires a normalized axis.
	var axis := q.get_axis()
	if axis.length_squared() < 1e-12:
		return Quaternion.IDENTITY
	return Quaternion(axis.normalized(), angle * weight)
