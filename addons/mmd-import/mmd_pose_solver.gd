@tool
class_name MMDPoseSolver extends SkeletonModifier3D

## Applies the MMD pose rules that plain [Animation] tracks cannot express:
## 付与 (append / grant) and IK.
##
## [b]付与[/b]: a bone inherits a weighted part of another bone's animation
## transform (twist bones, D bones, 肩C, 目戻...). Negative weights invert the
## inherited rotation. Bones are processed in the PMX transform order.
##
## [b]IK[/b]: the chains of the PMX file (足ＩＫ -> ひざ -> 足 -> 足首) are solved
## on the posed skeleton every frame by [MMDIKSolver], the way MMD does it. The
## result is not baked into the motion, so the animation stays blendable: an
## [AnimationMixer] can cross fade, an [AnimationTree] can blend and seek, and
## the motion file keeps its original keys.
##
## The IK on/off states of the motion (the property keys of the VMD file) are
## carried by the imported [Animation] as the "mmd_ik_toggles" metadata
## (see [constant IK_TOGGLES_META]) and evaluated at the playback time of the
## [AnimationPlayer] that drives the skeleton. Motions played through an
## [AnimationTree] have no readable time, so the chains stay enabled there.
##
## This node has to run after the [AnimationMixer] and before
## [MMDPhysicsSimulator], which is why [PMXImporter] adds it first: the physics
## places the rigid bodies on the final (solved) pose. The engine restores the
## unmodified poses after the modifiers ran, so the skin uses the solved pose
## while the animation keeps working on a clean pose every frame.
##
## A motion that already contains the rules (see [MMDPoseBaker], used by the
## "MMD: 烘焙动画库" editor tool) is recognised by its [constant BAKED_META]
## marker and skipped here, otherwise the inherited transform would be applied
## twice.
##
## The bone data is written by [PMXImporter] into the [Skeleton3D] metadata
## "mmd_bone_data".

const DATA_META := "mmd_bone_data"
## Mirrors [constant MMDPoseBaker.BAKED_META]; kept local so the solver does not
## depend on the baker class being registered in exported projects. Marks a
## motion that already contains the 付与 result (and, for motions baked by older
## versions of the importer, the IK result as well).
const BAKED_META := "mmd_append_baked"
## Marks a motion that already contains the IK result.
const BAKED_IK_META := "mmd_ik_baked"
## Metadata of an imported [Animation]: controller bone name -> array of
## [code][time in seconds, enabled][/code]. Written by [VMDImporter].
const IK_TOGGLES_META := "mmd_ik_toggles"

## Applies the 付与 rule. Turn it off to inspect the raw motion of a file.
@export var solve_append := true
## Solves the IK chains. Turn it off for motions authored for another rig or to
## inspect the keys of a file before the chains correct them.
@export var solve_ik := true

var _entries: Array = []
## Controller bone name of every entry, "" when the entry has no IK chain. Kept
## in the same order as [member _entries].
var _ik_names: Array[String] = []
## Controller bone name -> IK enabled, resolved once per frame.
var _ik_states := {}
var _bone_count := -1
## "instance id/animation name" -> { "append": bool, "ik": bool },
## see [method _baked_rules].
var _baked_cache := {}
## "instance id/animation name" -> sorted IK toggle data.
var _toggle_cache := {}


func _process_modification() -> void:
	var skeleton := get_skeleton()
	if skeleton == null:
		return
	if _bone_count != skeleton.get_bone_count():
		_build(skeleton)
	if _entries.is_empty():
		return
	var baked := _baked_rules(skeleton)
	var do_append := solve_append and not bool(baked["append"])
	var do_ik := solve_ik and not bool(baked["ik"])
	if not do_append and not do_ik:
		return
	if do_ik:
		_refresh_ik_states(skeleton)

	# Same order as MMD: the 付与 rule of a bone runs first, then its IK chain.
	# The cache of skeleton space poses has to be dropped after every change,
	# because the engine only refreshes its global poses after all modifiers ran.
	var global_cache := {}
	for index in range(_entries.size()):
		var entry: Dictionary = _entries[index]
		if do_append and int(entry["append"]) >= 0:
			apply_append(skeleton, entry)
			global_cache.clear()
		if do_ik and not _ik_names[index].is_empty():
			if bool(_ik_states.get(_ik_names[index], true)):
				MMDIKSolver.solve(skeleton, entry, global_cache)


func _skeleton_changed(_old_skeleton: Skeleton3D, new_skeleton: Skeleton3D) -> void:
	_bone_count = -1
	_entries.clear()
	_ik_names.clear()
	_ik_states.clear()
	_baked_cache.clear()
	_toggle_cache.clear()
	if new_skeleton != null:
		_build(new_skeleton)


func _build(skeleton: Skeleton3D) -> void:
	_bone_count = skeleton.get_bone_count()
	_entries.clear()
	_ik_names.clear()
	_ik_states.clear()
	_toggle_cache.clear()
	if not skeleton.has_meta(DATA_META):
		return
	var data: Variant = skeleton.get_meta(DATA_META)
	if not data is Array:
		return
	# Every bone with a rule, ordered like the PMX deform order so parents (and
	# the bones a chain depends on) are handled before their children.
	for entry in data:
		if not entry is Dictionary:
			continue
		if int(entry["append"]) >= 0 or int(entry["ik_target"]) >= 0:
			_entries.append(entry)
	_entries.sort_custom(sort_entries)
	for entry in _entries:
		var name := ""
		var controller: int = int(entry["bone"])
		if int(entry["ik_target"]) >= 0 and controller >= 0 and controller < _bone_count:
			name = skeleton.get_bone_name(controller)
			_ik_states[name] = true
		_ik_names.append(name)


## Resolves the IK on/off state of every chain from the motion that plays.
func _refresh_ik_states(skeleton: Skeleton3D) -> void:
	var playback := _toggle_playback(skeleton)
	if playback.is_empty():
		return
	var data: Dictionary = playback["data"]
	var time: float = playback["time"]
	for controller in data:
		if _ik_states.has(controller):
			_ik_states[controller] = _state_at(data[controller], time)


## Returns { "data": sorted toggles, "time": playback position } of the motion
## that drives [param skeleton], or an empty dictionary when it carries none.
func _toggle_playback(skeleton: Skeleton3D) -> Dictionary:
	for player in _driving_players(skeleton):
		var animation_name := String(player.current_animation)
		if animation_name.is_empty():
			continue
		var animation := _player_animation(player, animation_name)
		if animation == null or not animation.has_meta(IK_TOGGLES_META):
			continue
		# The key holds the resource, not only its name: a reimported motion is a
		# new [Animation] with new switches, while the name stays the same.
		var cache_key := "%d/%d" % [player.get_instance_id(), animation.get_instance_id()]
		if not _toggle_cache.has(cache_key):
			_toggle_cache[cache_key] = _sort_toggles(animation.get_meta(IK_TOGGLES_META))
		return {"data": _toggle_cache[cache_key], "time": player.current_animation_position}
	return {}


## Converts the "mmd_ik_toggles" metadata into time sorted arrays, so the state
## of a frame can be looked up without walking the keys.
static func _sort_toggles(raw: Variant) -> Dictionary:
	var out := {}
	if not raw is Dictionary:
		return out
	for controller in raw:
		var pairs: Array = []
		for key in raw[controller]:
			if key is Array and key.size() >= 2:
				pairs.append([float(key[0]), bool(key[1])])
		pairs.sort_custom(_sort_toggle_pairs)
		var times := PackedFloat64Array()
		var states := PackedByteArray()
		for pair in pairs:
			times.append(pair[0])
			states.append(1 if pair[1] else 0)
		out[String(controller)] = {"times": times, "states": states}
	return out


static func _sort_toggle_pairs(a: Array, b: Array) -> bool:
	return float(a[0]) < float(b[0])


## Returns the state a chain has at [param time]: the last key at or before it,
## enabled when the motion does not say anything yet.
static func _state_at(state: Dictionary, time: float) -> bool:
	var times: PackedFloat64Array = state["times"]
	if times.is_empty() or time < times[0]:
		return true
	var states: PackedByteArray = state["states"]
	var low := 0
	var high := times.size() - 1
	while low < high:
		var middle := (low + high + 1) / 2
		if times[middle] <= time:
			low = middle
		else:
			high = middle - 1
	return states[low] != 0


## Returns which rule sets are already contained in the motions that
## [param skeleton] plays: { "append": bool, "ik": bool }. A rule is only skipped
## when every driving player plays a motion that contains it (see
## [method MMDPoseBaker.bake]); otherwise the rule would be applied twice.
func _baked_rules(skeleton: Skeleton3D) -> Dictionary:
	var append_baked := true
	var ik_baked := true
	var driving := false
	for player in _driving_players(skeleton):
		driving = true
		var state := _player_baked_state(player)
		if not bool(state["append"]):
			append_baked = false
		if not bool(state["ik"]):
			ik_baked = false
	if not driving:
		return {"append": false, "ik": false}
	return {"append": append_baked, "ik": ik_baked}


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


## Returns { "append": bool, "ik": bool } for the motion [param player] plays.
func _player_baked_state(player: AnimationPlayer) -> Dictionary:
	var animation_name := String(player.current_animation)
	if animation_name.is_empty():
		return {"append": false, "ik": false}
	var cache_key := "%d/%s" % [player.get_instance_id(), animation_name]
	if _baked_cache.has(cache_key):
		return _baked_cache[cache_key]
	var animation := _player_animation(player, animation_name)
	var state := {"append": false, "ik": false}
	if animation != null:
		# The legacy marker was set by motions that contain both rules.
		var legacy := _meta_true(animation, BAKED_META)
		state = {
			"append": legacy,
			"ik": legacy or _meta_true(animation, BAKED_IK_META),
		}
	_baked_cache[cache_key] = state
	return state


static func _meta_true(resource: Resource, key: String) -> bool:
	return resource.has_meta(key) and bool(resource.get_meta(key))


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
