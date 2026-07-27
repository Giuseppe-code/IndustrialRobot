extends Node3D

@export var package_size := Vector3(0.5, 0.5, 0.5)
@export var package_position := Vector2(0.0, 0.0)
@export var top_clearance := 0.005
@export var package_collision_layer := 8
@export var package_collision_mask := 1
@export_range(0, 2) var forced_color_code := 0

const BLUE_CODE := 1
const RED_CODE := 2
const PACKAGE_COLORS := {
	BLUE_CODE: Color(0.04, 0.20, 0.85, 1.0),
	RED_CODE: Color(0.987, 0.143, 0.0, 1.0),
}

var rng := RandomNumberGenerator.new()
var package_body: StaticBody3D
var package_picked := false

func _ready() -> void:
	rng.randomize()
	DDS.subscribe("PayloadMass")
	_spawn_package()

func _process(_delta: float) -> void:
	if package_body == null:
		return
	var payload_mass = DDS.read("PayloadMass")
	var loaded := payload_mass != null and float(payload_mass) > 0.01
	if loaded:
		package_picked = true
	package_body.visible = not package_picked
	package_body.collision_layer = 0 if package_picked else package_collision_layer

func _spawn_package() -> void:
	var center_y := package_size.y * 0.5 + top_clearance
	package_body = StaticBody3D.new()
	package_body.name = "Package"
	package_body.collision_layer = package_collision_layer
	package_body.collision_mask = package_collision_mask
	package_body.position = Vector3(package_position.x, center_y, package_position.y)

	var mesh_instance := MeshInstance3D.new()
	var box_mesh := BoxMesh.new()
	box_mesh.size = package_size
	box_mesh.material = _package_material()
	mesh_instance.mesh = box_mesh
	package_body.add_child(mesh_instance)

	var collision := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = package_size
	collision.shape = shape
	package_body.add_child(collision)

	add_child(package_body)

func _package_material() -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = PACKAGE_COLORS[_selected_color_code()]
	material.roughness = 0.65
	return material

func _selected_color_code() -> int:
	# L'override d'ambiente rende ripetibili i test senza alterare la scena.
	if forced_color_code == BLUE_CODE or forced_color_code == RED_CODE:
		return forced_color_code
	return rng.randi_range(BLUE_CODE, RED_CODE)
