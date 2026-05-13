# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import json
import math
import os
import xml.dom.minidom as minidom
import xml.etree.ElementTree as ET
from collections import defaultdict
from typing import Any, Dict, List, Tuple

import bpy
from mathutils import Matrix, Quaternion, Vector

# Colors from https://colorbrewer2.org/
colors = [
    (166, 206, 227),
    (31, 120, 180),
    (178, 223, 138),
    (51, 160, 44),
    (251, 154, 153),
    (227, 26, 28),
    (253, 191, 111),
    (255, 127, 0),
    (202, 178, 214),
    (106, 61, 154),
    (255, 255, 153),
    (177, 89, 40),
]
colors = [(c[0] / 255, c[1] / 255, c[2] / 255) for c in colors]
next_color = 0

# state vars
armature = None
counter = 0
links = []
joints = []
bones_to_meshes: Dict[Any, List[Any]] = defaultdict(
    list
)  # maps bones to lists of mesh objects

# constants
LINK_NAME_FORMAT = "{bone_name}"
JOINT_NAME_FORMAT = "{bone_name}"
ORIGIN_NODE_FLOAT_PRECISION = 6
ORIGIN_NODE_FORMAT = "{{:.{0}f}} {{:.{0}f}} {{:.{0}f}}".format(
    ORIGIN_NODE_FLOAT_PRECISION
)
ZERO_ORIGIN_NODE = lambda: ET.fromstring('<origin xyz="0 0 0" rpy="0 0 0"/>')
INERTIA_NODE_FMT = '<inertia ixx="{}" ixy="{}" ixz="{}" iyy="{}" iyz="{}" izz="{}" />'
AXIS_NODE_FORMAT = lambda: ET.fromstring('<axis xyz="0 0 0" />')
BASE_LIMIT_NODE_STR = None


def round_scales(keyword: str = "collision"):
    """
    Rounds shape scale vectors to 4 decimal points (millimeter) accuracy. E.g. to eliminate small mismatches in collision shape scale.
    Use 'keyword' to discriminate between shape types.
    """
    for obj in bpy.context.scene.objects:
        if keyword in obj.name:
            for i in range(3):  # Iterate over X, Y, Z
                obj.scale[i] = round(obj.scale[i], 4)


def fix_scales(keyword: str = "collision"):
    """
    Flips negative scales for shapes. (E.g. collision shapes should not have negative scaling)
    Use 'keyword' to discriminate between shape types.
    """
    for obj in bpy.context.scene.objects:
        if keyword in obj.name:
            for i in range(3):  # Iterate over X, Y, Z
                obj.scale[i] = abs(obj.scale[i])


def set_base_limit_str(effort, velocity):
    """
    Default effort and velocity limits for Joints.
    """
    global BASE_LIMIT_NODE_STR
    BASE_LIMIT_NODE_STR = '<limit effort="{:.4f}" lower="-1.57079632679" upper="1.57079632679" velocity="{:.4f}"/>'.format(
        effort, velocity
    )


def deselect_all() -> None:
    """
    Deselect all objects.
    """
    for obj in bpy.context.selected_objects:
        obj.select_set(False)


def is_mesh(obj) -> bool:
    """
    Is the object a MESH?
    """
    return obj.type == "MESH"


def is_collision_mesh(mesh_obj) -> bool:
    """
    Is the object a collision mesh.
    """
    return "collision" in mesh_obj.name


def is_receptacle_mesh(mesh_obj) -> bool:
    """
    Is the object a receptacle mesh.
    """
    return "receptacle" in mesh_obj.name


def get_mesh_heirarchy(
    mesh_obj,
    select_set: bool = True,
    include_render: bool = True,
    include_collison: bool = False,
    include_receptacle: bool = False,
) -> List[Any]:
    """
    Select all MESH objects in the heirarchy specifically targeting or omitting meshes with "collision" in the name.

    :param mesh_obj: The Blender mesh object.
    :param select_set: Whether or not to select the objects as well as recording them.
    :param include_render: Include render objects without qualifiers in the name.
    :param include_collison: Include objects with 'collision' in the name.
    :param include_receptacle: Include objects with 'receptacle' in the name.

    :return: The list of Blender mesh objects.
    """
    selected_objects = []
    is_col_mesh = is_collision_mesh(mesh_obj)
    is_rec_mesh = is_receptacle_mesh(mesh_obj)
    if is_mesh(mesh_obj) and (
        (is_col_mesh and include_collison)
        or (is_rec_mesh and include_receptacle)
        or (include_render and not is_col_mesh and not is_rec_mesh)
    ):
        selected_objects.append(mesh_obj)
        if select_set:
            mesh_obj.select_set(True)
    for child in mesh_obj.children:
        if child.type != "ARMATURE":
            selected_objects.extend(
                get_mesh_heirarchy(
                    child,
                    select_set,
                    include_render,
                    include_collison,
                    include_receptacle,
                )
            )
    return selected_objects


def walk_armature(this_bone, handler, kwargs_for_handler=None):
    """
    Recursively apply a handler function to bone children to traverse the armature.
    """
    if kwargs_for_handler is None:
        kwargs_for_handler = {}
    handler(this_bone, **kwargs_for_handler)
    for child in this_bone.children:
        walk_armature(child, handler, kwargs_for_handler)


def bone_info(bone):
    """
    Print relevant bone info to console.
    """
    print(" bone info")
    print(f"    name: {bone.name}")
    print("    children")
    for child in bone.children:
        print(f"       - {child.name}")


def node_info(node):
    """
    Print relevant info about an object node.
    """
    print(" node info")
    print(f"    name: {node.name}")
    print("    children")
    for child in node.children:
        print(f"       - {child.name}")


def get_origin_from_matrix(M):
    """
    Construct a URDF 'origin' element from a Matrix by separating translation from rotation and converting to Euler.
    """
    translation = M.to_translation()
    euler = M.to_euler()
    origin_xml_node = ET.Element("origin")
    origin_xml_node.set("rpy", ORIGIN_NODE_FORMAT.format(euler.x, euler.y, euler.z))
    origin_xml_node.set(
        "xyz", ORIGIN_NODE_FORMAT.format(translation.x, translation.y, translation.z)
    )

    return origin_xml_node


def get_next_color() -> Tuple[int, int, int]:
    """
    Global function to get the next color in the colors list.
    """
    global next_color
    this_color = colors[next_color % len(colors)]
    next_color += 1
    return this_color


def add_color_material_to_visual(color, xml_visual) -> None:
    """
    Add a color material to a visual node.
    """
    this_xml_material = ET.Element("material")
    this_xml_material.set("name", "mat_col_{}".format(color))
    this_xml_color = ET.Element("color")
    this_xml_color.set("rgba", "{:.2f} {:.2f} {:.2f} 1.0".format(*color))
    this_xml_material.append(this_xml_color)
    xml_visual.append(this_xml_material)


def bone_to_urdf(
    this_bone,
    link_visuals=True,
    collision_visuals=False,
    joint_visuals=False,
    receptacle_visuals=False,
):
    """This function extracts the basic properties of the bone and populates
    links and joint lists with the corresponding urdf nodes"""

    print(f"link_visuals = {link_visuals}")
    print(f"collision_visuals = {collision_visuals}")
    print(f"joint_visuals = {joint_visuals}")
    print(f"receptacle_visuals = {receptacle_visuals}")

    global counter

    # Create the joint xml node
    if this_bone.parent:
        this_xml_link = create_bone_link(this_bone)
    else:
        this_xml_link = create_root_bone_link(this_bone)

    # NOTE: default intertia (assume overriden automatically in-engine)
    this_xml_link.append(ET.fromstring(INERTIA_NODE_FMT.format(1.0, 0, 0, 1.0, 0, 1.0)))

    # NOTE: default unit mass TODO: estimate somehow?
    this_xml_link.append(ET.fromstring('<mass value="{:.6f}"/>'.format(1.0)))

    # TODO: scrape the list of mesh filenames which would be generated by an export.
    collision_objects = []
    receptacle_objects = []
    for mesh_obj in bones_to_meshes[this_bone.name]:
        collision_objects.extend(
            get_mesh_heirarchy(
                mesh_obj,
                select_set=False,
                include_collison=True,
                include_render=False,
            )
        )
        if receptacle_visuals:
            receptacle_objects.extend(
                get_mesh_heirarchy(
                    mesh_obj,
                    select_set=False,
                    include_collison=False,
                    include_render=False,
                    include_receptacle=True,
                )
            )
        if mesh_obj.parent is not None and mesh_obj.parent.type == "ARMATURE":
            # this object is the mesh name for export, so use it
            # Create the visual node
            this_xml_visual = ET.Element("visual")
            this_xml_mesh_geom = ET.Element("geometry")
            this_xml_mesh = ET.Element("mesh")
            this_xml_mesh.set("filename", f"{mesh_obj.name}.glb")
            this_xml_mesh.set("scale", "1.0 1.0 1.0")
            this_xml_mesh_geom.append(this_xml_mesh)
            # NOTE: we can use zero because we reset the origin for the meshes before exporting them to glb
            this_xml_visual.append(ZERO_ORIGIN_NODE())
            this_xml_visual.append(this_xml_mesh_geom)
            if link_visuals:
                this_xml_link.append(this_xml_visual)

    # NOTE: visual debugging tool to add a box at the joint pivot locations
    if joint_visuals:
        this_xml_visual = ET.Element("visual")
        this_xml_test_geom = ET.Element("geometry")
        this_xml_box = ET.Element("box")
        this_xml_box.set("size", f"{0.1} {0.1} {0.1}")
        this_xml_test_geom.append(this_xml_box)
        this_xml_visual.append(ZERO_ORIGIN_NODE())
        this_xml_visual.append(this_xml_test_geom)
        this_xml_link.append(this_xml_visual)

    # NOTE: color each link's collision shapes for debugging
    this_color = get_next_color()

    supported_collision_shapes = [
        "collision_box",
        "collision_cylinder",
        "collision_sphere",
    ]

    for col in collision_objects:
        assert (
            len(
                [
                    col_name
                    for col_name in supported_collision_shapes
                    if col_name in col.name
                ]
            )
            == 1
        ), f"Only supporting exactly one of the following collision shapes currently: {supported_collision_shapes}. Shape name '{col.name}' unsupported."

        set_obj_origin_to_center(col)
        clear_obj_transform(
            col, apply=True, include_scale_apply=False, include_rot_apply=False
        )
        set_obj_origin_to_xyz(col, col.parent.matrix_world.translation)
        clear_obj_transform(col)
        set_obj_origin_to_center(col)

        # Create the collision node
        this_xml_collision = ET.Element("collision")
        if collision_visuals:
            this_xml_collision = ET.Element("visual")
            add_color_material_to_visual(this_color, this_xml_collision)
        this_xml_col_geom = ET.Element("geometry")
        xml_shape = None
        if "collision_box" in col.name:
            this_xml_box = ET.Element("box")
            box_size = col.scale
            this_xml_box.set("size", f"{box_size.x} {box_size.y} {box_size.z}")
            xml_shape = this_xml_box
        elif "collision_cylinder" in col.name:
            this_xml_cyl = ET.Element("cylinder")
            scale = col.scale
            # radius XY axis scale must match
            assert (
                abs(scale.x - scale.y) < 0.0001
            ), f"XY dimensions must match. Used as radius. node_name=='{col.name}', x={scale.x}, y={scale.y}"
            this_xml_cyl.set("radius", f"{scale.x/2.0}")
            # NOTE: assume Z axis is length of the cylinder
            this_xml_cyl.set("length", f"{scale.z}")
            xml_shape = this_xml_cyl
        elif "collision_sphere" in col.name:
            this_xml_sphere = ET.Element("sphere")
            scale = col.scale
            # radius XYZ axis scale must match
            assert (
                abs(scale.x - scale.y) < 0.0001 and abs(scale.x - scale.z) < 0.0001
            ), f"XYZ dimensions must match. Used as radius. node_name=='{col.name}', x={scale.x}, y={scale.y}, z={scale.z}"
            this_xml_sphere.set("radius", f"{scale.x/2.0}")
            xml_shape = this_xml_sphere

        this_xml_col_geom.append(xml_shape)
        # first get the rotation
        xml_origin = get_origin_from_matrix(col.matrix_local)
        # then get local translation
        col_link_position = col.location
        xml_origin.set(
            "xyz",
            ORIGIN_NODE_FORMAT.format(
                col_link_position.x, col_link_position.y, col_link_position.z
            ),
        )
        this_xml_collision.append(xml_origin)
        this_xml_collision.append(this_xml_col_geom)
        this_xml_link.append(this_xml_collision)

    if receptacle_visuals:
        for rec_mesh in receptacle_objects:
            # NOTE: color each link's collision shapes for debugging
            this_color = get_next_color()
            this_xml_visual = ET.Element("visual")
            this_xml_geom = ET.Element("geometry")
            this_xml_mesh = ET.Element("mesh")
            rec_filename = rec_mesh.parent.name + "_receptacle.glb"
            this_xml_mesh.set("filename", f"{rec_filename}")
            this_xml_mesh.set("scale", "1.0 1.0 1.0")
            this_xml_geom.append(this_xml_mesh)
            # NOTE: we can use zero because we reset the origin for the meshes before exporting them to glb
            this_xml_visual.append(ZERO_ORIGIN_NODE())
            this_xml_visual.append(this_xml_geom)
            add_color_material_to_visual(this_color, this_xml_visual)
            this_xml_link.append(this_xml_visual)

    if not this_bone.children:
        pass
        # We reached the end of the chain.

    counter += 1


def create_root_bone_link(this_bone):
    """
    Construct the root link element from a bone.
    Called for bones with no parent (i.e. the root node)
    """
    xml_link = ET.Element("link")
    xml_link_name = this_bone.name
    xml_link.set("name", xml_link_name)
    links.append(xml_link)

    this_bone.name = xml_link_name
    return xml_link


def get_origin_from_bone(bone):
    """
    Construct an origin element for a joint from a bone.
    """
    translation = (
        bone.matrix_local.to_translation() - bone.parent.matrix_local.to_translation()
    )

    origin_xml_node = ET.Element("origin")
    origin_xml_node.set("rpy", "0 0 0")
    origin_xml_node.set(
        "xyz", ORIGIN_NODE_FORMAT.format(translation.x, translation.y, translation.z)
    )

    return origin_xml_node


def create_bone_link(this_bone):
    """
    Construct Link and Joint elements from a bone.
    """
    global counter

    # construct limits and joint type from animation frames
    bone_limits = get_anim_limits_info(this_bone)

    # Get bone properties
    parent_bone = this_bone.parent
    base_joint_name = JOINT_NAME_FORMAT.format(
        counter=counter, bone_name=this_bone.name
    )

    # ------------- Create joint--------------

    joint = ET.Element("joint")
    joint.set("name", base_joint_name)

    # create origin node
    origin_xml_node = get_origin_from_bone(this_bone)

    # create parent node
    parent_xml_node = ET.Element("parent")
    parent_xml_node.set("link", parent_bone.name)

    xml_link = ET.Element("link")
    xml_link_name = this_bone.name
    xml_link.set("name", xml_link_name)
    links.append(xml_link)

    # create child node
    child_xml_node = ET.Element("child")
    child_xml_node.set("link", xml_link_name)

    joint.append(parent_xml_node)
    joint.append(child_xml_node)

    # create limits node
    limit_node = ET.fromstring(BASE_LIMIT_NODE_STR)

    local_axis = Vector()

    # Revolute
    if len(bone_limits["lower_limit"]) == 4:
        joint.set("type", "revolute")
        begin = Quaternion(bone_limits["lower_limit"])
        end = Quaternion(bone_limits["upper_limit"])
        rest = Quaternion()
        diff = begin.rotation_difference(end)
        local_axis, angle = diff.to_axis_angle()
        rest_diff = begin.rotation_difference(rest)
        rest_axis, rest_angle = rest_diff.to_axis_angle()
        limit_node.set("lower", f"{-rest_angle}")
        limit_node.set("upper", f"{angle-rest_angle}")

    # Prismatic
    if len(bone_limits["lower_limit"]) == 3:
        joint.set("type", "prismatic")
        upper_vec = Vector(bone_limits["upper_limit"])
        lower_vec = Vector(bone_limits["lower_limit"])
        displacement = upper_vec - lower_vec
        local_axis = displacement.normalized()
        limit_node.set("lower", f"{-lower_vec.length}")
        limit_node.set("upper", f"{upper_vec.length}")

    # NOTE: rest pose could be applied to the bone resulting in an additional rotation stored in the matrix property
    rest_correction = this_bone.matrix
    # NOTE: Blender bones and armature are always Y-up, so we need to rotate the axis into URDF coordinate space (Z-up)
    bone_axis = this_bone.vector
    to_z_up = bone_axis.rotation_difference(Vector([0, 0, 1]))
    # apply all rotations to arrive at the URDF Joint axis
    axis = rest_correction @ (to_z_up @ local_axis)

    xml_axis = AXIS_NODE_FORMAT()
    xml_axis.set("xyz", ORIGIN_NODE_FORMAT.format(axis.x, axis.y, axis.z))

    joint.append(xml_axis)
    joint.append(limit_node)
    joint.append(origin_xml_node)
    joints.append(joint)
    ret_link = xml_link

    return ret_link


# ==========================================


def set_obj_origin_to_center(obj) -> None:
    """
    Set object origin to it's own center.
    """
    deselect_all()
    obj.select_set(True)
    bpy.ops.object.origin_set(type="ORIGIN_GEOMETRY", center="MEDIAN")


def set_obj_origin_to_xyz(obj, xyz) -> None:
    """
    Set object origin to a global xyz location.
    """
    deselect_all()
    bpy.context.scene.cursor.location = xyz
    obj.select_set(True)
    bpy.ops.object.origin_set(type="ORIGIN_CURSOR", center="MEDIAN")


def set_obj_origin_to_bone(obj, bone):
    """
    Set the object origin to the bone transformation.
    """
    set_obj_origin_to_xyz(obj, bone.matrix_local.translation)


def clear_obj_transform(
    arm, apply=False, include_scale_apply=True, include_rot_apply=True
):
    """
    Clear the armature transform to align it with the origin.
    """
    deselect_all()
    arm.select_set(True)
    if apply:
        bpy.ops.object.transform_apply(
            location=True, rotation=include_rot_apply, scale=include_scale_apply
        )
    else:
        bpy.ops.object.location_clear(clear_delta=False)


def get_anim_limits_info(bone):
    """
    Get limits info from animation action tracks.
    """
    bone_limits = {"rest_pose": [], "lower_limit": [], "upper_limit": []}
    if "root" in bone.name:
        # no joint data defined for the root
        return bone_limits
    is_prismatic = False
    is_revolute = False

    for ac in bpy.data.actions:
        if bone.name in ac.name:
            key_match = [key for key in bone_limits if key in ac.name][0]
            limit_list = []
            for _fkey, fcurve in ac.fcurves.items():
                assert (
                    len(fcurve.keyframe_points) == 1
                ), "Expecting one keyframe per track."
                index = fcurve.array_index
                value = fcurve.keyframe_points[0].co[1]
                if "quaternion" in fcurve.data_path:
                    if len(limit_list) == 0:
                        limit_list = [0, 0, 0, 0]
                    is_revolute = True
                if "location" in fcurve.data_path:
                    if len(limit_list) == 0:
                        limit_list = [0, 0, 0]
                    is_prismatic = True
                try:
                    limit_list[index] = value
                except IndexError:
                    raise Exception(
                        f"Failed to get limits for fcurve: bone={bone.name}, curve_key={_fkey}, index={index}. Should have exactly 3 (position) or exactly 4 (quaternion) elements."
                    )

            bone_limits[key_match] = limit_list
    assert (
        is_prismatic or is_revolute
    ), f"Bone {bone.name} does not have animation data."
    assert not (
        is_prismatic and is_revolute
    ), f"Bone {bone.name} has both rotation and translation defined."
    return bone_limits


def get_parent_bone(obj):
    """
    Climb the node tree looking for the parent bone of an object.
    Return the parent bone or None if a parent bone does not exist.
    """
    if obj.parent_bone != "":
        return armature.data.bones[obj.parent_bone]
    if obj.parent is None:
        return None
    return get_parent_bone(obj.parent)


def get_root_bone():
    """
    Find the root bone.
    """
    root_bone = None
    for b in armature.data.bones:
        if not b.parent:
            assert root_bone is None, "More than one root bone found."
            root_bone = b
    return root_bone


def get_armature():
    """
    Search the objects for an armature object.
    """
    for obj in bpy.data.objects:
        if obj.type == "ARMATURE":
            return obj
    return None


def construct_root_rotation_joint(root_node_name):
    """
    Construct the root rotation joint XML Element.
    """
    xml_root_joint = ET.Element("joint")
    xml_root_joint.set("name", "root_rotation")
    xml_root_joint.set("type", "fixed")

    # construct a standard rotation matrix transform to apply to all root nodes
    M = Matrix.Rotation(math.radians(-90.0), 4, "X")
    xml_root_joint.append(get_origin_from_matrix(M))

    # create parent node
    parent_xml_node = ET.Element("parent")
    parent_xml_node.set("link", "root")

    # create child node
    child_xml_node = ET.Element("child")
    child_xml_node.set("link", root_node_name)

    xml_root_joint.append(parent_xml_node)
    xml_root_joint.append(child_xml_node)
    return xml_root_joint


def export(
    dirpath,
    settings,
    export_urdf: bool = True,
    export_meshes: bool = True,
    export_ao_config: bool = True,
    fix_materials: bool = True,
    **kwargs,
):
    """
    Run the Armature to URDF converter and export the .urdf file.
    Recursively travserses the armature bone tree and constructs Links and Joints.
    Note: This process is destructive and requires undo or revert in the editor after use.

    :return: export directory or URDF filepath
    """

    output_path = dirpath

    global LINK_NAME_FORMAT, JOINT_NAME_FORMAT, armature, root_bone, links, joints, counter
    counter = 0
    links = []
    joints = []

    # fixes a gltf export error caused by 1.0 ior values
    if fix_materials:
        for material in bpy.data.materials:
            if material.node_tree is not None:
                for node in material.node_tree.nodes:
                    if (
                        node.type == "BSDF_PRINCIPLED"
                        and "IOR" in node.inputs
                        and node.inputs["IOR"].default_value == 1.000
                    ):
                        node.inputs["IOR"].default_value = 0.000
                        print(f"Changed IOR value for material '{material.name}'")

        bpy.context.view_layer.update()

    # check poll() to avoid exception.
    if bpy.ops.object.mode_set.poll():
        bpy.ops.object.mode_set(mode="OBJECT")

    # get the armature
    armature = settings.get("armature")
    if armature is None:
        armature = bpy.data.objects["Armature"]

    # find the root bone
    root_bone = get_root_bone()

    if "link_name_format" in settings:
        LINK_NAME_FORMAT = settings["link_name_format"]

    if "joint_name_format" in settings:
        JOINT_NAME_FORMAT = settings["joint_name_format"]

    if "round_collision_scales" in settings and settings["round_collision_scales"]:
        round_scales()

    if "fix_collision_scales" in settings and settings["fix_collision_scales"]:
        fix_scales()

    # set the defaults to 100 T units and 3 units/sec (meters or radians)
    effort, velocity = (100, 3)
    if "def_limit_effort" in settings:
        effort = settings["def_limit_effort"]
    if "def_limit_vel" in settings:
        velocity = settings["def_limit_vel"]
    set_base_limit_str(effort, velocity)

    # clear the armature transform to remove unwanted transformations for later
    clear_obj_transform(armature, apply=True)

    # print all mesh object parents, reset origins for mesh export and transformation registery, collect bone to mesh map
    root_node = None
    receptacle_meshes = []
    receptacle_to_link_name = {}
    for obj in bpy.data.objects:
        if obj.type == "MESH":
            parent_bone = get_parent_bone(obj)
            set_obj_origin_to_bone(obj, parent_bone)
            print(f"MESH: {obj.name}")
            if obj.parent is not None:
                print(f" -p> {obj.parent.name}")
            if obj.parent_bone != "":
                bones_to_meshes[obj.parent_bone].append(obj)
                print(f" -pb> {obj.parent_bone}")
            if is_receptacle_mesh(obj):
                receptacle_meshes.append(obj)
                receptacle_to_link_name[obj.name] = obj.parent.name
        elif obj.type == "EMPTY":
            print(f"EMPTY: {obj.name}")
            if obj.parent is None and len(obj.children) > 0:
                print(" --IS ROOT")
                root_node = obj

    # make export directory for the object
    assert root_node is not None, "No root node, aborting."
    final_out_path = os.path.join(dirpath, f"{root_node.name}")
    os.makedirs(final_out_path, exist_ok=True)
    print(f"Output path : {final_out_path}")

    # export mesh components
    if export_meshes:
        for mesh_list in bones_to_meshes.values():
            for mesh_obj in mesh_list:
                if mesh_obj.parent is not None and mesh_obj.parent.type == "ARMATURE":
                    clear_obj_transform(mesh_obj)
                    deselect_all()
                    get_mesh_heirarchy(mesh_obj)
                    bpy.ops.export_scene.gltf(
                        filepath=os.path.join(final_out_path, mesh_obj.name),
                        use_selection=True,
                        export_yup=False,
                    )
        # export receptacle meshes
        for rec_mesh in receptacle_meshes:
            clear_obj_transform(rec_mesh.parent)
            deselect_all()
            rec_mesh.select_set(True)
            bpy.ops.export_scene.gltf(
                filepath=os.path.join(final_out_path, rec_mesh.name),
                use_selection=True,
                export_yup=False,
            )

    # print("------------------------")
    # print("Bone info recursion:")
    # walk_armature(root_bone, bone_info)
    # print("------------------------")
    # print("Node info recursion:")
    # walk_armature(root_node, node_info)
    # print("------------------------")
    if export_urdf:
        # Recursively generate the xml elements
        walk_armature(root_bone, bone_to_urdf, kwargs_for_handler=kwargs)

        # add all the joints and links to the root
        root_xml = ET.Element("robot")  # create <robot name="test_robot">
        root_xml.set("name", armature.name)

        # add a coordinate change in a dummy root node
        xml_root_link = ET.Element("link")
        xml_root_link.set("name", "root")
        xml_root_joint = construct_root_rotation_joint(root_bone.name)
        root_xml.append(xml_root_link)
        root_xml.append(xml_root_joint)

        root_xml.append(ET.Comment("LINKS"))
        for l in links:
            root_xml.append(l)

        root_xml.append(ET.Comment("JOINTS"))
        for j in joints:
            root_xml.append(j)

        # dump the xml string
        ET_raw_string = ET.tostring(root_xml, encoding="unicode")
        dom = minidom.parseString(ET_raw_string)
        ET_pretty_string = dom.toprettyxml()

        output_path = os.path.join(final_out_path, f"{root_node.name}.urdf")

        print(f"URDF output path : {output_path}")
        with open(output_path, "w") as f:
            f.write(ET_pretty_string)

    if export_ao_config:
        # write the ao_config
        ao_config_contents = {
            "urdf_filepath": f"{root_node.name}.urdf",
            "user_defined": {
                # insert receptacle metadata here
            },
        }
        for rec_name, link_name in receptacle_to_link_name.items():
            rec_label = "receptacle_mesh_" + rec_name
            ao_config_contents["user_defined"][rec_label] = {
                "name": rec_name,
                "parent_object": f"{root_node.name}",
                "parent_link": link_name,
                "position": [0, 0, 0],
                "rotation": [1, 0, 0, 0],
                "scale": [1, 1, 1],
                "up": [0, 0, 1],
                "mesh_filepath": rec_name + ".glb",
            }
        ao_config_filename = os.path.join(
            final_out_path, f"{root_node.name}.ao_config.json"
        )

        print(f"ao config output path : {ao_config_filename}")
        with open(ao_config_filename, "w") as f:
            json.dump(ao_config_contents, f)

    return output_path


if __name__ == "__main__":
    # NOTE: this must be run from within Blender and by default saves files in "blender_armatures/" relative to the directory containing the script

    export_path = None
    try:
        os.path.join(
            os.path.dirname(bpy.context.space_data.text.filepath), "blender_armatures"
        )
    except BaseException:
        print(
            "Couldn't get the directory from the filepath. E.g. running from commandline."
        )

    # -----------------------------------------------------------
    # To use this script in Blender editor:
    # 1. run with export meshes True
    # 2. undo changes in the editor
    # 3. run with export meshes False
    # 4. undo changes in the editor

    # NOTE: the following settings are overridden by commandline arguments if provided

    # Optionally override the save directory with an absolute path of your choice
    # export_path = "/home/my_path_choice/"

    export_urdf = False
    export_meshes = False
    export_ao_config = False
    round_collision_scales = False
    fix_collision_scales = False
    fix_materials = False

    # visual shape export flags for debugging
    link_visuals = True
    collision_visuals = False
    joint_visuals = False
    receptacle_visuals = False
    # -----------------------------------------------------------

    # -----------------------------------------------------------
    # To use from the commandline:
    #  1. `blender <path to asset>.blend --background --python <path to>blender_armature_to_urdf.py -- --export-path <export directory>
    #  2. add `--export-meshes` to export the link .glbs
    # Note: ' -- ' tells Blender to ignore the remaining arguments, so we pass anything after that into the script arguments below:
    import sys

    argv = sys.argv
    py_argv = ""
    if "--" in argv:
        py_argv = argv[argv.index("--") + 1 :]  # get all args after "--"

    import argparse

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--export-path",
        default=export_path,
        type=str,
        help="Path to the output directory for meshes and URDF.",
    )
    parser.add_argument(
        "--export-meshes",
        action="store_true",
        default=export_meshes,
        help="Export meshes for the link objects. If not set, instead generate the URDF.",
    )
    parser.add_argument(
        "--export-ao-config",
        action="store_true",
        default=export_ao_config,
        help="Export a *.ao_config.json file for the URDF.",
    )
    parser.add_argument(
        "--export-urdf",
        action="store_true",
        default=export_urdf,
        help="Export the *.urdf file.",
    )
    # Debugging flags:
    parser.add_argument(
        "--no-link-visuals",
        action="store_true",
        default=not link_visuals,
        help="Don't include visual mesh shapes in the exported URDF. E.g. for debugging.",
    )
    parser.add_argument(
        "--collision-visuals",
        action="store_true",
        default=collision_visuals,
        help="Include visual shapes for collision primitives in the exported URDF. E.g. for debugging.",
    )
    parser.add_argument(
        "--joint-visuals",
        action="store_true",
        default=joint_visuals,
        help="Include visual box shapes for joint pivot locations in the exported URDF. E.g. for debugging.",
    )
    parser.add_argument(
        "--receptacle-visuals",
        action="store_true",
        default=receptacle_visuals,
        help="Include visual mesh shapes for receptacles in the exported URDF. E.g. for debugging.",
    )
    parser.add_argument(
        "--round-collision-scales",
        action="store_true",
        default=round_collision_scales,
        help="Round all scale elements for collision shapes to 4 decimal points (millimeter accuracy).",
    )
    parser.add_argument(
        "--fix-collision-scales",
        action="store_true",
        default=fix_collision_scales,
        help="Flip all negative scale elements for collision shapes.",
    )
    parser.add_argument(
        "--fix-materials",
        action="store_true",
        default=fix_materials,
        help="Fixes materials with ior==1.0 which cause glTF export failure.",
    )

    args = parser.parse_args(py_argv)
    export_urdf = args.export_urdf
    export_meshes = args.export_meshes
    export_ao_config = args.export_ao_config
    export_path = args.export_path

    print(
        f"export_urdf : {export_urdf} | export_meshes : {export_meshes} | export_ao_config : {export_ao_config} | export_path : {export_path}"
    )

    # -----------------------------------------------------------

    assert (
        export_path is not None
    ), "No export path provided. If running from commandline, provide a path with '--export-path <path>' after ' -- '."

    output_path = export(
        export_path,
        {
            "armature": get_armature(),
            "round_collision_scales": args.round_collision_scales,
            "fix_collision_scales": args.fix_collision_scales,
            #'def_limit_effort': 100, #custom default effort limit for joints
            #'def_limit_vel': 3, #custom default vel limit for joints
        },
        export_urdf=export_urdf,
        export_meshes=export_meshes,
        export_ao_config=export_ao_config,
        fix_materials=args.fix_materials,
        link_visuals=not args.no_link_visuals,
        collision_visuals=args.collision_visuals,
        joint_visuals=args.joint_visuals,
        receptacle_visuals=args.receptacle_visuals,
    )
    print(f"\n ======== Output saved to {output_path} ========\n")
