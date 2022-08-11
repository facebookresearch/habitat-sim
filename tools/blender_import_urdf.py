import math
import os
import xml.etree.ElementTree as ET

import bpy
import mathutils
import numpy as np


def to_list(vector_element):
    # convert an xml vector element to a list of float
    return [float(x) for x in vector_element.split(" ")]


class BoxShape:
    def __init__(self, size, collision=False) -> None:
        self.size = size
        self.collision = collision

    def create_mesh(self):
        # create and return a box in blender for this shape named 'box.###'
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        box_object = bpy.context.selected_objects[0]
        box_object.scale = self.size
        box_object.name = "box_shape"
        if self.collision:
            box_object.name = "collision_" + box_object.name
        return box_object


class SphereShape:
    def __init__(self, radius, collision=False) -> None:
        self.radius = radius
        self.collision = collision

    def create_mesh(self):
        # create and return a sphere in blender for this shape named 'Sphere'
        bpy.ops.mesh.primitive_uv_sphere_add(radius=self.radius)
        sphere_object = bpy.context.selected_objects[0]
        sphere_object.name = "sphere_shape"
        if self.collision:
            sphere_object.name = "collision_" + sphere_object.name
        return sphere_object


# NOTE: global set during parse for directory joins
global urdf_source_dir


class MeshShape:
    def __init__(self, filepath, scale=np.ones(3), collision=False) -> None:
        self.filepath = os.path.join(urdf_source_dir, filepath)
        self.scale = scale
        self.collision = collision

    def create_mesh(self):
        # import the referenced mesh and return the resulting object
        # add a parent frame for the mesh(es)
        bpy.ops.object.empty_add()
        mesh_parent_frame = bpy.context.selected_objects[0]
        mesh_parent_frame.name = "mesh_shape_parent_frame"
        if self.collision:
            mesh_parent_frame.name = "collision_" + mesh_parent_frame.name
        # import the meshes
        print(self.filepath)
        if self.filepath.endswith(".glb") or self.filepath.endswith(".gltf"):
            bpy.ops.import_scene.gltf(filepath=self.filepath)
            # correct rotation frame for glTF to Blender
            to_hab = mathutils.Quaternion((1.0, 0.0, 0.0), math.radians(-90.0))
            mesh_objects = bpy.context.selected_objects
            for obj in mesh_objects:
                if obj.parent is None:
                    obj.rotation_quaternion.rotate(to_hab)
                    obj.location.rotate(to_hab)
        elif self.filepath.endswith(".obj"):
            bpy.ops.import_scene.obj(filepath=self.filepath)
        elif self.filepath.endswith(".ply"):
            bpy.ops.import_mesh.ply(filepath=self.filepath)
        elif self.filepath.endswith(".stl"):
            bpy.ops.import_mesh.stl(filepath=self.filepath)
        elif self.filepath.endswith(".dae"):
            bpy.ops.wm.collada_import(filepath=self.filepath)
        else:
            assert False, "The mesh asset type is not supported '{self.filepath}'."
        mesh_objects = bpy.context.selected_objects
        print(len(mesh_objects))
        for object in mesh_objects:
            if object.parent is None:
                object.parent = mesh_parent_frame
        return mesh_parent_frame


class URDF:
    # a collection of Links joined by Joints
    def __init__(self, name) -> None:
        self.name = name
        self.links = {}
        self.joints = {}

    def create_blender_objects(self, render=True, collision=True):
        # create a collection for the robot name
        # bpy.ops.collection.create(name=self.name)
        # bpy.context.scene.collection.children.link(bpy.data.collections[self.name])
        # return
        # start adding objects in the new collection
        for link_name, link in self.links.items():
            link.create_blender_objects(render=render, collision=collision)
        # create the joints and parenting
        for joint_name, joint in self.joints.items():
            # create an empty node for the joint
            bpy.ops.object.empty_add()
            joint_frame = bpy.context.selected_objects[0]
            joint_frame.name = f"joint_{joint_name}"
            # setup parent/child hierarchy
            parent_link = bpy.data.objects[f"link_{joint.parent}"]
            joint_frame.parent = parent_link
            joint_frame.location = joint.xyz
            joint_frame.rotation_euler = joint.rpy
            child_link = bpy.data.objects[f"link_{joint.child}"]
            child_link.parent = joint_frame


class Joint:
    def __init__(self, name, joint_type=None) -> None:
        self.name = name
        self.joint_type = joint_type
        self.parent = None
        self.child = None
        self.xyz = np.zeros(3)
        self.rpy = np.zeros(3)

    def __str__(self):
        string_rep = f"JOINT - {self.name}"
        string_rep += f"\n    type: {self.joint_type}"
        string_rep += f"\n    rpy: {self.rpy}"
        string_rep += f"\n    xyz: {self.xyz}"
        string_rep += f"\n    parent_link: {self.parent}"
        string_rep += f"\n    child_link: {self.child}"
        return string_rep


class Link:
    def __init__(self, name) -> None:
        self.name = name
        # NOTE: these transforms are not used for geometric relationships, only physics
        self.xyz = np.zeros(3)
        self.rpy = np.zeros(3)
        self.collision_objects = []
        self.render_objects = []

        self.parent_joint = None

    def __str__(self):
        string_rep = f"LINK - {self.name}"
        string_rep += f"\n    parent: {self.parent_joint}"
        # string_rep += f"\n    rpy: {self.rpy}"
        # string_rep += f"\n    xyz: {self.xyz}"
        string_rep += f"\n    visual: {self.render_objects}"
        string_rep += f"\n    collision: {self.collision_objects}"
        return string_rep

    def create_blender_objects(self, parent_object=None, render=True, collision=True):
        # create an empty root node for the link
        bpy.ops.object.empty_add()
        link_parent_frame = bpy.context.selected_objects[0]
        link_parent_frame.name = f"link_{self.name}"

        if render:
            for render_object in self.render_objects:
                print("creat link render objects")
                shape_object = render_object["shape"].create_mesh()
                shape_object.parent = link_parent_frame
                shape_object.location = render_object["xyz"]
                shape_object.rotation_euler = render_object["rpy"]

        if collision:
            for collision_object in self.collision_objects:
                print("creat link collision objects")
                shape_object = collision_object["shape"].create_mesh()
                shape_object.parent = link_parent_frame
                shape_object.location = collision_object["xyz"]
                shape_object.rotation_euler = collision_object["rpy"]

        # NOTE: these transforms are not used for geometric relationships, only physics
        # link_parent_frame.location = self.xyz
        # link_parent_frame.rotation_euler = self.rpy

        return link_parent_frame


def parse_geom(geom_element, collision=False):
    # parse a geometry element and return a shape object
    shape = None
    if geom_element.tag == "mesh":
        shape = MeshShape(geom_element.get("filename"), collision=collision)
        if "scale" in geom_element.attrib:
            shape.scale = np.array(list(geom_element.get("scale")))
    elif geom_element.tag == "box":
        shape = BoxShape(to_list(geom_element.get("size")), collision=collision)
    elif geom_element.tag == "sphere":
        shape = SphereShape(float(geom_element.get("radius")), collision=collision)
    else:
        assert False, f"parse_geom() - Shape '{geom_element.tag}' not supported."
    return shape


def parse_link(link_element):
    # parse a link element into a Link object
    link = Link(link_element.attrib.get("name"))
    for inertial in link_element.findall("inertial"):
        link.rpy = to_list(inertial.find("origin").get("rpy"))
        link.xyz = to_list(inertial.find("origin").get("xyz"))
    for visual in link_element.findall("visual"):
        shape = parse_geom(visual.find("geometry")[0])
        link.render_objects.append(
            {
                "shape": shape,
                "xyz": to_list(visual.find("origin").get("xyz")),
                "rpy": to_list(visual.find("origin").get("rpy")),
            }
        )
    for collision in link_element.findall("collision"):
        shape = parse_geom(collision.find("geometry")[0], collision=True)
        link.collision_objects.append(
            {
                "shape": shape,
                "xyz": to_list(collision.find("origin").get("xyz")),
                "rpy": to_list(collision.find("origin").get("rpy")),
            }
        )

    print(link)

    return link


def parse_joint(joint_element):
    # parse a jointelement into a Joint object
    joint = Joint(
        joint_element.attrib.get("name"), joint_type=joint_element.attrib.get("type")
    )

    joint.parent = joint_element.find("parent").get("link")
    joint.child = joint_element.find("child").get("link")

    joint.rpy = to_list(joint_element.find("origin").get("rpy"))
    joint.xyz = to_list(joint_element.find("origin").get("xyz"))

    print(joint)

    return joint


def parse_urdf(urdf_file):
    # NOTE: setting the global for asset directory joins
    global urdf_source_dir
    urdf_source_dir = urdf_file[: -len(urdf_file.split("/")[-1])]
    # create element tree object
    tree = ET.parse(urdf_file)

    # get root element
    root = tree.getroot()

    # setup the URDF structure
    urdf = URDF(name=root.get("name"))
    urdf.links = {}
    urdf.joints = {}

    print(f"root = {root.tag}")

    for child in root:
        # links and joints
        # print(f"    child = {child.tag} | {child.attrib}")
        if child.tag == "link":
            urdf.links[child.get("name")] = parse_link(child)
        elif child.tag == "joint":
            urdf.joints[child.get("name")] = parse_joint(child)

    # validate references
    for j_name, joint in urdf.joints.items():
        assert (
            joint.parent in urdf.links.keys()
        ), f" joint '{j_name}' parent link '{joint.parent}' is not registered."
        assert (
            joint.child in urdf.links.keys()
        ), f" joint '{j_name}' child link '{joint.child}' is not registered."

    return urdf


if __name__ == "__main__":
    # parse the target urdf file
    urdf = parse_urdf(
        "/run/media/alexclegg/Extreme_SSD/datasets/fremont/urdf/kitchen/kitchen_cupboard/kitchen_cupboard.urdf"
    )
    # create a blender object heirarchy for the URDF
    urdf.create_blender_objects(render=True, collision=True)
