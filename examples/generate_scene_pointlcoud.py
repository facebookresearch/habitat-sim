import argparse
from typing import Any, Dict, List

import numpy as np
import open3d as o3d

from habitat_sim import Simulator
from habitat_sim.utils.settings import default_sim_settings, make_cfg

# Slicing code adapted from https://stackoverflow.com/questions/75082217/crop-function-that-slices-triangles-instead-of-removing-them-open3d


def sliceplane(mesh, axis, value, direction):
    # axis can be 0,1,2 (which corresponds to x,y,z)
    # value where the plane is on that axis
    # direction can be True or False (True means remove everything that is
    # greater, False means less
    # than)

    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    new_vertices = list(vertices)
    new_triangles = []

    # (a, b) -> c
    # c refers to index of new vertex that sits at the intersection between a,b
    # and the boundingbox edge
    # a is always inside and b is always outside
    intersection_edges = dict()

    # find axes to compute
    axes_compute = [0, 1, 2]
    # remove axis that the plane is on
    axes_compute.remove(axis)

    def compute_intersection(vertex_in_index, vertex_out_index):
        vertex_in = vertices[vertex_in_index]
        vertex_out = vertices[vertex_out_index]
        if (vertex_in_index, vertex_out_index) in intersection_edges:
            intersection_index = intersection_edges[(vertex_in_index, vertex_out_index)]
            intersection = new_vertices[intersection_index]
        else:
            intersection = [None, None, None]
            intersection[axis] = value
            const_1 = (value - vertex_in[axis]) / (vertex_out[axis] - vertex_in[axis])
            c = axes_compute[0]
            intersection[c] = (const_1 * (vertex_out[c] - vertex_in[c])) + vertex_in[c]
            c = axes_compute[1]
            intersection[c] = (const_1 * (vertex_out[c] - vertex_in[c])) + vertex_in[c]
            assert None not in intersection
            # save new vertice and remember that this intersection already added an edge
            new_vertices.append(intersection)
            intersection_index = len(new_vertices) - 1
            intersection_edges[(vertex_in_index, vertex_out_index)] = intersection_index

        return intersection_index

    for t in triangles:
        v1, v2, v3 = t
        if direction:
            v1_out = vertices[v1][axis] > value
            v2_out = vertices[v2][axis] > value
            v3_out = vertices[v3][axis] > value
        else:
            v1_out = vertices[v1][axis] < value
            v2_out = vertices[v2][axis] < value
            v3_out = vertices[v3][axis] < value

        bool_sum = sum([v1_out, v2_out, v3_out])
        # print(f"{v1_out=}, {v2_out=}, {v3_out=}, {bool_sum=}")

        if bool_sum < 0 or bool_sum > 3:
            raise AssertionError()

        if bool_sum == 0:
            # triangle completely inside --> add and continue
            new_triangles.append(t)
        elif bool_sum == 3:
            # triangle completely outside --> skip
            pass
        elif bool_sum == 2:
            # two vertices outside
            # add triangle using both intersections
            vertex_in_index = v1 if (not v1_out) else (v2 if (not v2_out) else v3)
            vertex_out_1_index = v1 if v1_out else (v2 if v2_out else v3)
            vertex_out_2_index = v3 if v3_out else (v2 if v2_out else v1)
            # print(f"{vertex_in_index=}, {vertex_out_1_index=}, {vertex_out_2_index=}")
            # small sanity check if indices sum matches
            assert sum(
                [vertex_in_index, vertex_out_1_index, vertex_out_2_index]
            ) == sum([v1, v2, v3])

            # add new triangle
            new_triangles.append(
                [
                    vertex_in_index,
                    compute_intersection(vertex_in_index, vertex_out_1_index),
                    compute_intersection(vertex_in_index, vertex_out_2_index),
                ]
            )

        elif bool_sum == 1:
            # one vertex outside
            # add three triangles
            vertex_out_index = v1 if v1_out else (v2 if v2_out else v3)
            vertex_in_1_index = v1 if (not v1_out) else (v2 if (not v2_out) else v3)
            vertex_in_2_index = v3 if (not v3_out) else (v2 if (not v2_out) else v1)
            # print(f"{vertex_out_index=}, {vertex_in_1_index=}, {vertex_in_2_index=}")
            # small sanity check if outdices sum matches
            assert sum([vertex_out_index, vertex_in_1_index, vertex_in_2_index]) == sum(
                [v1, v2, v3]
            )

            new_triangles.append(
                [
                    vertex_in_1_index,
                    compute_intersection(vertex_in_1_index, vertex_out_index),
                    vertex_in_2_index,
                ]
            )
            new_triangles.append(
                [
                    compute_intersection(vertex_in_1_index, vertex_out_index),
                    compute_intersection(vertex_in_2_index, vertex_out_index),
                    vertex_in_2_index,
                ]
            )

    # TODO remap indices and remove unused

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(np.array(new_vertices))
    mesh.triangles = o3d.utility.Vector3iVector(np.array(new_triangles))
    return mesh


def clean_crop_xy(mesh, min_corner, max_corner):
    min_x = min(min_corner[0], max_corner[0])
    min_y = min(min_corner[1], max_corner[1])
    max_x = max(min_corner[0], max_corner[0])
    max_y = max(min_corner[1], max_corner[1])

    # mesh = sliceplane(mesh, 0, min_x, False)
    mesh_sliced = sliceplane(mesh, 0, max_x, True)
    mesh_sliced = sliceplane(mesh_sliced, 0, min_x, False)
    mesh_sliced = sliceplane(mesh_sliced, 1, max_y, True)
    mesh_sliced = sliceplane(mesh_sliced, 1, min_y, False)
    # mesh_sliced = mesh_sliced.paint_uniform_color([0,0,1])

    return mesh_sliced


def main(
    dataset: str = "", scene: str = "", point: List[float] = None, epsilon: float = 0.0
):
    """
    Initialize the simulator and generate a point cloud of the joined scene.
    """
    sim_settings: Dict[str, Any] = default_sim_settings
    sim_settings["scene_dataset_config_file"] = dataset
    sim_settings["scene"] = scene
    cfg = make_cfg(sim_settings)
    mesh = None
    with Simulator(cfg) as sim:
        mesh_verts = sim.get_joined_mesh_verts(include_static_objects=True)
        mesh_indices = sim.get_joined_mesh_indices(include_static_objects=True)
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(mesh_verts)
        tris = [mesh_indices[i : i + 3] for i in range(0, len(mesh_indices), 3)]
        mesh.triangles = o3d.utility.Vector3iVector(tris)
        o3d.io.write_triangle_mesh(f"{scene}_joined_mesh.ply", mesh)

    o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)

    # Set bounds to an epsilon distance from a given point
    # NOTE: this point is the center of the table in scene 102344193

    min_bound = [p - epsilon for p in point]
    max_bound = [p + epsilon for p in point]
    for i in range(3):
        mesh = sliceplane(mesh, i, min_bound[i], False)
        mesh = sliceplane(mesh, i, max_bound[i], True)

    o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)

    pcd = mesh.sample_points_poisson_disk(number_of_points=10000, init_factor=5)
    o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud(f"{scene}_pointcloud.pcd", pcd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate scene point cloud.")
    parser.add_argument(
        "--dataset",
        type=str,
        default="data/scene_datasets/hssd-hab/hssd-hab-articulated.scene_dataset_config.json",
        help="Path to the dataset config file",
    )
    parser.add_argument(
        "--scene", type=str, default="102344193", help="Scene name or ID"
    )
    args = parser.parse_args()

    # TODO: make these command line args
    point = [-2.97411, 0.761936, -7.4821]  # Replace with your desired point coordinates
    epsilon = 2.0  # Replace with your desired epsilon value
    main(dataset=args.dataset, scene=args.scene, point=point, epsilon=epsilon)
