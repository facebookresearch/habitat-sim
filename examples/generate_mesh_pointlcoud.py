import open3d as o3d


def main():
    mesh_file_root = (
        "/home/alexclegg/Documents/dev2/habitat-sim/data/objects/ycb/meshes/"
    )
    meshes = [
        "005_tomato_soup_can",
        "002_master_chef_can",
        "003_cracker_box",
        "004_sugar_box",
        "007_tuna_fish_can",
        "010_potted_meat_can",
        "024_bowl",
        "011_banana",
        "025_mug",
        "029_plate",
        "033_spatula",
        "048_hammer",
    ]
    mesh_postfix = "/google_16k/textured.glb"

    for mesh_name in meshes:
        mesh = o3d.io.read_triangle_mesh(mesh_file_root + mesh_name + mesh_postfix)
        pcd = mesh.sample_points_poisson_disk(number_of_points=500, init_factor=5)
        o3d.visualization.draw_geometries([pcd])
        o3d.io.write_point_cloud(f"{mesh_name}.pcd", pcd)

    # test reading back in
    # pcd2 = o3d.io.read_point_cloud("sampled_pointcloud.pcd")
    # o3d.visualization.draw_geometries([pcd2])


if __name__ == "__main__":
    main()
