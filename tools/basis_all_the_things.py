import argparse
import glob
import multiprocessing
import os
import os.path as osp
import shlex
import shutil
import subprocess
from typing import Any, Callable, List, Optional

import tqdm

TOOL_PATH = osp.realpath(
    osp.join(
        osp.dirname(__file__),
        "..",
        "src/deps/magnum-plugins/src/MagnumPlugins/TinyGltfImporter/Test",
    )
)

IMAGE_CONVERTER = "build/utils/imageconverter/magnum-imageconverter"


def build_parser(
    parser: Optional[argparse.ArgumentParser] = None,
) -> argparse.ArgumentParser:
    if parser is None:
        parser = argparse.ArgumentParser(
            description="Tool to convert all glbs in a given directory to basis compressed glbs."
            "  This allows them to be loaded faster onto the GPU and use between 4x and 6x less GPU memory.",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )

    parser.add_argument(
        "--num-workers",
        default=2,
        type=int,
        help="Number of workers to run in parallel. Most of the compression process is multi-threaded and set to use 4 threads.  Budget around ~3 CPU cores per worker.",
    )
    parser.add_argument(
        "--basedir", type=str, required=True, help="Base directory to find GLBs in."
    )
    parser.add_argument(
        "--pre-clean",
        action="store_true",
        help="Clean artifacts before running the script",
    )
    parser.add_argument(
        "--inplace",
        action="store_true",
        help="Compresses meshes in place.  If false, a copy of basedir will be made and compressed meshes will be put in there.",
    )
    parser.add_argument(
        "--rename-basis",
        action="store_true",
        help="Rename the basis meshes to *.glb, and if done inplace original glbs to *.glb.orig.  Otherwise the basis meshes are named *.basis.glb",
    )
    parser.add_argument(
        "--niceness", type=int, default=10, help="Niceness value for all the workers"
    )
    return parser


def set_nicesness(niceness):
    os.nice(niceness)


def extract_images(mesh_name: str) -> None:
    output_dir = osp.splitext(mesh_name)[0] + "_hab_basis_tool"
    os.makedirs(output_dir, exist_ok=True)

    tool = osp.join(TOOL_PATH, "glb2gltf.py")
    tool = osp.relpath(tool, output_dir)
    mesh_name = osp.relpath(osp.abspath(mesh_name), output_dir)

    subprocess.check_output(
        shlex.split(f"{tool} {mesh_name} --extract-images"),
        cwd=output_dir,
    )


def img_name_to_basis(img: str) -> str:
    return osp.splitext(img)[0] + ".basis"


def convert_image_to_basis(img: str) -> None:
    basis_img = img_name_to_basis(img)

    settings = "threads=4,mip_gen=true,compression_level=2"

    _ = subprocess.check_output(
        shlex.split(
            f"{IMAGE_CONVERTER} {img} {basis_img} --converter BasisImageConverter -c {settings}"
        )
    )


def package_meshes(mesh_name: str) -> None:
    output_dir = osp.splitext(mesh_name)[0] + "_hab_basis_tool"
    base_mesh_name = osp.join("..", osp.splitext(osp.basename(mesh_name))[0])

    tool = osp.join(TOOL_PATH, "gltf2basis.py")
    tool = osp.relpath(tool, output_dir)

    _ = subprocess.check_output(
        shlex.split(f"{tool} {base_mesh_name}.gltf {base_mesh_name}.basis.gltf"),
        cwd=output_dir,
    )

    tool = osp.join(TOOL_PATH, "gltf2glb.py")
    tool = osp.relpath(tool, output_dir)
    _ = subprocess.check_output(
        shlex.split(f"{tool} {base_mesh_name}.basis.gltf --bundle-images"),
        cwd=output_dir,
    )


def finalize(folder: str, inplace: bool, rename_basis: bool) -> str:
    if inplace:
        output_folder = folder
    else:
        output_folder = folder + "_basis"

        if osp.exists(output_folder):
            shutil.rmtree(output_folder)

        shutil.copytree(folder, output_folder)

    if rename_basis:
        for basis_mesh in glob.glob(
            osp.join(output_folder, "**", "*.basis.glb"), recursive=True
        ):
            mesh_name = osp.splitext(osp.splitext(basis_mesh)[0])[0] + ".glb"

            if inplace:
                shutil.move(mesh_name, mesh_name + ".orig")

            shutil.move(basis_mesh, mesh_name)

    return output_folder


def _map_all_and_wait(pool: multiprocessing.Pool, func: Callable, inputs: List[Any]):
    with tqdm.tqdm(total=len(inputs)) as pbar:
        for _ in pool.imap_unordered(func, inputs):
            pbar.update()


def _clean(lst: List[str]) -> None:
    if len(lst) > 0:
        print("Cleaning...")

    for f in tqdm.tqdm(lst):
        if osp.isdir(f):
            shutil.rmtree(f)
        else:
            os.remove(f)


def clean_up(folder: str, args) -> None:
    _clean(glob.glob(f"{folder}/**/*_hab_basis_tool", recursive=True))

    _clean(glob.glob(f"{folder}/**/*.gltf", recursive=True))

    _clean(glob.glob(f"{folder}/**/*.basis", recursive=True))

    _clean(glob.glob(f"{folder}/**/*.bin", recursive=True))

    if args.rename_basis:
        _clean(glob.glob(f"{folder}/**/*.basis.glb", recursive=True))


def main():
    args = build_parser().parse_args()
    if not osp.exists(IMAGE_CONVERTER):
        raise RuntimeError(
            "Could not find imageconverter.  "
            "Habitat needs to be built with '--build-basis-compressor' to use this tool."
        )

    if not osp.exists(TOOL_PATH):
        raise RuntimeError("Could not find magnum tools we use to convert meshes")

    files = glob.glob(osp.join(args.basedir, "**", "*.glb"), recursive=True)
    files = [f for f in files if ".basis." not in f]
    files = [f for f in files if "_convex" not in f]

    if args.pre_clean:
        clean_up(args.basedir, args)

    with multiprocessing.Pool(
        args.num_workers, initializer=set_nicesness, initargs=(args.niceness,)
    ) as pool:
        print("Extracting images...")
        _map_all_and_wait(pool, extract_images, files)

        # Convert to basis
        images = []
        images += list(
            glob.glob(f"{args.basedir}/**/*_hab_basis_tool/*.jpg", recursive=True)
        )
        images += list(
            glob.glob(f"{args.basedir}/**/*_hab_basis_tool/*.png", recursive=True)
        )
        images += list(
            glob.glob(f"{args.basedir}/**/*_hab_basis_tool/*.jpeg", recursive=True)
        )
        images = list(
            filter(lambda img: not osp.exists(img_name_to_basis(img)), images)
        )

        print(f"Compressing {len(images)} images with basis...")
        _map_all_and_wait(pool, convert_image_to_basis, images)

        # Make final meshes
        print("Creating basis meshes...")
        _map_all_and_wait(pool, package_meshes, files)

    print("Finalizing...")
    converted_name = finalize(args.basedir, args.inplace, args.rename_basis)
    # Do cleanup
    clean_up(args.basedir, args)
    clean_up(converted_name, args)
    print(f"Done. Basis meshes in {converted_name}")


if __name__ == "__main__":
    main()
