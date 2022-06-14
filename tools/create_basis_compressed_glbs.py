import argparse
import glob
import itertools
import json
import multiprocessing
import os
import os.path as osp
import shlex
import shutil
import subprocess
from typing import Any, Callable, List, Optional, Tuple

import tqdm

TOOL_PATH = osp.realpath(
    osp.join(
        osp.dirname(__file__),
        "..",
        "src/deps/magnum-plugins/src/MagnumPlugins/GltfImporter/Test",
    )
)

IMAGE_CONVERTER_DEFAULT = "build/utils/imageconverter/magnum-imageconverter"


def build_parser(
    parser: Optional[argparse.ArgumentParser] = None,
) -> argparse.ArgumentParser:
    if parser is None:
        parser = argparse.ArgumentParser(
            description="Tool to convert all glbs in a given directory to basis compressed glbs."
            "  This allows them to be loaded faster onto the GPU and use between 4x and 6x less GPU memory."
            "  Partial progress of compressing images is supported.  Killing the script half way through"
            " compressing images and then restarting it will not result in re-compressing any images that"
            " already compressed.  If you want to change compression settings, use '--pre-clean' to remove"
            " partial progress.",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )

    parser.add_argument("basedir", type=str, help="Base directory to find GLBs in.")

    parser.add_argument(
        "--num-workers",
        default=2,
        type=int,
        help="Number of workers to run in parallel. Most of the compression process is multi-threaded and set to use 4 threads.  Budget around ~3 CPU cores per worker.",
    )
    parser.add_argument(
        "--pre-clean",
        action="store_true",
        help="Clean artifacts before running the script",
    )
    parser.add_argument(
        "--rename-basis",
        action="store_true",
        help="Rename the basis meshes to *.glb, and original glbs to *.glb.orig.  "
        "Otherwise the basis meshes are named *.basis.glb",
    )
    parser.add_argument(
        "--niceness", type=int, default=10, help="Niceness value for all the workers"
    )
    parser.add_argument(
        "--convert-unlit",
        action="store_true",
        help="Convert meshes to unlit.  "
        "This is useful when processing reconstructions of real scenes or objects that have 'baked' lighting as it signifies to habitat that this object should use flat shading.",
    )

    parser.add_argument(
        "--magnum-imageconverter",
        type=str,
        default=IMAGE_CONVERTER_DEFAULT,
        help="Path to magnum imageconverter tool.  This must be built with basis encoding support.",
    )

    parser.add_argument(
        "--basis-compression-level",
        type=int,
        default=2,
        help="The compression level to pass to the basis compressor.  "
        "A higher level will result in a (possibly) better compression but will take considerably more time.",
    )

    return parser


def extract_images(mesh_name: str) -> None:
    output_dir = osp.splitext(mesh_name)[0] + "_hab_basis_tool"
    os.makedirs(output_dir, exist_ok=True)

    tool = osp.join(TOOL_PATH, "glb2gltf.py")
    mesh_name = osp.abspath(mesh_name)
    output_name = osp.splitext(osp.basename(mesh_name))[0] + ".gltf"

    subprocess.check_output(
        shlex.split(f"{tool} {mesh_name} --extract-images --output {output_name}"),
        cwd=output_dir,
    )


def img_name_to_basis(img: str) -> str:
    return osp.splitext(img)[0] + ".basis"


def convert_image_to_basis(args: Tuple[str, str, int]) -> None:
    img, imageconverter, compression_level = args
    basis_img = img_name_to_basis(img)

    settings = f"threads=4,mip_gen=true,compression_level={compression_level}"

    _ = subprocess.check_output(
        shlex.split(
            f"{imageconverter} {img} {basis_img} --converter BasisImageConverter -c {settings}"
        )
    )


def _gltf2unlit(gltf_name: str):
    assert osp.exists(gltf_name)
    with open(gltf_name, "r") as f:
        json_data = json.load(f)
    # add references to the KHR_materials_unlit extension in
    # gltf root-level tags
    ext_gltf_tags = ["extensionsUsed", "extensionsRequired"]
    for ext_tag in ext_gltf_tags:
        if ext_tag not in json_data:
            json_data[ext_tag] = []
        if "KHR_materials_unlit" not in json_data[ext_tag]:
            json_data[ext_tag].append("KHR_materials_unlit")

    for material in json_data["materials"]:
        assert "pbrMetallicRoughness" in material

        # Drop everything except base color and base color texture
        pbrMetallicRoughness = material["pbrMetallicRoughness"]
        for key in pbrMetallicRoughness.keys():
            if key not in ["baseColorFactor", "baseColorTexture"]:
                del pbrMetallicRoughness[key]
        for key in [
            "normalTexture",
            "occlusionTexture",
            "emissiveFactor",
            "emissiveTexture",
        ]:
            if key in material:
                del material[key]

        # Add the extension
        if "extensions" not in material:
            material["extensions"] = {}
        material["extensions"]["KHR_materials_unlit"] = {}

    with open(gltf_name, "wb") as f:
        f.write(json.dumps(json_data, indent=2).encode("utf-8"))


def package_meshes(args: Tuple[str, bool]) -> None:
    mesh_name, convert_to_unlit = args
    output_dir = osp.splitext(mesh_name)[0] + "_hab_basis_tool"
    base_mesh_name = osp.splitext(osp.basename(mesh_name))[0]

    tool = osp.join(TOOL_PATH, "gltf2basis.py")
    _ = subprocess.check_output(
        shlex.split(f"{tool} {base_mesh_name}.gltf {base_mesh_name}.basis.gltf"),
        cwd=output_dir,
    )

    if convert_to_unlit:
        _gltf2unlit(osp.join(output_dir, f"{base_mesh_name}.basis.gltf"))

    tool = osp.join(TOOL_PATH, "gltf2glb.py")
    _ = subprocess.check_output(
        shlex.split(f"{tool} {base_mesh_name}.basis.gltf --bundle-images"),
        cwd=output_dir,
    )

    shutil.copy(
        osp.join(output_dir, f"{base_mesh_name}.basis.glb"),
        osp.join(osp.dirname(mesh_name), f"{base_mesh_name}.basis.glb"),
    )


def finalize(output_folder: str, rename_basis: bool) -> None:
    if rename_basis:
        for basis_mesh in glob.glob(
            osp.join(output_folder, "**", "*.basis.glb"), recursive=True
        ):

            # skip false positives in the working directories which won't actually exist
            if "hab_basis_tool/" in basis_mesh:
                # print(f"...skipping {basis_mesh}")
                continue

            mesh_name = osp.splitext(osp.splitext(basis_mesh)[0])[0] + ".glb"

            shutil.move(mesh_name, mesh_name + ".orig")

            shutil.move(basis_mesh, mesh_name)


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

    if args.rename_basis:
        _clean(glob.glob(f"{folder}/**/*.basis.glb", recursive=True))


def main():
    args = build_parser().parse_args()
    # Doing a which first to also suport just passing magnum-imageconverter
    # if that is installed globally
    args.magnum_imageconverter = shutil.which(args.magnum_imageconverter)
    args.magnum_imageconverter = osp.realpath(args.magnum_imageconverter)
    if not osp.exists(args.magnum_imageconverter):
        raise RuntimeError(
            "Could not find magnum-imageconverter.  "
            "If you are using a non standard built, specify with the --magnum-imageconverter arg."
            "\nNote that habitat needs to be built with '--build-basis-compressor'."
        )

    if not osp.exists(TOOL_PATH):
        raise RuntimeError(
            "Could not find magnum tools used to convert meshes.  "
            "This tool is currently designed to be run in the habitat-sim repo with submodules cloned."
        )

    files = glob.glob(osp.join(args.basedir, "**", "*.glb"), recursive=True)
    files = [f for f in files if ".basis." not in f]
    files = [f for f in files if "_convex" not in f]

    if args.pre_clean:
        clean_up(args.basedir, args)

    with multiprocessing.Pool(
        args.num_workers, initializer=os.nice, initargs=(args.niceness,)
    ) as pool:
        print("Extracting images...")
        _map_all_and_wait(pool, extract_images, files)

        # Convert to basis
        images = []
        for ext in ("jpg", "png", "jpeg"):
            images += list(
                glob.glob(f"{args.basedir}/**/*_hab_basis_tool/*.{ext}", recursive=True)
            )

        images = list(
            filter(lambda img: not osp.exists(img_name_to_basis(img)), images)
        )

        print(f"Compressing {len(images)} images with basis...")
        _map_all_and_wait(
            pool,
            convert_image_to_basis,
            list(
                zip(
                    images,
                    itertools.repeat(args.magnum_imageconverter),
                    itertools.repeat(args.basis_compression_level),
                )
            ),
        )

        # Make final meshes
        print("Creating basis meshes...")
        _map_all_and_wait(
            pool,
            package_meshes,
            list(zip(files, itertools.repeat(args.convert_unlit))),
        )

    print("Finalizing...")
    finalize(args.basedir, args.rename_basis)
    # Do cleanup
    clean_up(args.basedir, args)
    print(f"Done. Converted all glbs in {args.basedir} to glbs")


if __name__ == "__main__":
    main()
