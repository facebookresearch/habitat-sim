import argparse

from .train import InstanceSegmentationEnvironment

defaults = {"scene": "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"}


def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        "--scene",
        default=defaults["scene"],
        required=True,
        help="The mesh file for the scene passed to the extractor, usually a .glb or .ply file.",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()
    env = InstanceSegmentationEnvironment(scene=args.scene)
    env.train()
