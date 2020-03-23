import argparse

from examples.instance_segmentation.train import InstanceSegmentationEnvironment

defaults = {
    "scene": "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
    "epochs": 100,
    "lr": 0.0005,
    "momentum": 0.9,
}


def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        "--scene",
        default=defaults["scene"],
        help="The mesh file for the scene passed to the extractor, usually a .glb or .ply file.",
    )
    parser.add_argument(
        "--epochs",
        default=defaults["epochs"],
        help="The number of epochs to train the model. (Optional)",
    )
    parser.add_argument(
        "--lr", default=defaults["lr"], help="Learning rate for the model. (Optional)"
    )
    parser.add_argument(
        "--momentum",
        default=defaults["momentum"],
        help="Momentum for the optimizer. (Optional)",
    )
    parser.add_argument(
        "--load-path",
        default=None,
        dest="load_path",
        help="Path to load weights model weights from. (Optional)",
    )
    parser.add_argument(
        "--save-path",
        default=None,
        dest="save_path",
        help="Path to save weights model weights to. (Optional)",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()
    env = InstanceSegmentationEnvironment(
        scene=args.scene, lr=args.lr, momentum=args.momentum
    )
    env.train(epochs=args.epochs, load_path=args.load_path, save_path=args.save_path)
