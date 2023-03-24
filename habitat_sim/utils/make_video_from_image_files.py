#!/usr/bin/env python3
# Copyright (c) Meta Platforms, Inc. All Rights Reserved
# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import argparse
import glob
import os
from argparse import ArgumentParser, Namespace
from PIL import Image
import numpy as np
from habitat_sim.utils import viz_utils as vut

def get_filepaths_from_directory(directory, filepath_glob):
    """Returns a list of filepaths."""
    filepaths = []
    os.chdir(directory)
    for filepath in glob.glob("*" + filepath_glob):
        filepaths.append(filepath)
    return filepaths


def create_arg_parser() -> ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image-glob",
        default="*.bmp",
    )
    parser.add_argument(
        "--image-glob2",
        default=None,
    )
    parser.add_argument(
        "--output",
        default="output.mp4"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30
    )
    parser.add_argument(
        "--trim-start",
        type=int,
        default=0
    )
    parser.add_argument(
        "--trim-end",
        type=int,
        default=0
    )
    parser.add_argument(
        "--pause-start",
        type=int,
        default=0
    )
    parser.add_argument(
        "--pause-end",
        type=int,
        default=0
    )
    return parser


def main():
    args = create_arg_parser().parse_args()

    video_file = args.output
    if not video_file.endswith(".mp4"):
        video_file = video_file + ".mp4"

    filepaths = get_filepaths_from_directory("./", args.image_glob)
    if args.image_glob2:
        filepaths2 = get_filepaths_from_directory("./", args.image_glob2)

    if len(filepaths) == 0:
        print("No files matching {} were found in the working directory.".format(args.image_glob))
        return

    if filepaths2 and len(filepaths) != len(filepaths2):
        print("mismatch: {} files for {} and {} files for {}".format(
            len(filepaths), args.image_glob, len(filepaths2), args.image_glob))

    filepaths.sort()  # sort alphabetically
    filepaths2.sort()  # sort alphabetically

    writer = vut.get_fast_video_writer(video_file, fps=args.fps)

    if args.image_glob2:
        for (i, filepath) in enumerate(filepaths):
            if i < args.trim_start or i >= len(filepaths) - args.trim_end:
                continue
            with Image.open(filepath) as im:
                filepath2 = filepaths2[i]
                with Image.open(filepath2) as im2:

                    print("appending {} and {}...".format(filepath, filepath2))

                    width, height = im.size
                    new_im = Image.new('RGB', (width * 2, height))
                    new_im.paste(im, (0,0))
                    new_im.paste(im2, (width,0))

                    data = np.asarray(new_im)
                    writer.append_data(data)
                    if i == args.trim_start:
                        for j in range(args.pause_start):
                            writer.append_data(data)
                    elif i == len(filepaths) - args.trim_end - 1:
                        for j in range(args.pause_end):
                            writer.append_data(data)
    else:
        for (i, filepath) in enumerate(filepaths):
            if i < args.trim_start or i >= len(filepaths) - args.trim_end:
                continue
            with Image.open(filepath) as im:
                print("appending {}...", format(filepath))
                data = np.asarray(im)
                writer.append_data(data)
                if i == args.trim_start:
                    for j in range(args.pause_start):
                        writer.append_data(data)
                elif i == len(filepaths) - args.trim_end - 1:
                    for j in range(args.pause_end):
                        writer.append_data(data)


    
    writer.close()
    print("wrote video {}".format(video_file))

if __name__ == "__main__":
    main()