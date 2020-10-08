import csv
import glob
import math
import os

import matplotlib.pyplot as plt
import numpy as np
import quaternion  # noqa


def deg2rad(degrees):
    return degrees * math.pi / 180.0


def rad2deg(rads):
    return rads / math.pi * 180.0


def axis_angle_to_quat(ax, ay, az, angle):
    s = math.sin(angle / 2)
    return np.quaternion(math.cos(angle / 2), ax * s, ay * s, az * s)


def conj(q):
    return np.quaternion(q.w, -q.x, -q.y, -q.z)


def angle_between(q1, q2):
    if q1.w < 0:
        q1 = -q1
    if q2.w < 0:
        q2 = -q2
    product = q1 * conj(q2)
    if product.w < 0:
        product = -product
    product.w = min(product.w, 1.0)
    return 2 * math.acos(product.w)


def get_sqlite_filepaths_from_directory(directory):
    """Returns a list of filepaths."""
    filepaths = []
    os.chdir(directory)
    for filepath in glob.glob("*.sqlite"):
        filepaths.append(filepath)
    return filepaths


def append_to_dataset(filepath, position_dataset, rotation_dataset):

    with open(filepath) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=",")
        for row in csv_reader:

            num_scalars_per_object = 7
            if row[-1] == " ":
                row.pop()

            assert len(row) % num_scalars_per_object == 0
            for i in range(0, len(row), num_scalars_per_object):
                # add Vector3 position as tuple
                position_dataset.append(
                    np.array([float(row[i + 0]), float(row[i + 1]), float(row[i + 2])])
                )

                # np.quaternion expects w,x,y,z. File data is x,y,z,w.
                rotation_dataset.append(
                    np.quaternion(
                        float(row[i + 6]),
                        float(row[i + 3]),
                        float(row[i + 4]),
                        float(row[i + 5]),
                    )
                )


def main():

    comparisons = [
        (
            "sanity check: compare start data",
            [
                "start_Fedora_bullet287debug_*rep0*",
                "start_Fedora_bullet289debug_*rep0*",
                "start_Fedora_bulletpackagerelease_*rep0*",
                "start_Mac_bulletpackagerelease_*rep0*",
                "start_Ubuntu_bulletpackagerelease_*rep0*",
                "start_Fedora_bullet287debug_*rep1*",
                "start_Fedora_bullet289debug_*rep1*",
                "start_Fedora_bulletpackagerelease_*rep1*",
                "start_Mac_bulletpackagerelease_*rep1*",
                "start_Ubuntu_bulletpackagerelease_*rep1*",
                "start_Fedora_bullet287debug_*rep2*",
                "start_Fedora_bullet289debug_*rep2*",
                "start_Fedora_bulletpackagerelease_*rep2*",
                "start_Mac_bulletpackagerelease_*rep2*",
                "start_Ubuntu_bulletpackagerelease_*rep2*",
            ],
        ),
        ("sanity check: compare all end data to itself", ["end*", "end*"]),
        # ("sanity check: compare Mac data to itself (2x)",
        # ["end*Mac*",
        # "end*Mac*"]),
        # ("sanity check: compare Ubuntu data to itself (2x)",
        # ["end*Ubuntu*",
        # "end*Ubuntu*"]),
        # ("sanity check: compare all data to itself (3x)",
        # ["end*",
        # "end*",
        # "end*"]),
        (
            "sanity check: Fedora, compare cube start and end data",
            [
                "start_Fedora_bulletpackagerelease_*cubeSolid*",
                "end_Fedora_bulletpackagerelease_*cubeSolid*",
            ],
        ),
        (
            "sanity check: Fedora, compare sphere start and end data",
            [
                "start_Fedora_bulletpackagerelease_*uvSphereSolid*",
                "end_Fedora_bulletpackagerelease_*uvSphereSolid*",
            ],
        ),
        # ("sanity check: Ubuntu, compare some sphere start and end data",
        # ["start_Ubuntu_bulletpackagerelease_uvSphereSolid*",
        # "end_Ubuntu_bulletpackagerelease_uvSphereSolid*"]),
        # ("sanity check: Mac, compare some sphere start and end data",
        # ["start_Mac_bulletpackagerelease_uvSphereSolid*",
        # "end_Mac_bulletpackagerelease_uvSphereSolid*"]),
        # ("sanity check: Fedora, compare some cube start and end data",
        # ["start_Fedora_bulletpackagerelease_cubeSolid*",
        # "end_Fedora_bulletpackagerelease_cubeSolid*"]),
        # ("sanity check: Ubuntu, compare some cube start and end data",
        # ["start_Ubuntu_bulletpackagerelease_cubeSolid*",
        # "end_Ubuntu_bulletpackagerelease_cubeSolid*"]),
        # ("sanity check: Mac, compare some cube start and end data",
        # ["start_Mac_bulletpackagerelease_cubeSolid*",
        # "end_Mac_bulletpackagerelease_cubeSolid*"]),
        # ("sanity check: compare different start data",
        # ["start_Fedora_bullet287debug_banana_rep0*",
        # "start_Fedora_bullet287debug_coneSolid_rep0*",
        # "start_Fedora_bullet287debug_cubeSolid_rep0*"]),
        (
            "Fedora, compare two executions of the same build",
            [
                "end*Fedora_bulletpackagerelease*",
                "end*Fedora_run2_bulletpackagerelease*",
            ],
        ),
        (
            "Fedora, compare Bullet 2.87 debug vs 2.87 release",
            ["end*Fedora_bullet287debug*", "end*Fedora_bulletpackagerelease*"],
        ),
        (
            "compare Fedora Bullet 2.87 debug vs Ubuntu Bullet 2.87 debug",
            ["end*Fedora_bulletpackagerelease*", "end*Ubuntu_bulletpackagerelease*"],
        ),
        (
            "compare Fedora Bullet 2.89 debug vs Mac 2.89 release",
            ["end*Fedora_bullet289debug*", "end*Mac_bulletpackagerelease*"],
        ),
        (
            "Fedora, compare Bullet 2.87 debug vs 2.89 debug",
            ["end*Fedora_bullet287debug*", "end*Fedora_bullet289debug*"],
        ),
        (
            "Fedora, compare repetitions w/out restart",
            [
                "end*Fedora_bulletpackagerelease*rep0*",
                "end*Fedora_bulletpackagerelease*rep1*",
                "end*Fedora_bulletpackagerelease*rep2*",
            ],
        ),
        (
            "Ubuntu, compare repetitions w/out restart",
            [
                "end*Ubuntu_bulletpackagerelease*rep0*",
                "end*Ubuntu_bulletpackagerelease*rep1*",
                "end*Ubuntu_bulletpackagerelease*rep2*",
            ],
        ),
        (
            "Mac, compare repetitions w/out restart",
            [
                "end*Mac_bulletpackagerelease*rep0*",
                "end*Mac_bulletpackagerelease*rep1*",
                "end*Mac_bulletpackagerelease*rep2*",
            ],
        ),
    ]

    figCount = 0
    do_save = False
    for (comparison_title, comparison_glob_list) in comparisons:

        print("comparison: {}, {}".format(comparison_title, comparison_glob_list))

        comparison_filepaths = []
        is_valid_comparison = True
        for glob_expr in comparison_glob_list:

            filepaths = []
            for filepath in glob.glob(glob_expr):
                filepaths.append(filepath)
            filepaths.sort()

            if len(comparison_filepaths) and (
                len(comparison_filepaths[0]) != len(filepaths)
            ):
                print(
                    "invalid comparison; len({}) == {} and len({}) == {}".format(
                        comparison_glob_list[0],
                        len(comparison_filepaths[0]),
                        glob_expr,
                        len(filepaths),
                    )
                )
                is_valid_comparison = False
                break

            comparison_filepaths.append(filepaths)

        if not is_valid_comparison:
            continue

        num_datasets = len(comparison_filepaths)
        print("num_datasets: {}".format(num_datasets))

        # show file matchups
        if False:
            for j in range(len(comparison_filepaths[0])):
                for i in range(num_datasets):
                    print("{}\t".format(comparison_filepaths[i][j]), end="")
                print("")
            print("")

        position_datasets = []
        rotation_datasets = []
        for i in range(num_datasets):

            position_dataset = []
            rotation_dataset = []
            # print("dataset {}: {}".format(i, comparison_filepaths[i]))
            for filepath in comparison_filepaths[i]:
                append_to_dataset(filepath, position_dataset, rotation_dataset)

            if len(position_datasets) and (
                len(position_datasets[0]) != len(position_dataset)
            ):
                print(
                    "invalid comparison; len(dataset {}) == {} and len(dataset {}) == {}".format(
                        0, len(position_datasets[0]), i, len(position_dataset)
                    )
                )
                is_valid_comparison = False

            position_datasets.append(position_dataset)
            rotation_datasets.append(rotation_dataset)

        print("len(dataset 0): {}".format(len(position_datasets[0])))
        # print("position_datasets: {}".format(position_datasets))
        print("")
        print("")

        dataset_size = len(position_datasets[0])

        if False:
            average_positions = []
            for i in range(dataset_size):

                sum_position = np.array([0.0, 0.0, 0.0])
                for j in range(num_datasets):
                    sum_position += position_datasets[j][i]
                avg_position = sum_position / num_datasets
                average_positions.append(avg_position)

        gt_positions = position_datasets[0]
        gt_rotations = rotation_datasets[0]

        position_errors_all_datasets = []
        rotation_errors_all_datasets = []
        num_position_exact_matches = 0
        num_rotation_exact_matches = 0
        for j in range(1, num_datasets):

            for i in range(dataset_size):
                position_error = np.linalg.norm(
                    gt_positions[i] - position_datasets[j][i]
                )
                position_errors_all_datasets.append(position_error)
                if np.array_equal(gt_positions[i], position_datasets[j][i]):
                    num_position_exact_matches += 1

                rotation_error = rad2deg(
                    angle_between(gt_rotations[i], rotation_datasets[j][i])
                )
                rotation_errors_all_datasets.append(rotation_error)
                if gt_rotations[i] == rotation_datasets[j][i]:
                    num_rotation_exact_matches += 1

        position_exact_match_fraction = num_position_exact_matches / (
            (num_datasets - 1) * dataset_size
        )
        rotation_exact_match_fraction = num_rotation_exact_matches / (
            (num_datasets - 1) * dataset_size
        )

        # plot histogram of position errors
        fig = plt.figure(figsize=(8, 8))
        n, bins, patches = plt.hist(position_errors_all_datasets, 50)
        plt.title(comparison_title + ", position")
        plt.yscale("log")
        plt.xlabel("distance (meters), deviation from run #0's position")
        highlight_percentile = 95
        highlight_x = np.percentile(position_errors_all_datasets, highlight_percentile)
        ax = plt.axes()
        if highlight_x > 0.01:
            plt.axvline(x=highlight_x, color="red")
            plt.text(
                1.0,
                1.0,
                "{}th percentile".format(highlight_percentile),
                color="red",
                transform=ax.transAxes,
                horizontalalignment="right",
                verticalalignment="top",
            )
        plt.text(
            0.0,
            1.0,
            "{:,.1f}% exact match".format(position_exact_match_fraction * 100),
            color="green",
            transform=ax.transAxes,
            horizontalalignment="left",
            verticalalignment="top",
        )
        if do_save:
            plt.savefig("fig{}.png".format(figCount), dpi=fig.dpi)
            figCount += 1
        else:
            plt.show()

        # plot histogram of rotation errors
        fig = plt.figure(figsize=(8, 8))
        n, bins, patches = plt.hist(rotation_errors_all_datasets, 50)
        plt.title(comparison_title + ", rotation")
        plt.yscale("log")
        plt.xlabel("distance (degrees), deviation from run #0's rotation")
        highlight_percentile = 95
        highlight_x = np.percentile(rotation_errors_all_datasets, highlight_percentile)
        if highlight_x > 0.01:
            plt.axvline(x=highlight_x, color="red")
            ax = plt.axes()
            plt.text(
                1.0,
                1.0,
                "{}th percentile".format(highlight_percentile),
                color="red",
                transform=ax.transAxes,
                horizontalalignment="right",
                verticalalignment="top",
            )
        plt.text(
            0.0,
            1.0,
            "{:,.1f}% exact match".format(rotation_exact_match_fraction * 100),
            color="green",
            transform=ax.transAxes,
            horizontalalignment="left",
            verticalalignment="top",
        )
        if do_save:
            plt.savefig("fig{}.png".format(figCount), dpi=fig.dpi)
            figCount += 1
        else:
            plt.show()


if __name__ == "__main__":
    main()
