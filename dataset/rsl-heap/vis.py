import open3d.ml.torch as ml3d
import argparse
import numpy as np
import json


def main() -> None:
    """"This script will visualize a point cloud with lables in the format specified by ml3d.
        More specifically it is currently hardcoded to assign the subset of categories that was
        selected by the authors of the SPVNAS paper."""

    parser = argparse.ArgumentParser()
    parser.add_argument('--name', type=str, help='file name')

    args, opts = parser.parse_known_args()

    with open(args.name, "r") as read_file:
        plot_data = json.load(read_file)

    lut = ml3d.vis.LabelLUT()
    lut.add_label('car', 0)
    lut.add_label('bicycle', 1)
    lut.add_label('motorcycle', 2)
    lut.add_label('truck', 3)
    lut.add_label('other-vehicle', 4)
    lut.add_label('person', 5)
    lut.add_label('bicyclist', 6)
    lut.add_label('motorcyclist', 7)
    lut.add_label('road', 8)
    lut.add_label('parking', 9)
    lut.add_label('sidewalk', 10)
    lut.add_label('other-ground', 11)
    lut.add_label('building', 12)
    lut.add_label('fence', 13)
    lut.add_label('vegetation', 14)
    lut.add_label('trunk', 15)
    lut.add_label('terrain', 16)
    lut.add_label('pole', 17)
    lut.add_label('traffic-sign', 18)


    vis = ml3d.vis.Visualizer()

    vis.set_lut("lables", lut)
    vis.visualize(plot_data)



if __name__ == '__main__':
    main()
