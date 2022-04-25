import argparse
import os

import mayavi.mlab as mlab
import numpy as np
import json
from tqdm import tqdm

cmap = np.array([
    [245, 150, 100, 255],
    [245, 230, 100, 255],
    [150, 60, 30, 255],
    [180, 30, 80, 255],
    [255, 0, 0, 255],
    [30, 30, 255, 255],
    [200, 40, 255, 255],
    [90, 30, 150, 255],
    [255, 0, 255, 255],
    [255, 150, 255, 255],
    [75, 0, 75, 255],
    [75, 0, 175, 255],
    [0, 200, 255, 255],
    [50, 120, 255, 255],
    [0, 175, 0, 255],
    [0, 60, 135, 255],
    [80, 240, 150, 255],
    [150, 240, 255, 255],
    [0, 0, 255, 255],
])
cmap = cmap[:, [2, 1, 0, 3]]  # convert bgra to rgba


def draw_lidar(pc,
               color=None,
               fig=None,
               bgcolor=(1, 1, 1),
               pts_scale=0.2,
               pts_mode='2dcircle',
               pts_color=None):
    if fig is None:
        fig = mlab.figure(figure=None,
                          bgcolor=bgcolor,
                          fgcolor=None,
                          engine=None,
                          size=(800, 500))
    if color is None:
        color = pc[:, 2]
    pts = mlab.points3d(pc[:, 0],
                        pc[:, 1],
                        pc[:, 2],
                        color,
                        mode=pts_mode,
                        scale_factor=pts_scale,
                        figure=fig)
    pts.glyph.scale_mode = 'scale_by_vector'
    pts.glyph.color_mode = 'color_by_scalar'  # Color by scalar
    pts.module_manager.scalar_lut_manager.lut.table = cmap
    pts.module_manager.scalar_lut_manager.lut.number_of_colors = cmap.shape[0]

    mlab.view(azimuth=270,
              elevation=54,
              focalpoint=[1100, 1200, 333],
              distance=1200,
              figure=fig)
    view = mlab.view()
    roll = mlab.roll()

    return fig


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--dir-name', type=str, help='directory name')
    parser.add_argument('--save-dir', type=str, help='save directory name')

    args, opts = parser.parse_known_args()

    files = os.listdir(args.dir_name)
    # files = files[1:3]
    plot_data = []
    for file in files:
        with open(args.dir_name + '/' + file, "r") as read_file:
            new_plot_data = json.load(read_file)
            new_plot_data[0]['name'] = new_plot_data[0]['name'][12:]
            plot_data.append(new_plot_data[0])

    sorted_plot_data = sorted(plot_data, key=lambda d: int(d['name']))

    for index, scan in tqdm(enumerate(sorted_plot_data)):
        if index > 10:
            break
        points = np.array(scan['points'])
        labels = np.array(scan['labels'])
        fig = draw_lidar(points, labels)
        vis_file_path = args.save_dir + '/' + str(index) + '.png'
        mlab.savefig(vis_file_path)
