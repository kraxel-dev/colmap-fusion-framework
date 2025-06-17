#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Clone folder of KITTI odom challange images and overwrite img names with their respective timestamps from times.txt file."""

import shutil
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
import numpy as np
import os
from pathlib import Path

DESC = (
    "Duplicate folder of KITTI (odom challange) images and overwrite img names with their respective timestamps from times.txt file. "
    "Output folder will be in same folder as input image dir under name: 'imgages'."
)


def rename_kitti_imgs_with_stamps(image_dir: Path, timestamp_file: Path, as_nsec: bool):

    # --- Checks image dir
    if not image_dir.exists():
        error_msg = f"Image dir {image_dir} does not exist! Abort"
        raise file_interface.FileInterfaceException(error_msg)

    # --- Create target dir
    # Create the target directory if it doesn't exist
    target_dir = image_dir.parent / "images"
    os.makedirs(target_dir, exist_ok=True)

    # --- Glob images
    imgs = sorted(image_dir.glob("*.png"))
    if len(imgs) <= 1:
        error_msg = f"Images in {image_dir} are empty! Abort"
        raise file_interface.FileInterfaceException(error_msg)

    # --- Check time stamp file matching
    raw_timestamps_mat = file_interface.csv_read_matrix(timestamp_file)
    error_msg = "timestamp file must have one column of timestamps and same number of rows as the KITTI images"
    if len(raw_timestamps_mat) > 0 and len(raw_timestamps_mat[0]) != 1:
        raise file_interface.FileInterfaceException(error_msg)

    error_msg = "Stamps in file does not equal the amoun of images. Abort!"
    if len(raw_timestamps_mat) != len(imgs):
        raise file_interface.FileInterfaceException(error_msg)

    try:
        timestamps_mat = np.array(raw_timestamps_mat).astype(float)
    except ValueError:
        raise file_interface.FileInterfaceException(error_msg)

    # --- Assign images stamp names and copy to new folder
    for i, (stamp, img_name) in enumerate(zip(raw_timestamps_mat, imgs)):
        # convert timestamp to img name
        matched_stamp = float(stamp[0])
        matched_name = str(matched_stamp) + ".png"
        # convert second to nsec
        if as_nsec:
            matched_stamp = int(matched_stamp * 1e9)  # no digits after decimal
            # left pad name with 0 so that there will always be 19 digits in the name
            epoch_digits = 19  # n numbers the unix epoch in nsec holds
            matched_name = str(matched_stamp).zfill(epoch_digits) + ".png"  

        img_file_src_path = image_dir / img_name
        img_file_dst_path = target_dir / matched_name

        # copy image with timestamp as name to new folder
        print(f"Copying {img_file_src_path} as {img_file_dst_path}")
        shutil.copy2(img_file_src_path, img_file_dst_path)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=DESC)
    parser.add_argument(
        "image_dir", help="dir path to folder containing KITTI image sequence"
    )
    parser.add_argument("timestamp_file", help="KITTI timestamp file of images")
    parser.add_argument(
        "--as_nsec",
        type=bool,
        default=1,
        help="1 or 0. whether to have img names as nsec or as seconds",
    )
    args = parser.parse_args()

    trajectory = rename_kitti_imgs_with_stamps(
        Path(args.image_dir).resolve(),
        Path(args.timestamp_file).resolve(),
        args.as_nsec,
    )
