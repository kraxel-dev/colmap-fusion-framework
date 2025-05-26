""" Forward colmap model (images.txt) to convert image poses to tum trajectory.
"""
import numpy as np
import pandas as pd
from pathlib import Path
import os
import csv, argparse

import math
from scipy.spatial.transform import Rotation as R
from numpy.linalg import inv

def build_transformation_matrix(r, t):
    t_matrix = np.eye(4)
    t_matrix[0:3,0:3] = r
    t_matrix[0:3,3] = t
    return t_matrix


def quat_xyzw_from_rotation_matrix(r):
    return R.from_matrix(r).as_quat()


def transformation_matrix_from_quat_xyzw(quat, t):
    r = rotation_matrix_from_quat_xyzw(quat)
    print(r)
    return build_transformation_matrix(r, t)


def rotation_matrix_from_quat_xyzw(quat):
    return R.from_quat(quat).as_matrix()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="Full path to images.txt created by colmap containing features and poses of the registered cameras.")
    # parser.add_argument("--output", required=True, help="Full file paht to output tum file (example: ~/$WORKSPACE/cam_trajectory.tum).")
    parser.add_argument("--lower_bound", type=int ,help="Ignore poses below this nsec integer timestamp. Useful in case you don't want to export the train images.")
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    image_txt_file = args.input
    # out_file = args.output
    lower_bound = args.lower_bound

    with open(image_txt_file) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=' ')
        line_count = -1

        i = 1
        df = pd.DataFrame(columns=("idx", "timestamp", "x", "y", "z" ,"q_x", "q_y", "q_z", "q_w"))
        for row in csv_reader:
            line_count += 1
            
            # skip header rows
            if line_count <= 3:
                continue
            
            # only take every 1st row that is pose -> skip second as second are all the feature points
            if line_count%2 == 0:
                print(row)

                # retrieve index
                idx = int(row[0])

                # Order of entries in images.txt file
                # IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME

                # retrieve quaternion
                quat = row[1:5]  
                quat.append(quat.pop(0))  # switch qx with qw

                # get positions entries
                pos = row[5:8]

                # construct pose to perform inversion
                pose = transformation_matrix_from_quat_xyzw(np.array(quat), np.array(pos))
                pose_inv = inv(pose)  # invert since original entries are world poses with respect to cameras

                # recover quaternion and position entries
                quat = quat_xyzw_from_rotation_matrix(pose_inv[0:3, 0:3])
                pos = pose_inv[0:3, 3]

                tstamp = []
                tstamp.append( Path(row[-1]).stem ) 

                # kick poses below timestamp thresh
                if lower_bound is not None:
                    if int(tstamp[0]) <= lower_bound:
                        # skip this pose
                        continue
                
                # convert nsecs to secs
                tstamp[0] = int(tstamp[0]) / 1e9
                
                # timestamp x y z q_x q_y q_z q_w
                tum_row = tstamp + list(pos) + list(quat)
                tum_row.insert(0, idx)
                print(tum_row)
                df.loc[i] = tum_row
                i+=1
                # writer.writerow(tum_row)
        
    print(df)
    df.sort_values('timestamp', ascending=True, inplace=True)
    print(df)
    df.drop('idx', inplace=True, axis=1)
    parent_dir = os.path.dirname(image_txt_file)
    outfile = os.path.join(parent_dir, "cam_trajectory.tum")
    print("Writing tum file to: ",outfile)
    df.to_csv(outfile, index=False, sep=' ', header=False)


if __name__ == "__main__":
    main()
