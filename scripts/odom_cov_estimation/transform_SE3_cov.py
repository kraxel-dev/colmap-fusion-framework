"""Given a EVO json transform from base to camera frame, rotate the std covariance matrix for a relative pose referring to vehicle base
the same cov but expressed for the camera sensor frame.

section: "Distribution of the inverse"
https://gtsam.org/2021/02/23/uncertainties-part3.html
"""

import argparse
from pathlib import Path

import evo.core.lie_algebra as lie
from evo.core import trajectory
from evo.core.metrics import Unit
from evo.tools import file_interface, log

import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def adjoint_SE3(R, t):
    Ad = np.zeros((6, 6))
    Ad[0:3, 0:3] = R
    Ad[0:3, 3:6] = skew(t) @ R
    Ad[3:6, 3:6] = R
    return Ad


def generate_std_cov_SE3():
    """Return hardcoded SE3 covariance matrix (6DoF RPE error per sec) with std values instead of variances.
    Paste your std values from EVO RPE results here.
    """
    std_cov = np.zeros((6, 6))
    # translation part
    std_cov[0, 0] = 0.040916161558996175
    std_cov[1, 1] = 0.010334600313526225
    std_cov[2, 2] = 0.004897247281516253
    # rotation part (angle axis rad)
    std_cov[3, 3] = 0.0018685288956326457
    std_cov[4, 4] = 0.0021703716136063913
    std_cov[5, 5] = 0.004712373554898492

    return std_cov


if __name__ == "__main__":

    # * please provide transform in EVO json format (used in --transform_right for evo_traj)
    # T_bc: cam pose expressed in vehicle base
    tf_file = "~/transfer/eval/left_optical_in_bl.json" #! TODO: generic file path
    T_bc = file_interface.load_transform(tf_file)  # cam pose in vehicle base frame

    # invert cause we need adjoint of T_cb
    T_cb = lie.se3_inverse(T_bc)

    R_cb = T_cb[:3, :3]  # rotation part
    t_cb = T_cb[:3, 3]  # translation part

    # compute adjoint of T_cb
    adj_T_cb = adjoint_SE3(R_cb, t_cb)

    # transform original covariance matrix (std) to the new frame
    # https://gtsam.org/2021/02/23/uncertainties-part3.html
    cov_b = generate_std_cov_SE3()  # covariance in base frame
    cov_c = adj_T_cb @ cov_b @ adj_T_cb.T  # covariance in camera frame

    # print results
    np.set_printoptions(precision=7, suppress=True)
    # np.set_printoptions( suppress=True)
    print("Original std CovMat (per secs) in baselink:\n", cov_b)
    print("Camera frame std CovMat (per secs) after transformation:\n", cov_c)
