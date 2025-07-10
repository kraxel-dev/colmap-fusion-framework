"""Create timestamp file (required by "SE3_Pose_Interp" python package) from a tum file. Use this ideally on the colmap cam trajectory tum,
as other sensors will be matched towards the images.

https://github.com/MichaelGrupp/evo/blob/9f77be90cad46abf76fbe3fe1baf3b39b86569c1/evo/core/trajectory.py#L393
https://github.com/rancheng/SE3_Pose_Interp/blob/master/gen_timestamp.py
https://github.com/rancheng/SE3_Pose_Interp/blob/master/data/timestamps.txt

Format of timestamps.txt file:
000001 1620666926.73657
000002 1620666926.76957
000003 1620666926.83557
... (more lines)
"""

from pathlib import Path
import sys, argparse
from evo.tools import file_interface

DESC = (
    "Create timestamp file (required by 'SE3_Pose_Interp' python package) from a tum file. "
    "Use this ideally on the colmap cam trajectory tum, as other sensors will be matched towards the images."
)


def parse_args():
    parser = argparse.ArgumentParser(description=DESC)
    parser.add_argument(
        "-i",
        "--input",
        required=True,
        help="file path to .tum containing the timestamps and absolute poses. Ideally the cam traj is used as tum, as other sensors match to the images",
    )
    args = parser.parse_args()
    return args


if __name__ == "__main__":

    args = parse_args()

    # full path of input result.txt
    tum = Path(args.input).resolve()

    # generate output tum file path
    out_stamps = tum.parent / "cam_timestamps.txt"
    print(f"Extracting stamps of {tum} and saving to {out_stamps}")

    # read tum file with evo interface
    traj = file_interface.read_tum_trajectory_file(tum)

    with open(out_stamps, "a") as ts_f:

        for idx, ts in enumerate(traj.timestamps):
            # zero padding infront of index
            index_str = "%06d" % idx
            # write idx and tstamp to txt
            line_txt = index_str + " " + str(ts) + "\n"
            ts_f.write(line_txt)

    print(f"Done!")
