"""Create timestamp file (required by "SE3_Pose_Interp" python package) from a tum file. Use this ideally on the colmap cam trajectory tum,
as other sensors will be matched towards the images. Can upsample original timestamps by a specified factor. This easens downstream evaluation
tasks given slow ground truth data.

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
    parser.add_argument(
        "-u",
        "--upsample",
        type=int,
        default=3,
        help="Number of interpolation points between true stamps (default: 3). If 0 or negative, no upsampling is performed.",
    )
    args = parser.parse_args()
    return args


if __name__ == "__main__":

    args = parse_args()

    # full path of input result.txt
    tum = Path(args.input).resolve()

    # generate output tum file path
    out_stamps = tum.parent / "upscaled_stamps.txt"
    print(f"Extracting stamps of {tum} and saving to {out_stamps}")

    # read tum file with evo interface
    traj = file_interface.read_tum_trajectory_file(tum)

    orig_timestamps = traj.timestamps
    upsample_n = args.upsample
    upsampled_timestamps = []
    # upsample timestamps if requested
    if upsample_n and upsample_n > 0:
        for i in range(len(orig_timestamps) - 1):
            t0 = orig_timestamps[i]
            t1 = orig_timestamps[i + 1]
            upsampled_timestamps.append(t0)
            for j in range(1, upsample_n + 1):
                interp = t0 + (t1 - t0) * (j / (upsample_n + 1))
                upsampled_timestamps.append(interp)
        if len(orig_timestamps) > 2:
            upsampled_timestamps.append(orig_timestamps[-1])
        print(f"Upsampling enabled: {upsample_n} points between each true stamp.")
    else:
        upsampled_timestamps = list(orig_timestamps)
        print("No upsampling performed (upsample param <= 0).")

    with open(out_stamps, "w") as ts_f:
        for idx, ts in enumerate(upsampled_timestamps):
            index_str = "%06d" % idx
            line_txt = index_str + " " + str(ts) + "\n"
            ts_f.write(line_txt)

    print(f"Done! Output {len(upsampled_timestamps)} timestamps.")
