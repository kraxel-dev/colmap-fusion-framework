"""convert tum 4 seasons result file (fused stereo inertial with gps) to tum fromat

Format of result.txt file:
1620666926.73657    0.175145085582259  -0.0590170758191425 0.000739583329611696 0.0289748655862279 0.722055810591313 -0.691219254258318 0.00343600031392951
1620666926.76957   0.173043704946294  -0.196915095481955 -0.0099508384451797 0.0291422346259211 0.722154231289278 -0.691110478468514 0.0032098197801758
1620666926.83557   0.170397594828241  -0.468823761578322 -0.0319316614196918 0.0295160559834833 0.722682999639825 -0.690543814596542 0.00270639857042758
... (more lines)
"""

from pathlib import Path
import sys, argparse


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i",
        "--input",
        required=True,
        help="file path to results.txt (fused pose file) of 4 seasons dataset sequence.",
    )
    args = parser.parse_args()
    return args


def normalize_spaces(input_file, output_file):
    with open(input_file, "r") as fin, open(output_file, "w") as fout:
        for line in fin:
            # Strip leading/trailing whitespace and split by any whitespace, then join by single space
            normalized = " ".join(line.strip().split())
            fout.write(normalized + "\n")


if __name__ == "__main__":

    args = parse_args()

    # full path of input result.txt
    input_txt = Path(args.input).resolve()

    # generate output tum file path
    out_tum_file = input_txt.parent / "odom.tum"

    print(f"Converting poses of {input_txt} to TUM format and saving to {out_tum_file}")
    normalize_spaces(input_txt, out_tum_file)
