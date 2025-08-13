"""EVO's Relative Pose Error (RPE) along XYZ axes independantly. Plot and save as EVO results for further analysis.

https://github.com/MichaelGrupp/evo/blob/master/evo/core/metrics.py#L158
https://github.com/MichaelGrupp/evo/blob/master/notebooks/metrics.py_API_Documentation.ipynb

"""

from pathlib import Path
import numpy as np
import argparse

from evo.tools import plot
import matplotlib.pyplot as plt

from evo.core import metrics

from evo.core.metrics import PoseRelation, PathPair, RPE
from evo.core.units import Unit, ANGLE_UNITS, LENGTH_UNITS, METER_SCALE_FACTORS

from evo.tools import file_interface
from evo.core import sync


class RPESingleAxis(RPE):
    """Deived Evo RPE Class to hold rel-pose error along each axis."""

    def __init__(
        self,
        pose_relation: PoseRelation = PoseRelation.translation_part,
        delta: float = 1,
        delta_unit: Unit = Unit.frames,
        rel_delta_tol: float = 0.1,
        all_pairs: bool = False,
        pairs_from_reference: bool = False,
        error_for_each_axes: bool = True,
    ):
        super().__init__(
            pose_relation,
            delta,
            delta_unit,
            rel_delta_tol,
            all_pairs,
            pairs_from_reference,
        )

        self.for_each_axes = error_for_each_axes
        self.error_x: np.array = None
        self.error_y: np.array = None
        self.error_z: np.array = None

    def process_data(self, data: PathPair):
        """Process the data and compute RPE along XYZ axis."""

        super().process_data(data)

        if self.for_each_axes:

            if not self.pose_relation in (
                PoseRelation.translation_part,
                PoseRelation.rotation_angle_rad,
                PoseRelation.rotation_angle_deg,
            ):
                print(
                    f"Warning: RPEAlongXYZ only supports translation and rotation angle relations, got {self.pose_relation}."
                )
                return

            print(f"Calculating error along xyz axes!")
            self.error_x = np.array([E_i[0, 3] for E_i in self.E])
            self.error_y = np.array([E_i[1, 3] for E_i in self.E])
            self.error_z = np.array([E_i[2, 3] for E_i in self.E])


def plot_rpe_along_axis(rpe_sax, seconds_from_start, axis="x"):
    """Plot the RPE along a specific axis.
    [evo/notebooks/metrics.py\_API\_Documentation.ipynb at master · MichaelGrupp/evo · GitHub](https://github.com/MichaelGrupp/evo/blob/master/notebooks/metrics.py_API_Documentation.ipynb)

    Args:
        rpe_sax RPE_metric: single axis RPE metric object with error and statistics
        seconds_from_start: calc from outside traj with reduced ids
        axis (str, optional): _description_. Defaults to "x".
    """

    stats = rpe_sax.get_all_statistics()

    x_label = f"$t$ (s)"
    y_axis_unit = f"RPE  ({rpe_sax.unit.value})"

    fig = plt.figure()
    plot.error_array(
        fig.gca(),
        rpe_sax.error,
        x_array=seconds_from_start[1:],
        statistics={s: v for s, v in stats.items() if s != "sse"},
        name=f"{axis} axis RPE",
        title=f"RPE along {axis} w.r.t. {rpe_metric.pose_relation.value} ({rpe_metric.unit.value}) per {delta} {delta_unit.value}",
        xlabel=x_label,
        ylabel=y_axis_unit,
    )


def prepare_rpe_results(
    rpe, traj_ref_reduced, traj_est_reduced, traj_est_name="odom", axis="x"
):
    """
    Helper function to fill results from a evo rpe object targeted for error in specific axes.
    traj must already be reduced to delta ids, meaning that each id is a (self-selected)
    delta step apart (e.g. 1 frame, 1 meter, etc.)

    https://github.com/MichaelGrupp/evo/blob/master/evo/core/metrics.py#L158

    Args:
        rpe: single axis RPE metric object with error and statistics
        traj_ref_reduced:
        traj_est_reduced:
        axis (str, optional): Defaults to "x".

    Returns:
        evo results:
    """
    traj_est_name_axis = traj_est_name + "_" + axis
    res = rpe.get_result("gt", traj_est_name_axis)

    res.add_trajectory("gt", traj_ref_reduced)
    res.add_trajectory(traj_est_name_axis, traj_est_reduced)

    seconds_from_start = np.array(
        [t - traj_est_reduced.timestamps[0] for t in traj_est_reduced.timestamps]
    )

    res.add_np_array("seconds_from_start", seconds_from_start[1:])
    res.add_np_array("timestamps", traj_est_plot.timestamps[1:])
    res.add_np_array("distances_from_start", traj_ref_reduced.distances[1:])
    res.add_np_array("distances", traj_est_reduced.distances[1:])

    res.info["title"] = (
        f"RPE w.r.t. {rpe.pose_relation.value} ({rpe.unit.value}) per {rpe.delta} {rpe.delta_unit.value}"
    )
    return res


def save_rpe_results(
    res_along_axis, parent_dir, axis="x", unit_str=Unit.meters.value
):
    """Save the RPE results for a specific axis such that it can be used by vanilla EVO res.

    Args:
        res_along_axis (_type_): RPE results along a specific axis
        parent_dir : directory where the results will be saved
        axis (str, optional): axis for which the results are saved. Defaults to "x".
    """
    file_interface.save_res_file(
        (parent_dir / f"res_{axis}_{unit_str}.zip"), res_along_axis
    )


DESC = (
    "EVO's Relative Pose Error (RPE) along XYZ axes independantly. Plot and save as EVO results for further analysis. Saves results in parent "
    "directory of the reference trajectory."
)


def parse_args():
    parser = argparse.ArgumentParser(description=DESC)
    parser.add_argument(
        "-r",
        "--ref",
        required=True,
        help="gt .tum trajectory",
    )
    parser.add_argument(
        "-e",
        "--est",
        required=True,
        help="Estimated trajectory .tum file to compare against the reference.",
    )
    args = parser.parse_args()
    return args


if __name__ == "__main__":

    # args = parse_args()

    # ref_file = "/home/azuo/transfer/eval/sq1in/lidar_traj_as_bl_sq1in_interp.tum"
    # est_file = "/home/azuo/transfer/eval/sq1in/radar_traj_as_bl_sq1in.tum"
    ref_file = "/home/azuo/transfer/eval/sq3in/lidar_traj_as_bl_sq3in.tum"
    est_file = "/home/azuo/transfer/eval/sq3in/radar_traj_as_bl_sq3in.tum"
    est_name = "radar_traj_as_bl_sq3in.tum"
    parent_dir = Path(ref_file).resolve().parent

    traj_ref = file_interface.read_tum_trajectory_file(ref_file)
    traj_est = file_interface.read_tum_trajectory_file(est_file)

    max_diff = 0.1
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)
    data = (traj_ref, traj_est)

    # --- RPE params
    pose_relation = metrics.PoseRelation.translation_part
    # pose_relation == PoseRelation.rotation_angle_rad

    # delta_unit = Unit.meters  # (frames, meters, deg, rad)
    delta_unit = Unit.frames  # (frames, meters, deg, rad)
    delta = 9  # calc RPE per x (frames, meters, deg, rad)

    # all pairs mode
    all_pairs = False  # activate

    # our custom flag to get error for each axis
    error_for_each_axes = True

    # --- calc RPE
    rpe_metric = RPESingleAxis(
        pose_relation=pose_relation,
        delta=delta,
        delta_unit=delta_unit,
        all_pairs=all_pairs,
        error_for_each_axes=error_for_each_axes,
    )

    rpe_metric.process_data(data)

    import copy

    # --- hack to get individual statistics for each axis
    rpe_x = copy.deepcopy(rpe_metric)
    rpe_y = copy.deepcopy(rpe_metric)
    rpe_z = copy.deepcopy(rpe_metric)
    rpe_x.error, rpe_y.error, rpe_z.error = (
        rpe_metric.error_x,
        rpe_metric.error_y,
        rpe_metric.error_z,
    )
    stats_x, stats_y, stats_z = (
        rpe_x.get_all_statistics(),
        rpe_y.get_all_statistics(),
        rpe_z.get_all_statistics(),
    )

    # --- reduced trajectory
    # important: restrict data to delta ids for plot
    # to only contain ids that are (user-chosen) delta apart
    traj_ref_plot = copy.deepcopy(traj_ref)
    traj_est_plot = copy.deepcopy(traj_est)
    delta_ids_with_first_pose = [0] + rpe_metric.delta_ids
    traj_ref_plot.reduce_to_ids(delta_ids_with_first_pose)
    traj_est_plot.reduce_to_ids(delta_ids_with_first_pose)
    seconds_from_start = np.array(
        [t - traj_est_plot.timestamps[0] for t in traj_est_plot.timestamps]
    )

    # --- plot error along XYZ axes

    plot_rpe_along_axis(rpe_x, seconds_from_start, axis="X")
    plot_rpe_along_axis(rpe_y, seconds_from_start, axis="Y")
    plot_rpe_along_axis(rpe_z, seconds_from_start, axis="Z")
    plt.show()

    # --- obtain results
    x_res = prepare_rpe_results(
        rpe_x, traj_ref_plot, traj_est_plot, traj_est_name=est_name, axis="x"
    )
    y_res = prepare_rpe_results(
        rpe_y, traj_ref_plot, traj_est_plot, traj_est_name=est_name, axis="y"
    )
    z_res = prepare_rpe_results(
        rpe_z, traj_ref_plot, traj_est_plot, traj_est_name=est_name, axis="z"
    )

    save_rpe_results(x_res, parent_dir, "x", unit_str=rpe_metric.unit.value)
    save_rpe_results(y_res, parent_dir, "y", unit_str=rpe_metric.unit.value)
    save_rpe_results(z_res, parent_dir, "z", unit_str=rpe_metric.unit.value)
