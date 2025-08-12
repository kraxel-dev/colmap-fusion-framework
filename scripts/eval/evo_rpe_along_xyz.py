"""_summary_
[evo/evo/core/metrics.py at master · MichaelGrupp/evo · GitHub](https://github.com/MichaelGrupp/evo/blob/master/evo/core/metrics.py#L158)
[evo/notebooks/metrics.py\_API\_Documentation.ipynb at master · MichaelGrupp/evo · GitHub](https://github.com/MichaelGrupp/evo/blob/master/notebooks/metrics.py_API_Documentation.ipynb)

"""

import numpy as np

from evo.tools import plot
import matplotlib.pyplot as plt

from evo.core import metrics

from evo.core.metrics import PoseRelation, PathPair, RPE
from evo.core.units import Unit, ANGLE_UNITS, LENGTH_UNITS, METER_SCALE_FACTORS

from evo.tools import file_interface
from evo.core import sync


class RPEAlongXYZ(RPE):
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


if __name__ == "__main__":

    ref_file = "/home/azuo/transfer/eval/sq1in/lidar_traj_as_bl_sq1in_interp.tum"
    est_file = "/home/azuo/transfer/eval/sq1in/radar_traj_as_bl_sq1in.tum"

    traj_ref = file_interface.read_tum_trajectory_file(ref_file)
    traj_est = file_interface.read_tum_trajectory_file(est_file)

    max_diff = 0.01
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)
    data = (traj_ref, traj_est)

    # --- RPE params
    pose_relation = metrics.PoseRelation.translation_part

    delta = 1  # calc RPE per 1 meter
    # delta_unit = Unit.meters  # (frames, meters, deg, rad)
    delta_unit = Unit.meters  # (frames, meters, deg, rad)
    # all pairs mode
    all_pairs = False  # activate

    # our custom flag to get error for each axis
    error_for_each_axes = True

    # --- calc RPE
    rpe_metric = RPEAlongXYZ(
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

    # --- trajectory plots
    traj_ref_plot = copy.deepcopy(traj_ref)
    traj_est_plot = copy.deepcopy(traj_est)
    delta_ids_with_first_pose = [0] + rpe_metric.delta_ids
    traj_ref_plot.reduce_to_ids(delta_ids_with_first_pose)
    traj_est_plot.reduce_to_ids(delta_ids_with_first_pose)
    seconds_from_start = [
        t - traj_est.timestamps[0] for t in traj_est.timestamps
    ]

    # --- plot error along XYZ axes
    if delta_unit is Unit.frames:
        si_unit = "s"
        x_label = f"$t$ ({si_unit})"
        x_steps = seconds_from_start
    elif delta_unit is Unit.meters:
        si_unit = "m"
        x_label = f"$path$ ({si_unit})"
        x_steps = np.arange(0, len(rpe_metric.error_x) * delta, delta)
    else:
        x_label = "$t$ undetermined unit"

    fig = plt.figure()
    plot.error_array(
        fig.gca(),
        rpe_metric.error_x,
        x_array=x_steps,
        statistics={s: v for s, v in stats_x.items() if s != "sse"},
        name="X axis RPE",
        title="RPE w.r.t. " + rpe_metric.pose_relation.value + " along X per " + str(delta) + si_unit,
        xlabel=x_label,
    )
    fig = plt.figure()
    plot.error_array(
        fig.gca(),
        rpe_metric.error_y,
        x_array=x_steps,
        statistics={s: v for s, v in stats_y.items() if s != "sse"},
        name="Y axis RPE",
        title="RPE w.r.t. " + rpe_metric.pose_relation.value + " along Y per " + str(delta)+ si_unit,
        xlabel=x_label,
    )
    fig = plt.figure()
    plot.error_array(
        fig.gca(),
        rpe_metric.error_z,
        x_array=x_steps,
        statistics={s: v for s, v in stats_z.items() if s != "sse"},
        name="X axis RPE ",
        title="RPE w.r.t. " + rpe_metric.pose_relation.value + " along Z per " + str(delta)+ si_unit,
        xlabel=x_label,
    )
    plt.show()

    # --- obtain results
    x_res = rpe_x.get_result("lidar_traj", "radar_traj_x")
    y_res = rpe_y.get_result("lidar_traj", "radar_traj_y")
    z_res = rpe_z.get_result("lidar_traj", "radar_traj_z")

    x_res.add_trajectory("lidar_traj", traj_ref_plot)
    x_res.add_trajectory("radar_traj_x", traj_est_plot)
    y_res.add_trajectory("lidar_traj", traj_ref_plot)
    y_res.add_trajectory("radar_traj_y", traj_est_plot)
    z_res.add_trajectory("lidar_traj", traj_ref_plot)
    z_res.add_trajectory("radar_traj_z", traj_est_plot)

    x_res.info["title"] = "title"
    x_res.add_np_array("seconds_from_start", seconds_from_start[1:])
    x_res.add_np_array("timestamps", traj_est_plot.timestamps[1:])
    x_res.add_np_array("distances_from_start", traj_ref_plot.distances[1:])
    x_res.add_np_array("distances", traj_est_plot.distances[1:])
    y_res.info["title"] = "title"
    y_res.add_np_array("seconds_from_start", seconds_from_start[1:])
    y_res.add_np_array("timestamps", traj_est_plot.timestamps[1:])
    y_res.add_np_array("distances_from_start", traj_ref_plot.distances[1:])
    y_res.add_np_array("distances", traj_est_plot.distances[1:])
    z_res.info["title"] = "title"
    z_res.add_np_array("seconds_from_start", seconds_from_start[1:])
    z_res.add_np_array("timestamps", traj_est_plot.timestamps[1:])
    z_res.add_np_array("distances_from_start", traj_ref_plot.distances[1:])
    z_res.add_np_array("distances", traj_est_plot.distances[1:])

    file_interface.save_res_file("/home/azuo/transfer/eval/sq1in/results/res_x.zip", x_res)
    file_interface.save_res_file("/home/azuo/transfer/eval/sq1in/results/res_y.zip", y_res)
    file_interface.save_res_file("/home/azuo/transfer/eval/sq1in/results/res_z.zip", z_res)
