# High Level Fusion Module

Fusion of fully reconstructed COLMAP models with relative pose constraints from corresponding external odometry data. Watch your unscaled COLMAP model grow to the true real-world scale through the fusion Bundle Adjustment process on your finalized model.

<img alt="high-lvl-showcase" src="./marketing/showcase-high-level-growing-small-pts.gif" width="90%">

<img alt="high-lvl-front-showcase" src="./marketing/showcase-high-level-growing-front-small-pts.gif" width="90%">

This module was mainly written for:

1. Personal familiarization and debugging of factor-graph concepts.
2. Platform/playground to easily integrate and test additional factors (like IMU) before integrating it properly in tightly-coupled fusion.

## Features

1. Covariance weighted reproj cost-functor for arbitrary camera models (COLMAP 3.11.1 did not already implemented it)

## Samples and Usage

- As always, [prepare your own data](../README.md#prepare-your-own-data) before usage.
- Check [how-to-rerun-viewer.md](how-to-rerun-viewer.md) for using the Rerun viewer for the samples.

### Metric Odometry Bundle Adjustment

Please refer to [metric_odom_bundle_adjust.cpp](../src/high_level_fusion/metric_odom_bundle_adjust.cpp) for more implementation details.

1. Exe is found in repo's build folder:

```bash
build/src/high_level_fusion/.metric_odom_bundle_adjust \
--log_level 2 \
--input_path $COLMAP_MODEL_DIR/ \
--output_path $COLMAP_MODEL_DIR/bundle \
--Fusion.tum_file $TUM_FILE_PATH \
--Fusion.track_residuals 0 \
--BundleAdjustment.max_num_iterations 100 \
--Rerun.log 1 \
--Rerun.odom_as_pred 1 \
--Rerun.save_rrd 0 \
--Rerun.ignore_out_of_bbox_pts 0
```

Options you can play around with are below. Use `/.metric_odom_bundle_adjust -h` to see more available options

```bash
# play around with covariance values for each rel pose edge
--OdomCov.tx_std 0.2 \
--OdomCov.ty_std 0.2 \
--OdomCov.tz_std 0.2 \
--OdomCov.rx_std 0.2 \
--OdomCov.ry_std 0.2 \
--OdomCov.rz_std 0.2 \
# crops 3D points in rerun that are beyond the COLMAP model BB. 
--Rerun.ignore_out_of_bbox_pts 1
# draw odom edges as absolute pose trajectory. does not change fusion behavior
--Rerun.odom_as_pred 0 
```
