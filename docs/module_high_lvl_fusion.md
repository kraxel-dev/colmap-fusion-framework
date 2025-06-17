# High Level Fusion Module

This module was mainly written for:

1. Personal familiarization and debugging of factor-graph concepts.
2. Platform/playground to easily integrate and test additional factors (like IMU) before integrating it properly in tightly-coupled fusion.

## Features

1. Covariance weighted reproj cost-functor for arbitrary camera models (COLMAP 3.11.1 did not already implemented it)

## Samples and Usage

### Metric Odometry Bundle Adjustment

```bash
build/src/high_level_fusion/.metric_odom_bundle_adjust \
--log_level 2 \
--input_path $COLMAP_MODEL_DIR/ \
--output_path $COLMAP_MODEL_DIR/bundle \
--Fusion.tum_file $TUM_FILE_PATH \
--Fusion.cov 0.1 \
--Fusion.track_residuals 1 \
--Model.pre_crop_points 1 \
--Rerun.log 1 \
--Rerun.save_rrd 0 \
--Rerun.odom_as_pred 1 \
--BundleAdjustment.max_num_iterations 50 
```

Please refer to [metric_odom_bundle_adjust.cpp](../src/high_level_fusion/metric_odom_bundle_adjust.cpp) for more implementation details.
