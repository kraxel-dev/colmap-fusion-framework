# Tightly-Coupled Fusion Module (tcf)

Fusion of relative pose constraints from external odometry data during COLMAP's active incremental mapping process. This module represents my thesis' main contribution. TODO: more infos.

<p align="center">
  <img src="./marketing/showcase-fuma-zoom-slow-downscaled.gif" alt="showcase-fuma" width="70%"/>
</p>

<p align="center">
  <img src="./marketing/showcase-fuma-active-mapping-frontal-dscaled.gif" alt="showcase-fuma-frontal" width="55%"/>
</p>

Table of Contents:

- [Tightly-Coupled Fusion Module (tcf)](#tightly-coupled-fusion-module-tcf)
  - [Features](#features)
  - [Implementation Structure](#implementation-structure)
  - [Samples and Usage](#samples-and-usage)
    - [Default Bundle Adjuster vs Fusion Bundle Adjuster](#default-bundle-adjuster-vs-fusion-bundle-adjuster)
    - [Default Incremental Mapper with Rerun Visualuzation](#default-incremental-mapper-with-rerun-visualuzation)
    - [Fusion Incremental Mapper (Fuma)](#fusion-incremental-mapper-fuma)

## Features

1. Fusion of relative pose constraints from external odometry data during COLMAP's active incremental mapping:

## Implementation Structure

The core tcf mapping engine was implemented by deriving classes from the original  COLMAP library (e.g. IncrementalMapper, BundleAdjuster) which are clustered in [src/tightly_coupled_fusion/*](../src/tightly_coupled_fusion) and [include/tightly_coupled_fusion/*](../include/tightly_coupled_fusion). This is why the folder and files in tcf mirror the internal file structure of vanilla COLMAP (locked to version 3.11.1). Note that in tcf, only the COLMAP folders show up that were actually used for deriving a tcf class.

```bash
# in tcf, only modules derived from original COLMAP show up
include/  # from $REPO_ROOT_DIR
├── fusion_helper
│   ├── ...
├── high_level_fusion
│   ├── ...
└── tightly_coupled_fusion  # mirroring original COLMAP repo structure
    ├── estimators
    │   └── bundle_adjustment.h  # derived classes for vanilla BA with rerun logging and BA with fusion of external odom
    ├── exe
    │   └── gui.h  # derived to run gui with fusion reconstruction. NOTE: not maintained, will be kicked before release
    ├── sfm
    │   └── incremental_mapper.h  # derived class for sequential mapping with rerun and tcf fusion mapping
    └── ui
        └── main_window.h  # derived to run gui with fusion reconstruction. NOTE: not maintained, will be kicked before release
 ```

The main structural change to the original COLMAP structure is that the Ceres cost-functions are located in the `fusion helper` module instead of the `estimators` dir in tcf.

```bash
include/  # from $REPO_ROOT_DIR
├── fusion_helper
│   ├── ...
│   ├── cost_functions.h  # Ceres cost-factors are in helper instead of tcf
│   ├── ...
├── high_level_fusion
│   ├── ...
└── tightly_coupled_fusion  # mirroring original COLMAP repo structure
    ├── ...

```

## Samples and Usage

> [!TIP] Prerequisites
>
> - As always, [prepare your own data](../README.md#prepare-your-own-data) before usage.
> - Check [how-to-rerun-viewer.md](how-to-rerun-viewer.md) to visualize the samples below.
> - All tcf executables are located in `$REPO_DIR/build/src/tightly_coupled_fusion/`

### Default Bundle Adjuster vs Fusion Bundle Adjuster

Refer to

- [src/tightly_coupled_fusion/samples/run_default_bundle_adjuster_rerun.cpp](../src/tightly_coupled_fusion/samples/run_default_bundle_adjuster_rerun.cpp)
- [src/tightly_coupled_fusion/samples/run_fusion_bundle_adjuster.cpp](../src/tightly_coupled_fusion/samples/run_fusion_bundle_adjuster.cpp)

for a detailed description of what both modules do.

TODO: cli commands

### Default Incremental Mapper with Rerun Visualuzation

Refer to [src/tightly_coupled_fusion/samples/run_incremental_mapper_rerun.cpp](../src/tightly_coupled_fusion/samples/run_incremental_mapper_rerun.cpp) for a detailed description

TODO: cli commands

### Fusion Incremental Mapper (Fuma)

Refer to [src/tightly_coupled_fusion/samples/run_fusion_mapper.cpp](../src/tightly_coupled_fusion/samples/run_fusion_mapper.cpp) for a detailed description

> [!NOTE]
> Below are the cli args to produce the results from my master thesis.

1. Navigate to `$REPO_DIR/build/src/tightly_coupled_fusion/` and execute:

```bash
./run_fusion_mapper \
--log_level 2 \
--db_path $DB \
--output_path $OUT \
--log_path $OUT \
--Fusion.tum_file $TUM \
--Fusion.is_mapping_with_fusion 1 \
--Fusion.fix_first_cam_pose 1 \
--Fusion.brute_force_scale_recovery 1 \
--Fusion.fusion_in_local_ba 1 \
--Fusion.fusion_in_global_ba 1 \
--FusionMapper.estimate_scale_on_init 1 \
--FrameAlign.align_first_cam_to_specific_pose 1 \
--FrameAlign.n_reg_for_alignment 12 \
--Rerun.log 1 \
--Rerun.save_rrd 1 \
--Rerun.rrd_path $OUT
```

- More information about the forwarded fusion parameters can be found at the respective options struct definition -> [include/tightly_coupled_fusion/estimators/bundle_adjustment.h](../include/tightly_coupled_fusion/estimators/bundle_adjustment.h)
