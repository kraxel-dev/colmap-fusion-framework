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
  - [Features and Concept](#features-and-concept)
  - [Implementation Structure](#implementation-structure)
  - [Samples and Usage](#samples-and-usage)
    - [Default Bundle Adjuster vs Fusion Bundle Adjuster](#default-bundle-adjuster-vs-fusion-bundle-adjuster)
    - [Default Incremental Mapper with Rerun Visualization](#default-incremental-mapper-with-rerun-visualization)
    - [Fusion Incremental Mapper (Fuma)](#fusion-incremental-mapper-fuma)

## Features and Concept

Fusion of relative pose constraints from external odometry data during COLMAP's active incremental mapping:
<p align="center">
  <span style="background-color: white; padding: 10px; display: inline-block; border-radius: 8px;">
   <img src="marketing/pipeline-fusion-mapping-implemented.png" width="65%">
</p>

- Figure 1: Pipeline overview of this works implemented incremental fusion mapping system. This overview was adapted from the original concept figure of the [COLMAP](https://github.com/colmap/colmap) framework ~ (Schönberger & Frahm, "Structure-from-Motion Revisited," CVPR 2016)

Concepts:

- Odometry sources (TBD)
- 6DoF Relative Pose Costfactor as odom edges (TBD)
- Local Bundle Adjustment vs Global Bundle Adjustment (TBD)
- Fusion Bundle Adjustment with odom edges between images (TBD)
- Brute-force vs Estimation-based Scale recovery for monocular COLMAP using fusion (TBD)

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
> - [Prepare your own data](./how-to-prepare-own-data.md) before usage.
> - Check [how-to-rerun-viewer.md](how-to-rerun-viewer.md) to open rr viewer before mapping to visualize the samples below.
> - Navigate to `$REPO_DIR/build/src/tightly_coupled_fusion/` to run tcf executables.
> - Check [Fusion mapping CLI args](how-to-cli-options.md) for more details on the cli args

Example ENV exports to set COLMAP db and paths:

```bash
# example setup
export DB=/path/db_low_q.db \  # standard COLMAP db after ft-extraction and sequential matching
export OUT=/path/fuma \        # path for resulting COLMAP model 
export TUM=/path/radar_traj_as_campose_matched.tum  # time match external odom expressed as camera poses
```

### Default Bundle Adjuster vs Fusion Bundle Adjuster

Run default or fusion bundle adjuster on an already fully reconstructed model as sanity check. Rerun logging helps with visualization. Each iteration of the internal Ceres optimization process is logged to the rerun viewer.

Refer to doxygen @brief at the top of the files:

- [src/tightly_coupled_fusion/samples/run_default_bundle_adjuster_rerun.cpp](../src/tightly_coupled_fusion/samples/run_default_bundle_adjuster_rerun.cpp)
- [src/tightly_coupled_fusion/samples/run_fusion_bundle_adjuster.cpp](../src/tightly_coupled_fusion/samples/run_fusion_bundle_adjuster.cpp)

for a detailed description of what both modules do.

Example execution

```bash
# takes in same cli args as the original BundleAdjuster exe from original COLMAP repo
./run_default_bundle_adjuster_rerun -h
```

```bash
./run_fusion_bundle_adjuster -h
```

TODO: actual cli command for running

### Default Incremental Mapper with Rerun Visualization

This script reflects the original COLMAP incremental mapping behavior, stripped from its strict initialization routines and other safety measures. It can be used to understand the original mapping steps in a simplified manner as well as drastically speed up a reconstruction process by reducing the number of registered images that trigger a local BA. Rerun visualization helps to debug important steps like init triangulation, local BA and global BA influence, etc.

Refer to doxygen @brief at the top of the file [src/tightly_coupled_fusion/samples/run_incremental_mapper_rerun.cpp](../src/tightly_coupled_fusion/samples/run_incremental_mapper_rerun.cpp) for more detailed description.

Example execution:

```bash
./run_incremental_mapper_rerun \
--db_path $DB \
--output_path $OUT \
--log_level 3 \
--Mapper.ba_global_max_refinements 2 \
--Mapper.ba_local_max_refinements 2 \
--time_diff_local_ba 0.3 \
--Mapper.ba_global_max_num_iterations 15 \
--Mapper.ba_local_max_num_iterations 15 \
--Mapper.ba_local_num_images 6 \
--FrameAlign.n_reg_for_alignment 24 \
--Init.n_init_pair_skip 2
```

To see all options:

```bash
./run_incremental_mapper_rerun -h
```


### Fusion Incremental Mapper (Fuma)

This script performs COLMAP's incremental mapping process with fusion of relative pose constraints from external odometry data during active reconstruction (tightly-coupled manner). The external pose constraints are fused in the local and global BA steps of the ongoing mapping process.

Refer to doxygen @brief at the top of the file [src/tightly_coupled_fusion/samples/run_fusion_mapper.cpp](../src/tightly_coupled_fusion/samples/run_fusion_mapper.cpp) for a detailed description.

> [!NOTE]
> Below are the cli args to produce the results from my master thesis.

Navigate to `$REPO_DIR/build/src/tightly_coupled_fusion/` and run:

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

To see all options:

```bash
./run_fusion_mapper -h
```

For explanation on fusion options:

- Check [COLMAP fusion cli options](how-to-cli-options.md) for explanations of the params for fusion.
