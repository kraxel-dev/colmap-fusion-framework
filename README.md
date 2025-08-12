# Colmap Fusion Framework

Prototyping platform for COLMAP 3D reconstruction enhanced with multi-modal sensor fusion. Structure-from-Motion revisited, this time with more sensors to include in the ceres Bundle Adjustment process.

**DISCLAIMER** This repo is still under heavy maintenance. Do not expect any warranty for usability, correctness or code quality.

## Features

### Variety of SfM and Fusion Helpers

1. Custom Rerun SfM Logger class -> [include/fusion_helper/rr_sfm_logger.h](include/fusion_helper/rr_sfm_logger.h):
   1. Stream your COLMAP model (cam poses and 3d points) directly to your rerun viewer
   2. Visualizes the odometry edges between your camera poses during fusion
   3. Deploy the logger into COLMAP's incremental reconstruction to see whats happening in between mapping
2. Ceres iteration callbacks -> [include/fusion_helper/fusion_iteration_callback.h](include/fusion_helper/fusion_iteration_callback.h):
   1. Stream your COLMAP model's bundle adjustment process directly to rerun viewer
   2. Highlight local and global BA during incremental reconstruction in the rerun viewer
   3. Stream the fusion BA process (both high-level and tightly-coupled) to rerun viewer
3. A variety of cost functions
   1. Covariance Weighted Reprojection Error

### High Level Fusion

1. [High Level Fusion Module Description](docs/module_high_lvl_fusion.md)

Fusion of fully reconstructed COLMAP models with relative pose constraints from corresponding external odometry data:
![showcase-high-lvl](docs/marketing/high_lvl_fusion_showcase.gif)

### Tightly-coupled Fusion

1. [Tightly Coupled Fusion Module Description](docs/module_tightly_coupled_fusion.md)

Fusion of relative pose constraints from external odometry data during COLMAP's incremental mapping from scratch:

### Rerun Visualization

### Samples and resources for a better understanding of COLMAP's internal reconstruction process

1. See what's happening in your vanilla incremental reconstruction through visualization during reconstruction in rerun.
2. Understand COLMAP's internal vanilla reconstruction steps through nice code samples and additional explanations.

## Build

### Dependencies and prerequisites

Build process is only tested on Ubuntu 20.04.

- colmap 3.11.1 (will be cloned and build automatically by this repo)
- rerun_sdk 0.22.0 (c++)
- rerun_viewer 0.22.0 (pip or rust)

Except for the rerun_viewer, cloning and building these 3rd party packages are handled automatically by the main library cmake configuration.

### Build instructions

Make sure to apt install all colmap dependencies, listed here: <https://colmap.github.io/install.html#debian-ubuntu> (do not install COLMAP itself)

On toplevel root dir of repo, create a build directory, source the cmake config and execute make.

```bash
mkdir build && cd build  # at root level of this repo
cmake ..  # will exit early during first time cmaking to focus on the 3rd party dependencies
make -j4
```

Running `cmake ..` and `make` for the first time will fetch, build and locally install the 3rd party repos (COLMAP + rerun SDK) automatically. Do not worry about stray install paths polluting your system, 3rd party build and install paths are scoped locally within this repo. If colmap decides to abort its build process in the middle, try `make` again .<br>

After the 3rd party packages are build, `cmake ..` and `make` again for a 2nd time:

```bash
cmake ..  # again from build directory of main repo
make -j4
```

This builds the main `colmap-fusion-framework` library. Links to 3rd party dependencies are resolved automatically, no need to adjust any paths unless you want to point to your own custom build comlmap version.

Finally, `apt install` the `rerun_viewer` dependencies listed here: <https://rerun.io/docs/getting-started/troubleshooting#running-on-linux> and <https://rerun.io/docs/getting-started/troubleshooting#wsl2> for a wsl2 setup.

Afterwards, pip install the `rerun viewer` through the rerun python sdk.

```bash
pip3 install --upgrade pip  # upgrade pip to find rerun python sdk for ubuntu 20.04
source ~/.bashrc  # source your bash after upgrading pip
pip3 install rerun-sdk==0.22.0  # rerun viewer is bundled in the python rerun-sdk
```

## Usage and Samples

### High level fusion with external odometry

## Open Issues

### Scale and cost factors

1. between factor with scale as optimization param from glomap

### Scale and frame alignment

1. Adapt strategy: scale estimation after n poses and only brute force afterwards to tackle mono sfm scale drift
2. Use pca to align to ground-plane after n reg images

### Reconstruction quality and filtering

1. Test different quality presets for OptionsManager
2. Validated filtering of 3d points with reconstruction bounding box

### Rerun

1. 3d points cropping bounding box needs to update during iter callbacks in high level fusion otherwise growing models will loose all points
2. Insert pngs into pinhole plane
3. color extraction
4. add rerun graph view
5. debug pose shift when setting campose as const ceresparam

### Fusion Iteration Callback

1. Merge marathon and vanilla fusion iter class

### Cost logging

1. Find a way to wrap loss function around cost in ResidualCostTracker
2. Add residual tracking to tcf
3. Eventually remove ceres_eval_utils completely, once newer ResidualCostTracker is validated to all use cases.

### OdomEdgesManager

1. Decide if edges are created outside of mapper and delete setter method accordingly
2. Deal with multiple tums simultaneously
3. Deal with disconnected poses in tum file or disconnected image ids
4. Find better class name

### Incremental fusion mapper

1. create superset of mapping options to control actions that belong to mapper and not to FusionBA object

1. PCA alginment options is task of mapper

### Auto eval pipeline

Automatically reconstruct multiple colmap models for same image dataset under different conditions.
