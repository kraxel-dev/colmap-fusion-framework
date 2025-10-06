# Colmap Fusion Framework (Fuma)

COLMAP **3D reconstruction** **fused** with additional senor modalities (e.g. **external odometry**). **Structure-from-Motion** revisited; This time with more sensors and under the **factor graph formalism**.

> [!WARNING]
> This repo is still under heavy maintenance. Do not expect any warranty for usability, correctness, or code quality.

## Overview

This repo is a prototyping platform that enhances COLMAP's (vision-only) incremental mapping process with measurement factors from other sensors. Specifically, demos for fusing relative pose factors from external odometry are implemented. However, integration of other modalities (e.g. IMU pre-integration) is easily done, when following the instructions of this repo.

![showcase-fuma](docs/marketing/showcase-fuma-zoom-slow-downscaled.gif)

This platform was developed as part of my Master Thesis in cooperation with [Expleo](https://expleo.com/global/de/branchen/automotive/#overview) Germany's  <u>[ADAS department](https://expleo.com/global/en/insights/campaigns/autonomous-driving-adas/)</u> in Berlin. Main objective was a generalizable framework to easily obtain metric-scale (SfM) maps in which our demonstrator vehicle can re-localize itself using only a monocular camera. Speaking about vehicles and sensors, check out our awesome <u>[demonstrator vehicle](https://expleo.com/global/en/case-studies/automated-valet-parking/)</u>, which has been utilized to prototype and showcase many exciting applications in the domain of ADAS and AD.

## Table of Contents

- [Colmap Fusion Framework (Fuma)](#colmap-fusion-framework-fuma)
  - [Overview](#overview)
  - [Table of Contents](#table-of-contents)
  - [Features](#features)
    - [Variety of SfM and Fusion Helpers - Visualization and Ceres Stuff](#variety-of-sfm-and-fusion-helpers---visualization-and-ceres-stuff)
    - [High Level Fusion](#high-level-fusion)
    - [Tightly-coupled Fusion (Incremental Fusion Mapping)](#tightly-coupled-fusion-incremental-fusion-mapping)
    - [Samples for better understanding COLMAP's default reconstruction process](#samples-for-better-understanding-colmaps-default-reconstruction-process)
    - [Handy Scripts for Evaluation and Data Processing - Mix and Match](#handy-scripts-for-evaluation-and-data-processing---mix-and-match)
  - [Build](#build)
    - [Dependencies and prerequisites](#dependencies-and-prerequisites)
    - [Build instructions](#build-instructions)
  - [Usage and Samples](#usage-and-samples)
    - [High level fusion with external odometry](#high-level-fusion-with-external-odometry)
    - [Tightly-coupled Fusion with external odometry](#tightly-coupled-fusion-with-external-odometry)
  - [Prepare your own Data](#prepare-your-own-data)
  - [Open Issues](#open-issues)

## Features

### Variety of SfM and Fusion Helpers - Visualization and Ceres Stuff

1. Custom Rerun SfM Logger class -> [include/fusion_helper/rr_sfm_logger.h](include/fusion_helper/rr_sfm_logger.h):
   1. Stream your COLMAP model (cam poses and 3d points) directly to your rerun viewer
   2. Visualizes the odometry edges between your camera poses during fusion <img src="docs/marketing/showcase-odom-edge-annotated.png" width="50%">
   3. Deploy the logger into COLMAP's incremental reconstruction to see whats happening in between mapping <img alt="rerun stream colmap model" src="docs/marketing/showcase-rerun-model-streamer.gif" width="94%">
2. Ceres iteration callbacks -> [include/fusion_helper/fusion_iteration_callback.h](include/fusion_helper/fusion_iteration_callback.h):
   1. Stream your COLMAP model's bundle adjustment process directly to rerun viewer
   2. Highlight local and global BA during default incremental reconstruction in the rerun viewer
   3. Stream the fusion local and global BA (both high-level and tightly-coupled) to rerun viewer
3. A variety of cost functions
   1. Covariance weighted re-projection error
   2. Scale-aware relative pose factor (7-DoF)

### High Level Fusion

Originally used for familiarization with Ceres cost function concepts. Details under:

- <u>[High Level Fusion Module Description](docs/module_high_lvl_fusion.md)</u>

Fusion of fully reconstructed COLMAP models with relative pose constraints from corresponding external odometry data. Watch your unscaled COLMAP model grow to the true real-world scale through the fusion Bundle Adjustment process on your finalized model.

<img src="docs/marketing/showcase-high-level-growing-small-pts.gif" width="50%"><img src="docs/marketing/showcase-high-level-growing-front-small-pts.gif" width="50%">

### Tightly-coupled Fusion (Incremental Fusion Mapping)

The star of this repo. Details under:

- <u>[Tightly Coupled Fusion Module Description](docs/module_tightly_coupled_fusion.md)</u>

Fusion of relative pose constraints from external odometry data during COLMAP's incremental mapping from scratch:

TODO

### Samples for better understanding COLMAP's default reconstruction process

1. See what's happening in your vanilla incremental reconstruction through visualization during reconstruction in rerun.
2. Understand COLMAP's internal vanilla reconstruction steps through nice code samples and additional explanations.

### Handy Scripts for Evaluation and Data Processing - Mix and Match

TODO

1. Multi-project auto Reconstruction Pipeline (default COLMAP models)
   1. Automatically reconstruct COLMAP models for independent COLMAP projects (e.g. same image dataset under different conditions) at once.
2. Export cam poses in COLMAP model as tum trajectory (requires imgs name to be nsec timestamp before reconstruction)
   1. TODO
3. Prepare public dataset images for COLMAP reconstruction
   1. TODO

## Build

### Dependencies and prerequisites

Build process is only tested on Ubuntu 20.04.

- colmap 3.11.1 (will be cloned and build automatically by this repo)
- rerun_sdk 0.22.0 (c++)
- rerun_viewer 0.22.0 (pip or rust)

Except for the rerun_viewer, cloning and building these 3rd party packages are handled automatically by the main library cmake configuration.

### Build instructions

Make sure to apt install all colmap dependencies, listed here: <https://colmap.github.io/install.html#debian-ubuntu> (do not install COLMAP itself)

- Clone this repo
- On toplevel root dir of cloned repo, create a build directory, source the cmake config and execute make.

```bash
mkdir build && cd build  # at root level of this repo
cmake ..  # will exit early during first time cmaking to focus on the 3rd party dependencies
make -j4
```

> [!NOTE]
> Everything is fetched and installed locally (awesome)! Running `cmake ..` and `make` for the first time will fetch, build, and locally install the 3rd party repos (COLMAP + rerun SDK) automatically. Do not worry about stray install paths polluting your system; 3rd party build and install paths are scoped within this repo.

- If colmap decides to abort its build process in the middle, try `make` again .<br>

- After the 3rd party packages are build, `cmake ..` and `make` again for a 2nd time:

```bash
cmake ..  # again from build directory of main repo
make -j4
```

- This builds the main `colmap-fusion-framework` library. Links to 3rd party dependencies are resolved automatically, no need to adjust any paths unless you want to point to your own custom build comlmap version.

- Finally, `apt install` the `rerun_viewer` dependencies listed here: <https://rerun.io/docs/getting-started/troubleshooting#running-on-linux> and <https://rerun.io/docs/getting-started/troubleshooting#wsl2> for a wsl2 setup.

- Afterwards, pip install the `rerun viewer` through the rerun python sdk.

```bash
pip3 install --upgrade pip  # upgrade pip to find rerun python sdk for ubuntu 20.04
source ~/.bashrc  # source your bash after upgrading pip
pip3 install rerun-sdk==0.22.0  # rerun viewer is bundled in the python rerun-sdk
```

## Usage and Samples

Before running the executables for awesome fusion-aided 3D reconstruction, you need to prepare you default COLMAP database and external odometry tum file as described in [prepare-your-own-data](#prepare-your-own-data).

### High level fusion with external odometry

TODO

### Tightly-coupled Fusion with external odometry

TODO

## Prepare your own Data

TODO

## Open Issues

1. Scale and cost factors
   1. between factor with scale as optimization param from glomap

2. Scale and frame alignment
   1. Adapt strategy: scale estimation after n poses and only brute force afterwards to tackle mono sfm scale drift
   2. Use pca to align to ground-plane after n reg images

3. Reconstruction quality and filtering
   1. Test different quality presets for OptionsManager
   2. Validated filtering of 3d points with reconstruction bounding box

4. Rerun
   1. 3d points cropping bounding box needs to update during iter callbacks in high level fusion otherwise growing models will loose all points
   2. Insert pngs into pinhole plane
   3. color extraction
   4. add rerun graph view
   5. debug pose shift when setting campose as const ceresparam

5. Fusion Iteration Callback
   1. Merge marathon and vanilla fusion iter class

6. Cost logging
   1. Find a way to wrap loss function around cost in ResidualCostTracker
   2. Add residual tracking to tcf
   3. Eventually remove ceres_eval_utils completely, once newer ResidualCostTracker is validated to all use cases.

7. Incremental fusion mapper
   1. create superset of mapping options to control actions that belong to mapper and not to FusionBA object
   2. PCA alginment options is task of mapper
