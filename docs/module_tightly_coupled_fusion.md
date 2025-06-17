# Tightly Coupled Fusion Module

aka tcf

## Features

1. Fusion of relative pose constraints from external odometry data during COLMAP's incremental mapping from scratch:

## Implementation Structure

 All functionalities in tcf were implemented by deriving classes from the original main COLMAP library (e.g. IncrementalMapper, BundleAdjuster) and clustered in [src/tightly_coupled_fusion/*](../src/tightly_coupled_fusion) and [include/tightly_coupled_fusion/*](../include/tightly_coupled_fusion), which is why the folder and files in tcf mirror the internal file structure of vanilla COLMAP (locked to version 3.11.1). Note that in tcf only the COLMAP folders show up that were actually used for deriving a tcf class.

 ```bash
  # only modules derived from original colmap show up, not every colmap module
 include/tightly_coupled_fusion 
├── estimators
│   └── bundle_adjustment.h  # derived class for vanilla BA with rerun logging and BA with fusion of external odom
├── exe
│   └── gui.h  # derived to run gui with fusion reconstruction. NOTE: not mainted, will be kicked before release
├── sfm
│   └── incremental_mapper.h # derived class for sequential mapping with rerun and 
└── ui
    └── main_window.h  # derived to run gui with fusion reconstruction. NOTE: not mainted, will be kicked before release
 ```

## Samples and Usage
