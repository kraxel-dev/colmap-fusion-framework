# colmap-fusion-framework

Platform for colmap with multi-modal sensor fusion. Structure-from-Motion revisited, this time with more sensors to include in the graph-based ceres optimization.

# Build

## Dependencies and prerequisites

Build process is only tested on Ubuntu 20.04.

- colmap 3.11.1
- rerun_sdk 0.21.0 (c++)
- rerun_viewer 0.21.0 (pip or rust)

Except for the rerun_viewer, cloning and building these 3rd party packages are handled automatically by the main library cmake configuration.

## Build instructions

Building this repo should be straightforward.<br>

First make sure to apt install all colmap dependencies, listed here: <https://colmap.github.io/install.html#debian-ubuntu> (Do not proceed to clone, configure and compile COLMAP)

On toplevel root dir, create a build directory

```
# at root level of this repo
mkdir build
```

and source the cmake config

```
cd build
cmake ..
```

Running `cmake ..` for the first time will configure the fetching process of 3rd party repos. For this, the cmake script will exit early as the main library depends on the 3rd party libs that are still not build yet.<br>
After cmaking, run `make` to automatically fetch, build and locally install the 3rd party repos. Build and install paths are preconfigured and scoped locally within this repo, no need to worry about setting paths manually.

```
make -j4
```

Try `make` again if colmap decides to abort its build process in the middle.<br>
Once the thirdparty packages are build, source the cmake config again and proceed with `make`:

```
# again at toplevel build directory of main repo
cmake ..
make -j4
```

This should build the main library. Links to 3rd party dependencies are resolved automatically, no need to adjust any paths unless you want to point to your own custom build comlmap version.

Finally, apt-get the rerun_viewer apt dependencies listed here: <https://rerun.io/docs/getting-started/troubleshooting#running-on-linux> and <https://rerun.io/docs/getting-started/troubleshooting#wsl2> for a wsl2 setup. Then pip install the viewer through the rerun python sdk.

```
pip3 install --upgrade pip  # upgrade pip to find rerun python sdk for ubuntu 20.04
source ~/.bashrc  # source your bash after upgrading pip
pip3 install rerun-sdk==0.22.0  # rerun viewer is bundled in the python rerun-sdk
```

# Open Issues

## Scale and cost factors

1. between factor with scale as optimization param from glomap

## Reconstruction preproccessing

1. Validated filtering of 3d points with reconstruction bounding box

## Rerun

- add rerun graph view
- move rerun_interface to fusion helper to avoid circular dependencies with iteration callback

## Cost logging

1. Find a way to wrap loss function around cost in ResidualCostTracker
2. Eventually remove ceres_eval_utils completely, once newer ResidualCostTracker is validated to all use cases.

## OdomEdgesManager

1. Deal with multiple tums simultaneously
2. Deal with disconnected poses in tum file or disconnected image ids
3. Find better name
