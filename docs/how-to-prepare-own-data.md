# Prepare your own data

## Capturing your images

### Image names

Your image files must be named after their respective timestamp [nano-seconds] of capture.

```bash
# e.g.
1704810038020192671.png  # machine time in nsces since unix epoch
```

This image name will be parsed by the colmap-fusion-framework and attached to each image as timestamp. Make sure to double check your file names, if parsing fails due to a wrong name format, the framework will not be able to match images to their respective tum poses (see: [[#External odometry]]).

### Non-motion filter

Ideally, you drop images during capturing with no significant movement with respect to the previous captured image through a motion-filter. In a ros setup, this is easily achievable through a subscriber node that checks traveled distance through your odometry source (wheel or radar) and only saves a image to disk when enough distance is covered.

## Generate standard COLMAP feature database

TODO

## External odometry

The **external odometry source** that should provide the **relative pose constraints** between images in your colmap fusion reconstruction. Any source can be used as long as it provides **absolute** poses with respect to some static reference frame, for example:

- Wheel odometry
- Radar odometry
- GPS
- LiDAR odometry / SLAM
- Motion capture poses

### File format of external odometry

Your external odometry should be captured as a **tum file** [(Formats · MichaelGrupp/evo Wiki · GitHub)](https://github.com/MichaelGrupp/evo/wiki/Formats#tum---tum-rgb-d-dataset-trajectory-format) in **absolute poses** and with **timestamps** in regular **seconds** (not nano seconds).

Example format:

```csv
1744812914.509087 0.000000 0.000000 0.000000 -0.000000 0.000000 0.000000 1.000000
1744812917.976055 0.174400 0.000078 0.000000 -0.000000 0.000000 0.000292 1.000000
1744812918.375852 0.355191 0.000184 0.000000 -0.000000 0.000000 0.000115 1.000000
1744812918.675876 0.536792 0.000203 0.000000 -0.000000 0.000000 -0.000144 1.000000
1744812918.909401 0.718561 0.000158 0.000000 -0.000000 0.000000 0.000003 1.000000
1744812919.142552 0.925356 0.000078 0.000000 -0.000000 0.000000 -0.000245 1.000000
```

1. **Rosbag to tum file**: Converting your odometry from rosbag to tum file can be easily done with: [MichaelGrupp/evo · exporting-to-other-formats · GitHub](https://github.com/MichaelGrupp/evo/wiki/Formats#saving--exporting-to-other-formats)

### Coordinate frame convention of the odometry poses

#### Poses with respect to which frame?

The absolute poses of your external odometry sensors can be expressed with respect to **any** static reference frame as long as they all refer to the **same** static frame that stays consistent over time. This is because the absolute poses between iterations will be used to calculate **relative motion**, which only cares about the motion along its local body-frame (e.g. plain forward motion of a car is always along its local x-axis, no matter the absolute pose of the car in a map). The next part about expression of odom as camera poses is more important .

#### Odom must be expressed as camera poses - extrinsic calibration from odometry sensor to camera lense
>
> [!warning] Health warning.
> This section covers extrinsic calibration between sensor links.

This is the not-so-fun part. The absolute poses in your tum file **must** be the external odom sensor-link **expressed** as its respective **camera lens pose** (z-axis pointing forwards). Let's break this down:
...
You require the **extrinsic** calibration of the camera lens with respect to the local body-frame of the odometry sensor (static pose of **cam expressed in odom** sensor).

1. Given the correct extrinsics, you can **right-transform** your original tum file using [transformations · MichaelGrupp/evo Wiki](https://github.com/MichaelGrupp/evo/wiki/evo_traj#transformations) to obtain a new tum file (use arg: --save_as_tum to obtain the new one) that performs the following operation:
1. `poses of odometry sensor` --`right-transform`--> `poses of odometry sensor expressed as camera poses`
1. Alternatively, use your ros tf tree (assuming a ros setup and correctly populated tfs between all sensors) during data capture to fetch and save the pose of your odometry sensor already expressed as campose.

### Time stamp matching of external tum to captured images

Interpolate the external poses of your tum file to match the timestamps of the images within the COLMAP database. Timestamp digits after a certain decimal point number are cut to make internal association easier -> your stamps do not have to match in all 18 digits (see: [io.h](../include/fusion_helper/io.h) )

>
> [!warning] Differing Time units.
> The stamps of the external tum file should be in seconds while the image names are in nanoseconds. Account for this when you attempt the interpolation/matching

1. It is okay if the external odometry starts/stops earlier in time than the image sequence (or vice versa)
2. It is also okay if the external odometry source has lower (e.g. radar / lidar) or higher (wheel encoder) pose output frequency than the captured images. Not every image needs an external pose. What's important is that the poses that do match images must also match in stamp value (see above) for internal association.

### Covariance for odometry

Use evo's RPE metric to calculate the per-second-std of your external odometry w.r.t. a ground-truth trajectory around all six local pose axes. You can use this .py file: [../scripts/eval/evo_rpe_along_xyz.py](../scripts/eval/evo_rpe_along_xyz.py). Remember that you want to obtain the covariances w.r.t. the motion of the camera sensor, hence express gt and odom trajectory as camera frame before RPE estimation. Use these std values as cli arg for the fusion mapping binaries.
