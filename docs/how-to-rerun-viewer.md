# Streaming Mapping Data to Rerun Viewer

## References

- [Navigating the viewer — Rerun](https://rerun.io/docs/getting-started/navigating-the-viewer#launching-an-example)

## Rerun Fusion Mapping User Guide

Streaming visualization data to your rerun viewer is mostly identical for the executables in both `high level` and `tightly-coupled` fusion.

Before running any of the fusion executables listed in [module_tightly_coupled_fusion.md](module_tightly_coupled_fusion.md) or [module_high_lvl_fusion.md](module_high_lvl_fusion.md), you have to open an instance of the rerun viewer.

### Open rerun viewer before running this projects's binaries

- Either stream to your rerun web browser tab or:
- In your Linux terminal, open a rerun viewer instance:

```bash
rerun --renderer vulkan  
```

- The `--renderer vulkan` flag is for a `wsl2` setup, if not required simply omit it
- After the viewer started, you can run your desired executable with the `--Rerun.log 1` flag toggled

### Write the data stream to disk

For saving the data stream directly to disk, toggle the `--Rerun.save_rrd 1` flag for your executable and set the path `--Rerun.rrd_path $OUT_PATH` to your desired location. No need to open the viewer for this.

- The recording will be saved as `recording.rrd`

> [!WARNING]
> When saving a recording to disk, the rerun viewer does not receive any data to display. You have to remove the `save_rrd` flag again for live visualization.

- You can inspect a saved recording with your viewer by

```bash
rerun --renderer vulkan  $$OUT_PATH/recording.rrd
```

### General Visualization Options

 TODO

- 3D points crop
