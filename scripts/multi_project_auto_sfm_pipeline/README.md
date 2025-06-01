Auto reconstruction pipeline to run colmap reconstruction on multiple projects at once (with shared param set)

# Workpsace structure
How each workspace/project you want to automatically reconstruct should be structured
```
.
├── images
│   ├── 1620666969669673472.png
│   ├── 1620666969702673664.png
│   ├── ...
├── mask  # optional 
│   ├── img_mask.png
```
After reconstruction the folder should look like this:
```
.
├── db.db
├── 0
│   ├── images.bin
│   ├── ...
├── images
│   ├── 1620666969669673472.png
│   ├── 1620666969702673664.png
│   ├── ...
├── mask  # optional 
│   ├── img_mask.png # if mask desired, name of mask-img must match exactly to this
```