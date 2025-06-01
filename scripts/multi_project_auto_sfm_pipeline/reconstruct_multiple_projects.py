import os, subprocess, yaml, subprocess, sys
from pathlib import Path


def get_git_root() -> Path:
    """Get the root directory of the Git repository this python script is lying in."""
    try:
        root = subprocess.check_output(["git", "rev-parse", "--show-toplevel"], text=True).strip()
        return Path(root)
    except subprocess.CalledProcessError:
        raise RuntimeError("This script must be run inside a Git repository.")


def get_curr_pyscript_dir() -> Path:
    """get the directory of the current script no matter where it is called from"""
    return Path(__file__).resolve().parent


def get_generic_colmap_exe_path() -> Path:
    # generic path to colmap exe given the root dir of this repo
    colmap_exe_path = "thirdparty/colmap_3.11.1_build/src/colmap/exe/./colmap"
    return get_git_root() / colmap_exe_path


class CmdArg:
    def __init__(self, arg_flag, arg_value):
        self.flag = arg_flag
        self.value = arg_value


class ColmapCmdArgs:
    def __init__(self, ws_path, quality="low"):
        self.workspace_path = CmdArg("--workspace_path", ws_path)
        self.output_path = CmdArg(
            "--output_path", ws_path
        )  # model output path is same as workspace/project path
        # subfolder to project ws path
        self.image_path = CmdArg("--image_path", ws_path / "images")
        self.database_path = CmdArg("--database_path", ws_path / f"db_{quality}_q.db")
        self.quality = CmdArg("--quality", quality)

        # --- camera and feature extratcion params
        self.single_camera = CmdArg("--ImageReader.single_camera", True)
        self.camera_model = CmdArg("--ImageReader.camera_model", "PINHOLE")
        # fx, fy, cx, cy
        self.camera_params = CmdArg(
            "--ImageReader.camera_params",
            "501.4757919305817, 501.4757919305817, 421.7953735163109, 167.65799492501083",
        )
        self.camera_mask_path = CmdArg("--ImageReader.camera_mask_path", ws_path / "mask/img_mask.png")

        # --- sequential matcher params
        self.matcher_use_gpu = CmdArg("--SiftMatching.use_gpu", 0)
        self.matcher_quadratic_overlap = CmdArg("--SequentialMatching.quadratic_overlap", 1)

        # --- Mapper params
        self.mapper_ba_refine_focal_length = CmdArg("--Mapper.ba_refine_focal_length", 0)
        self.mapper_ba_refine_principal_point = CmdArg("--Mapper.ba_refine_principal_point", 0)
        self.mapper_ba_refine_extra_params = CmdArg("--Mapper.ba_refine_extra_params", 0)
        self.mapper_ba_use_gpu = CmdArg("--Mapper.ba_use_gpu", 0)
        self.mapper_multiple_models = CmdArg("--Mapper.multiple_models", 1)


def extract_features(cmd_args: ColmapCmdArgs):
    # --- check if images subfolder exists in project dir
    if not cmd_args.image_path.value.is_dir():
        print(
            f"Error: Subfolder '{cmd_args.image_path.value}' not found in '{cmd_args.workspace_path.value}'. Make sure you have placed the /images folder into your project dir accordingly."
        )
        return
    
    print(f"Images found for : {cmd_args.workspace_path.value}")

    # generic path to colmap exe given the root dir of this repo
    col_exe = str(get_generic_colmap_exe_path())

    # --- Build the feature extraction command
    extractor = "feature_extractor"
    # fmt: off
    command = [
        col_exe, extractor, 
        cmd_args.database_path.flag,  str(cmd_args.database_path.value), 
        cmd_args.image_path.flag, str(cmd_args.image_path.value),
        cmd_args.single_camera.flag, str(cmd_args.single_camera.value),
        cmd_args.camera_model.flag, str(cmd_args.camera_model.value),
        cmd_args.camera_params.flag, str(cmd_args.camera_params.value),
    ]
    # fmt: on

    # --- Check for mask path and add it to the command if it exists
    if cmd_args.camera_mask_path.value.is_file():
        print(f"Camera mask img detected: {cmd_args.camera_mask_path.value}")
        command.append(cmd_args.camera_mask_path.flag)
        command.append(str(cmd_args.camera_mask_path.value))

    # --- Run the feature extractor command
    run_cmd(command, extractor)


def sequentially_match_imgs(cmd_args: ColmapCmdArgs):
    if not cmd_args.database_path.value.is_file():
        print(
            f"Error: Database '{cmd_args.database_path.value}' does not exist. Skipping sequential matching!"
        )
        return

    print(f"Database found for : {cmd_args.workspace_path.value}")
    col_exe = str(get_generic_colmap_exe_path())
    matcher = "sequential_matcher"

    # fmt: off
    command = [
        col_exe, matcher, 
        cmd_args.matcher_use_gpu.flag,  str(cmd_args.matcher_use_gpu.value), 
        cmd_args.matcher_quadratic_overlap.flag, str(cmd_args.matcher_quadratic_overlap.value),
        cmd_args.database_path.flag, str(cmd_args.database_path.value),
    ]
    # fmt: on
    run_cmd(command, matcher)


def reoncstruct_model(cmd_args: ColmapCmdArgs):
    if not cmd_args.database_path.value.is_file():
        print(
            f"Error: Database '{cmd_args.database_path.value}' does not exist. Skipping model reconstruction!"
        )
        return
    elif not cmd_args.image_path.value.is_dir():
        print(
            f"Error: Images folder '{cmd_args.image_path.value}' does not exist. Skipping model reconstruction!"
        )
        return
    
    # --- cancel reoncstr if model already exists
    if (cmd_args.output_path.value / "0").is_dir() and (cmd_args.output_path.value / "0" / "images.bin").is_file(): 
        # if images.bin exists and is not empty, model already exists
        if os.path.getsize(cmd_args.output_path.value / "0" / "images.bin") > 1000: # bytes
            print(f"Model already exists at {cmd_args.output_path.value}. Skipping reconstruction.")
            return

    col_exe = str(get_generic_colmap_exe_path())
    mapper = "mapper"

    # fmt: off
    command = [
        col_exe, mapper,
        cmd_args.database_path.flag, str(cmd_args.database_path.value),
        cmd_args.image_path.flag, str(cmd_args.image_path.value),
        cmd_args.output_path.flag, str(cmd_args.output_path.value),
        cmd_args.mapper_ba_refine_focal_length.flag, str(cmd_args.mapper_ba_refine_focal_length.value),
        cmd_args.mapper_ba_refine_principal_point.flag, str(cmd_args.mapper_ba_refine_principal_point.value),
        cmd_args.mapper_ba_refine_extra_params.flag, str(cmd_args.mapper_ba_refine_extra_params.value),
        cmd_args.mapper_ba_use_gpu.flag, str(cmd_args.mapper_ba_use_gpu.value),
        cmd_args.mapper_multiple_models.flag, str(cmd_args.mapper_multiple_models.value),
    ]
    # fmt: on
    run_cmd(command, mapper)


def run_cmd(command, type):
    print(f"Running command: {' '.join(command)}\nThis may take a while...\n")
    try:
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )
        # Print output line by line in real time
        if process.stdout:
            for line in process.stdout:
                print(line, end="")  # avoid adding extra newlines
        # Wait for the process to end
        return_code = process.wait()
        if return_code != 0:
            print(f"\n Command failed with return code {return_code}.")
    finally:
        print(f"Failed to run {type} for {ws_path}.")
        if process.stdout:
            process.stdout.close()


if __name__ == "__main__":

    # --- load yaml configuration file
    cfg_path = get_curr_pyscript_dir() / "multi_project_config.yaml"
    print(f"Loading configuration from {cfg_path}")
    with open(str(cfg_path), "r") as file:
        cfg = yaml.safe_load(file)

    reconstruction_quality = cfg["reconstruction_quality"]

    # --- iterate over all workspaces for reconstruction
    for ws in cfg["workspaces"]:
        ws_path = Path(cfg["workspaces"][ws]["ws_path"]).expanduser().resolve()
        print(f"Preparing workspace at path {ws_path}")

        # --- check if workspace path exists
        if not ws_path.is_dir():
            print(f"Workspace path {ws_path} does not exist. Skipping reconstruction.")
            continue

        # --- prepare command line arguments for colmap reconstruction
        col_args = ColmapCmdArgs(ws_path, reconstruction_quality)

        # --- perform feature extraction on current workspace
        extract_features(col_args)

        # --- sequentially match images on curr ws
        sequentially_match_imgs(col_args)

        # --- reconstruct model for current workspace
        reoncstruct_model(col_args)
