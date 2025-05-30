import os, subprocess, yaml, subprocess, sys
from pathlib import Path


def get_git_root() -> Path:
    try:
        root = subprocess.check_output(["git", "rev-parse", "--show-toplevel"], text=True).strip()
        return Path(root)
    except subprocess.CalledProcessError:
        raise RuntimeError("This script must be run inside a Git repository.")


def get_curr_pyscript_dir() -> Path:
    # get the directory of the current script no matter where it is called from
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
        # subfolder to project ws path
        self.image_path = CmdArg("--image_path", ws_path / "images")
        self.database_path = CmdArg("--database_path", ws_path / f"db_{quality}_q.db")
        self.quality = CmdArg("--quality", quality)

        # --- camera params
        self.single_camera = CmdArg("--ImageReader.single_camera", True)
        self.camera_model = CmdArg("--ImageReader.camera_model", "PINHOLE")
        # fx, fy, cx, cy
        self.camera_params = CmdArg(
            "--ImageReader.camera_params",
            "501.4757919305817, 501.4757919305817, 421.7953735163109, 167.65799492501083",
        )

        # --- sequential matcher params
        self.matcher_use_gpu = CmdArg("--SiftMatching.use_gpu", 0)
        self.matcher_quadratic_overlap = CmdArg("--SequentialMatching.quadratic_overlap", 1)


def reconstruct_single_workspace(ws_path, model_quality="low"):
    """reconstruct a single colmap model for a given a workspace. Workspace must contain populated /images subfolder.
    quality settings {low, medium, high, extreme}
    """

    # --- check if images subfolder exists in project dir
    imgs_sub_dir = ws_path / "images"
    if not imgs_sub_dir.is_dir():
        print(
            f"Error: Subfolder '{imgs_sub_dir}' not found in '{ws_path}'. Make sure you have placed the /images folder into your project dir accordingly."
        )
        sys.exit(1)
    else:
        print(f"Images found for : {ws_path}")

    # --- prepare command line arguments for colmap reconstruction
    col_params = {}
    # paths stuff
    workspace_path = "--workspace_path"
    image_path = "--image_path"
    database_path = "--database_path"
    quality = "--quality"
    col_params[workspace_path] = ws_path
    col_params[image_path] = imgs_sub_dir  # subfolder to project ws path
    col_params[database_path] = ws_path / f"db_{model_quality}_q.db"
    col_params[quality] = model_quality

    # cam params
    single_camera = "--ImageReader.single_camera"
    camera_model = "--ImageReader.camera_model"
    camera_params = "--ImageReader.camera_params"
    col_params[single_camera] = 1
    col_params[camera_model] = "PINHOLE"
    col_params[camera_params] = (
        "501.4757919305817, 501.4757919305817, 421.7953735163109, 167.65799492501083"  # fx, fy, cx, cy
    )
    exe = str(get_generic_colmap_exe_path() / "./colmap")

    # --- Build the feature extraction command
    extractor = "feature_extractor"
    # fmt: off
    command = [
        exe, extractor, 
        database_path, str(col_params[database_path]), 
        image_path, str(col_params[image_path]), 
        single_camera, str(col_params[single_camera]), 
        camera_model, str(col_params[camera_model]), 
        camera_params, str(col_params[camera_params]),
    ]
    # fmt: on
    # --- Run the feature extractor command
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
        print(f"Failed to run feature extractor for {ws_path}.")
        if process.stdout:
            process.stdout.close()


def extract_features(cmd_args: ColmapCmdArgs):
    # --- check if images subfolder exists in project dir
    if not cmd_args.image_path.value.is_dir():
        print(
            f"Error: Subfolder '{cmd_args.image_path.value}' not found in '{cmd_args.workspace_path.value}'. Make sure you have placed the /images folder into your project dir accordingly."
        )
        sys.exit(1)
    else:
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
    # --- Run the feature extractor command
    run_cmd(command)


def sequentially_match_imgs(cmd_args: ColmapCmdArgs):
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
    run_cmd(command)


def run_cmd(command):
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
        print(f"Failed to run feature extractor for {ws_path}.")
        if process.stdout:
            process.stdout.close()


if __name__ == "__main__":
    # --- load yaml configuration file
    cfg_path = get_curr_pyscript_dir() / "multi_config.yaml"
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

        # reconstruct_single_workspace(ws_path, reconstruction_quality)
        # --- perform feature extraction on current workspace
        extract_features(col_args)

        # --- sequentially match images on curr ws
        sequentially_match_imgs(col_args)
        
