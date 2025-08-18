"""Reconstruct multiple independent projects with COLMAP given a .yaml of specified folders. Each folder needs to contain an /images subfolder. That's basically it."""

import argparse
import os, subprocess, yaml, subprocess, sys, configparser
from pathlib import Path


def get_git_root() -> Path:
    """Get the root directory of the Git repository this python script is lying in."""
    try:
        root = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"], text=True
        ).strip()
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
    def __init__(self, ws_path, db_name_extension=""):
        self.workspace_path = CmdArg("--workspace_path", ws_path)
        self.output_path = CmdArg(
            "--output_path", ws_path
        )  # model output path is same as workspace/project path
        # subfolder to project ws path
        self.image_path = CmdArg("--image_path", ws_path / "images")
        self.database_path = CmdArg(
            "--database_path", ws_path / f"db_{db_name_extension}.db"
        ) #! TODO: choose db name from yaml file
        self.camera_mask_path = CmdArg(
            "--ImageReader.camera_mask_path", ws_path / "mask/img_mask.png")
        # NOTE : everything below is depracated and should be set in the project.ini file
        # self.quality = CmdArg("--quality", db_name_extension)

        # --- camera and feature extratcion params
        self.single_camera = CmdArg("--ImageReader.single_camera", True)
        self.camera_model = CmdArg("--ImageReader.camera_model", "PINHOLE")
        # fx, fy, cx, cy
        self.camera_params = CmdArg(
            "--ImageReader.camera_params",
            "501.4757919305817, 501.4757919305817, 421.7953735163109, 167.65799492501083",
        )

        # --- sequential matcher params
        self.matcher_use_gpu = CmdArg("--SiftMatching.use_gpu", 0)
        self.matcher_quadratic_overlap = CmdArg(
            "--SequentialMatching.quadratic_overlap", 1
        )

        # --- Mapper params
        self.mapper_ba_refine_focal_length = CmdArg(
            "--Mapper.ba_refine_focal_length", 0
        )
        self.mapper_ba_refine_principal_point = CmdArg(
            "--Mapper.ba_refine_principal_point", 0
        )
        self.mapper_ba_refine_extra_params = CmdArg(
            "--Mapper.ba_refine_extra_params", 0
        )
        self.mapper_ba_use_gpu = CmdArg("--Mapper.ba_use_gpu", 0)
        self.mapper_multiple_models = CmdArg("--Mapper.multiple_models", 1)


def extract_features(cmd_args: ColmapCmdArgs, ini_config: dict):
    """Execute colmap cli command to extract features from the images in the given workspace and database.

    Args:
        cmd_args (ColmapCmdArgs): cmd args containing image and database paths
        ini_config (dict): colmap params from project.ini file containing all other reconstructino related settings
    """
    # --- check if images subfolder exists in project dir
    if not cmd_args.image_path.value.is_dir():
        print(
            f"Error: Subfolder '{cmd_args.image_path.value}' not found in '{cmd_args.workspace_path.value}'. Make sure you have placed the /images folder into your project dir accordingly."
        )
        return

    print(f"Images found for : {cmd_args.workspace_path.value}")

    # generic path to colmap exe given the root dir of this repo
    col_exe = str(get_generic_colmap_exe_path())

    # --- Build the feature extraction cli command
    extractor = "feature_extractor"

    # define input and output paths from our own yaml
    # fmt: off
    command = [
        col_exe, extractor, 
        cmd_args.database_path.flag,  str(cmd_args.database_path.value), 
        cmd_args.image_path.flag, str(cmd_args.image_path.value),
    ]
    # fmt: on

    # pipe rest of the parameters from project.ini file into the exe command
    image_reader = "ImageReader"
    sift_extratcion = "SiftExtraction"
    command = extend_cmd_args_with_ini(command, ini_config, image_reader)
    command = extend_cmd_args_with_ini(command, ini_config, sift_extratcion)

    # --- Check for mask path and add it to the command if it exists
    if cmd_args.camera_mask_path.value.is_file():
        print(f"Camera mask img detected: {cmd_args.camera_mask_path.value}")
        command.append(cmd_args.camera_mask_path.flag)
        command.append(str(cmd_args.camera_mask_path.value))

    # --- Run the feature extractor command
    run_cmd(command, extractor)


def sequentially_match_imgs(cmd_args: ColmapCmdArgs, ini_config: dict):
    """Execute colmap cli command to sequentially match the images in the given workspace and database.

    Args:
        cmd_args (ColmapCmdArgs): cmd args containing image and database paths
        ini_config (dict): colmap params from project.ini file containing all other reconstructino related settings

    """
    if not cmd_args.database_path.value.is_file():
        print(
            f"Error: Database '{cmd_args.database_path.value}' does not exist. Skipping sequential matching!"
        )
        return

    print(f"Database found for : {cmd_args.workspace_path.value}")
    col_exe = str(get_generic_colmap_exe_path())
    matcher = "sequential_matcher"

    # start build cmd arg for runnug matcher and define database paths from our own yaml
    # fmt: off
    command = [
        col_exe, matcher, 
        cmd_args.database_path.flag, str(cmd_args.database_path.value),
    ]
    # fmt: on

    # pipe rest of the parameters from project.ini file into the exe command
    TwoViewGeometry = "TwoViewGeometry"
    SiftMatching = "SiftMatching"
    SequentialMatching = "SequentialMatching"
    command = extend_cmd_args_with_ini(command, ini_config, TwoViewGeometry)
    command = extend_cmd_args_with_ini(command, ini_config, SiftMatching)
    command = extend_cmd_args_with_ini(command, ini_config, SequentialMatching)
    run_cmd(command, matcher)


def reoncstruct_model(cmd_args: ColmapCmdArgs, ini_config: dict):
    """Execute cli command to reconstruct the model using COLMAP's mapper module.

    Args:
        cmd_args (ColmapCmdArgs): cmd args containing image and database paths
        ini_config (dict): colmap params from project.ini file containing all other reconstructino related settings

    """
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
    if (cmd_args.output_path.value / "0").is_dir() and (
        cmd_args.output_path.value / "0" / "images.bin"
    ).is_file():
        # if images.bin exists and is not empty, model already exists
        if (
            os.path.getsize(cmd_args.output_path.value / "0" / "images.bin") > 1000
        ):  # bytes
            print(
                f"Model already exists at {cmd_args.output_path.value}. Skipping reconstruction."
            )
            return

    # col_exe = str(get_generic_colmap_exe_path())
    col_exe = str(get_generic_colmap_exe_path())
    mapper = "mapper"

    # build cli command for mapper and provide input, database and output paths from our own yaml
    # fmt: off
    command = [
        col_exe, mapper,
        "--log_level", "0",
        cmd_args.database_path.flag, str(cmd_args.database_path.value),
        cmd_args.image_path.flag, str(cmd_args.image_path.value),
        cmd_args.output_path.flag, str(cmd_args.output_path.value),
    ]
    # fmt: on

    # pipe rest of the parameters from project.ini file into the exe command
    Mapper = "Mapper"
    command = extend_cmd_args_with_ini(command, ini_config, Mapper)
    run_cmd(command, mapper)


def is_blacklisted(cmd_flag: str) -> bool:
    """Check if a command line flag is blacklisted based on manually defined hitman list."""
    # "--ImageReader.camera_mask_path" should not be parsed from project.ini file as it is generically handled for each ws through this python script
    blacklist = {
        "--ImageReader.camera_mask_path",
    }
    return cmd_flag in blacklist


def extend_cmd_args_with_ini(command: list, ini_config: dict, col_module: str):
    """Extend the colmap command line args for running a specific module (e.g. ./colmap feature_extractor --Flag FlagValue) with parameters from the project.ini file."""
    # iterate over all flags for colmap ini category (e.g. ImageReader, SequentialMatching, Mapper)
    for key, value in ini_config[col_module].items():
        # extent the cli command with the flag and value
        flag = f"--{col_module}.{key}"

        # check handcrafted list if cmd flag from project.ini should be skipped
        if is_blacklisted(flag):
            print(f"Skipping blacklisted flag: {flag}")
            continue

        command.append(flag)
        command.append(value)

    return command


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


def parse_args():
    DESC = (
        "Reconstruct multiple independent projects with COLMAP given a .yaml of specified folders. "
        "Each folder needs to contain an /images subfolder. That's basically it."
    )

    parser = argparse.ArgumentParser(
        DESC, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "-y",
        "--yaml",
        type=str,
        default="./multi_project_config.yaml",
        help="Relative path to yaml config telling which folders to perform reconstruction and which .ini to use as COLMAP params.",
    )
    args = parser.parse_args()
    return args


if __name__ == "__main__":

    # --- CLI
    args = parse_args()

    # --- load yaml configuration file
    # yaml mainly contains the paths to the workspaces so that the actual settings from parms.ini can be applied generically to all workspaces
    cfg_path = (get_curr_pyscript_dir() / args.yaml).expanduser().resolve()
    if not cfg_path.suffix.lower() in (".yaml", ".yml"):
        print(f"Multi project .yaml config {cfg_path} does not exist. Exiting!")
        sys.exit(1)

    print(f"Loading configuration from {cfg_path}")
    with open(str(cfg_path), "r") as file:
        cfg = yaml.safe_load(file)

    # string with which created db files will be extend with
    db_name_ext = cfg["db_name_extension"]
    # COLMAP params
    project_ini_file = (
        (get_curr_pyscript_dir() / cfg["project_ini"]).expanduser().resolve()
    )

    # --- parse project.ini param file
    # Contains actual settings that will be applied to each reconstruction equally
    print(f"Loading project parameters from {project_ini_file}")
    if not project_ini_file.is_file():
        print(f"Project parameters file {project_ini_file} does not exist. Exiting.")
        sys.exit(1)

    ini_config = configparser.ConfigParser()
    ini_config.read(project_ini_file)
    # convert ini config to dict for easier access
    ini_dict = {
        section: dict(ini_config.items(section)) for section in ini_config.sections()
    }  # dict containing colmap params from project.ini

    
    # --- obtain absolute path of all workspaces chosen for reconstruction
    abs_ws_paths = []
    for ws in cfg["workspaces"]:
        # --- obtain absolute workspace path from ws name in yaml config
        ws_path = Path(cfg["workspaces"][ws]["ws_path"]).expanduser().resolve()
        print(f"Preparing workspace at path {ws_path}")

        # --- check if workspace path exists
        if not ws_path.is_dir():
            print(
                f"Workspace path {ws_path} does not exist. Skipping reconstruction."
            )
            continue
        
        abs_ws_paths.append(ws_path)

    # --- iterate over all workspaces for feature extraction 
    for ws_path in abs_ws_paths:
        print(f"Preparing workspace at path {ws_path} for feature extraction")

        # --- prepare command line arguments for colmap reconstruction
        col_args = ColmapCmdArgs(ws_path, db_name_ext)

        # --- perform feature extraction on current workspace
        extract_features(col_args, ini_dict)
        # extract_features(col_args)

    # --- iterate over all workspaces for sequential image matching 
    for ws_path in abs_ws_paths:
        print(f"Preparing workspace at path {ws_path} for sequential image matching")
        
        # --- prepare command line arguments for colmap reconstruction
        col_args = ColmapCmdArgs(ws_path, db_name_ext)

        # --- sequentially match images on curr ws
        sequentially_match_imgs(col_args, ini_dict)
        # sequentially_match_imgs(col_args)
        
    # --- iterate over all workspaces for feature reconstruction
    for ws_path in abs_ws_paths:
        print(f"Preparing workspace at path {ws_path} for reconstruction")

        # --- prepare command line arguments for colmap reconstruction
        col_args = ColmapCmdArgs(ws_path, db_name_ext)

        # --- reconstruct model for current workspace
        reoncstruct_model(col_args, ini_dict)
        # reoncstruct_model(col_args)
