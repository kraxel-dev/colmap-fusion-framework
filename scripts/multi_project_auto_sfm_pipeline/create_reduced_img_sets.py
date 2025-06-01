"""Given the path to a colmap workspace, duplicate the nested /images folder but as reduced subset of original amount of images (e.g. keep every 2nd img, keep every 3rd img and so on.)"""

import sys, argparse, os, shutil
from pathlib import Path


def get_img_extensions():
    return {".jpg", ".jpeg", ".png", ".bmp"}


def duplicate_img_folder():
    pass


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i",
        "--input",
        required=True,
        help="Directory path to a colmap project folder containing /images subfolder that will be duplicated and reduced.",
    )
    parser.add_argument(
        "-f",
        "--force",
        default=False,
        help="Allow overwriting of existing reduced image folders.",
    )
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    # duplicated folders will be created with every n-th image kept (e.g. sparse3x, sparse5x, sparse7x)
    keep_every_nth = [3, 5, 7]

    # root dir path where the folders with reduced imgs will be created
    project_dir = Path(args.input).resolve()
    is_overwrite = args.force

    subfolder_name = "images"  # subfolder containing the original images
    og_imgs_path = project_dir / subfolder_name

    # check if images subfolder exists in project dir
    if not og_imgs_path.is_dir():
        print(
            f"Error: Subfolder '{subfolder_name}' not found in '{project_dir}'. Make sure you have placed the /images folder into your project dir accordingly."
        )
        sys.exit(1)
    else:
        print(f"Subfolder found: {og_imgs_path}")

    # iterate over all specified sets of reducement
    for n in keep_every_nth:

        # destination folder for reduced imgs in root dir (next to original /images dir)
        reduced_proj_dir = project_dir / f"sparse{n}x"
        reduced_imgs_path = reduced_proj_dir / "images"
        print(f"Copying images to new path: {reduced_imgs_path}")

        # copy original images to new dst
        shutil.copytree(og_imgs_path, reduced_imgs_path, dirs_exist_ok=is_overwrite)
        # check wheter img mask exists and copy it as well
        if (project_dir/"mask").is_dir():
            print(f"Found image mask in: {project_dir}. Copying it to reduced images folder.")
            shutil.copytree(
                project_dir / "mask",
                reduced_proj_dir / "mask",
                dirs_exist_ok=is_overwrite,
            )

        # grab all imgs existing in new images folder
        image_files = sorted(
            f
            for f in reduced_imgs_path.rglob("*")
            if f.suffix.lower() in get_img_extensions() and f.is_file()
        )

        # keep every n-th (including very first)
        good_indicies = range(0, len(image_files), n)

        for i, file in enumerate(image_files):
            if i not in good_indicies:
                print(f"deleting file: {file.name} with idx: {i}")
                file.unlink()


if __name__ == "__main__":
    main()
