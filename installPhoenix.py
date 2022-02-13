import os
from pathlib import Path
import shutil as shell
from getpass import getuser


def moveFiles(download_dir: Path, install_dir: Path, obj_dir: Path) -> None:
    """ moveFiles moves Files"""
    # transition to utilizing a less edge-case build for moveFiles, i.e. just downlaod_dir install_dir
    for file in download_dir.iterdir():
        shell.copytree(file, install_dir)
    shell.copytree(obj_dir, install_dir / "lib")
    print(f"Succesfully copied files into {install_dir}")

    return


def main() -> None:
    # builds out function call
    home = Path("/home")
    user = getuser()
    download_dir = home / user / "Downloads"
    install_dir = Path("/usr/lib/ctre")

    phoenix_library = download_dir / "Phoenix-Linux-SocketCAN-Example" / "include"
    phoenix_obj_lib = download_dir / "Phoenix-Linux-SocketCAN-Example" / "lib"
    # phoenix_library = download_dir / "Phoenix-Linux-SocketCAN-Example" / "include" / "ctre"
    print(phoenix_library.is_dir())

    if not (phoenix_library.is_dir()):
        print(
            f"The CTRE PheonixLinuxSocketCan code does not currently exist at {phoenix_library} \n, please try running install.sh again"
        )
        exit(1)
    else:
        moveFiles(phoenix_library, install_dir, phoenix_obj_lib)

    print("Have a nice day!!")

    return 0


if __name__ == "__main__":
    main()
