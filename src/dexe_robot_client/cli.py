'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from importlib import resources
from pathlib import Path
import shutil
from dexe_robot_client.common import DATA_DIR


def _copy_tree(pkg, dst: Path):
    root = resources.files(pkg)

    def walk(src, rel):
        for item in src.iterdir():
            if item.is_dir():
                walk(item, rel / item.name)
            else:
                if item.name.endswith(".py"):
                    target = dst / rel / item.name
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with resources.as_file(item) as real_path:
                        shutil.copy(real_path, target)

    walk(root, Path())


def export_scripts(base_dir=None):
    if base_dir is None:
        base_dir = DATA_DIR
    else:
        base_dir = Path(base_dir)

    examples_dst = base_dir / "examples"
    scripts_dst = base_dir / "scripts"

    examples_dst.mkdir(parents=True, exist_ok=True)
    scripts_dst.mkdir(parents=True, exist_ok=True)

    # ---- examples ----
    _copy_tree("dexe_robot_client.examples", examples_dst)

    # ---- scripts（一层即可）----
    scripts_pkg = resources.files("dexe_robot_client.scripts")
    for item in scripts_pkg.iterdir():
        if item.is_file() and item.name.endswith(".py"):
            with resources.as_file(item) as real_path:
                shutil.copy(real_path, scripts_dst / item.name)

    print(f"Data installed to: {base_dir.resolve()}")
