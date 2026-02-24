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


def _copy_tree(src_root, dst: Path):
    for item in src_root.rglob("*.py"):
        rel = item.relative_to(src_root)
        target = dst / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        with resources.as_file(item) as real_path:
            shutil.copy(real_path, target)


def export_scripts(base_dir=None):
    base_dir = Path(base_dir) if base_dir else DATA_DIR

    examples_dst = base_dir / "examples"
    scripts_dst = base_dir / "scripts"

    examples_dst.mkdir(parents=True, exist_ok=True)
    scripts_dst.mkdir(parents=True, exist_ok=True)

    # 从 wheel 里的 data files 定位
    pkg_root = resources.files("dexe_robot_client").parent

    examples_src = pkg_root / "examples"
    scripts_src = pkg_root / "scripts"

    if examples_src.exists():
        _copy_tree(examples_src, examples_dst)

    if scripts_src.exists():
        for item in scripts_src.glob("*.py"):
            with resources.as_file(item) as real_path:
                shutil.copy(real_path, scripts_dst / item.name)

    print(f"Data installed to: {base_dir.resolve()}")
