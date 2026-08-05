#!/usr/bin/env python3
"""Read-only consistency check for navigation_client delivery documentation."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys

DOCS_DIR = Path(__file__).resolve().parent
MODULE_DIR = DOCS_DIR.parent


def find_repo_root() -> Path:
    for candidate in (MODULE_DIR, *MODULE_DIR.parents):
        if (candidate / "pyproject.toml").is_file():
            return candidate
    raise RuntimeError("未找到包含 pyproject.toml 的仓库根目录")


def public_methods() -> list[str]:
    tree = ast.parse(
        (MODULE_DIR / "chassis_client.py").read_text(encoding="utf-8"))
    client = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "ChassisClient")
    return [
        node.name
        for node in client.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and
        not node.name.startswith("_")
    ]


def interface_configs() -> dict[str, dict[str, object]]:
    tree = ast.parse((MODULE_DIR / "interfaces.py").read_text(encoding="utf-8"))
    configs: dict[str, dict[str, object]] = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not isinstance(target, ast.Name) or target.id not in {"REAL", "SIM"}:
            continue
        if not isinstance(node.value, ast.Call):
            continue
        configs[target.id] = {
            keyword.arg: ast.literal_eval(keyword.value)
            for keyword in node.value.keywords
            if keyword.arg is not None
        }
    return configs


def automatic_cases() -> list[list[str]]:
    tree = ast.parse(
        (MODULE_DIR / "interface_test.py").read_text(encoding="utf-8"))
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if not any(
                isinstance(target, ast.Name) and target.id == "CASES"
                for target in node.targets):
            continue
        if not isinstance(node.value, (ast.Tuple, ast.List)):
            break
        return [[str(ast.literal_eval(arg))
                 for arg in element.args[:3]]
                for element in node.value.elts
                if isinstance(element, ast.Call)]
    raise RuntimeError("未能从 interface_test.py 解析 CASES")


def contract_payload() -> dict[str, object]:
    return {
        "public_methods": public_methods(),
        "interface_configs": interface_configs(),
        "automatic_cases": automatic_cases(),
    }


def contract_fingerprint() -> str:
    normalized = json.dumps(
        contract_payload(),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(normalized).hexdigest()


def project_version(pyproject_text: str) -> str:
    """Read project.version without adding a Python 3.10 TOML dependency."""
    in_project = False
    for raw_line in pyproject_text.splitlines():
        line = raw_line.strip()
        if line.startswith("["):
            in_project = line == "[project]"
            continue
        if in_project and line.startswith("version") and "=" in line:
            return line.split("=", 1)[1].strip().strip('"').strip("'")
    raise RuntimeError("pyproject.toml [project] 中未找到 version")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="校验 navigation_client 交付文档与当前公开契约")
    parser.add_argument(
        "--print-fingerprint",
        action="store_true",
        help="打印当前公开契约指纹，供人工审阅变更后更新 manifest",
    )
    args = parser.parse_args()

    fingerprint = contract_fingerprint()
    if args.print_fingerprint:
        print(fingerprint)
        return 0

    repo_root = find_repo_root()
    pyproject_text = (repo_root / "pyproject.toml").read_text(encoding="utf-8")
    manifest = json.loads(
        (DOCS_DIR / "version_manifest.json").read_text(encoding="utf-8"))
    package_version = project_version(pyproject_text)
    failures: list[str] = []

    if manifest.get("package_version") != package_version:
        failures.append(
            "manifest package_version 与 pyproject.toml project.version 不一致")
    if manifest.get("contract_fingerprint") != fingerprint:
        failures.append("公开方法/接口配置/RC-A 目录指纹已变化；请复核正文后更新 manifest")

    documents = manifest.get("documents", [])
    for name in documents:
        path = DOCS_DIR / name
        if not path.is_file():
            failures.append(f"缺少文档：{name}")
            continue
        content = path.read_text(encoding="utf-8")
        if f"**适用包版本**：`{package_version}`" not in content:
            failures.append(f"{name} 未声明当前包版本 {package_version}")

    api_text = (DOCS_DIR / "api_and_usage.md").read_text(encoding="utf-8")
    for method in public_methods():
        if f"`{method}" not in api_text:
            failures.append(f"调用侧文档缺少公开方法：{method}")
    for case_id, case_name, _purpose in automatic_cases():
        if f"`{case_id}`" not in api_text or case_name not in api_text:
            failures.append(f"调用侧文档缺少自动用例：{case_id} {case_name}")

    for pattern in ("README.md", "docs/*.md", "docs/*.json", "docs/*.py"):
        if f'"{pattern}"' not in pyproject_text:
            failures.append(f"setuptools package-data 缺少：{pattern}")

    if failures:
        print("FAIL: navigation_client 交付文档一致性检查失败")
        for failure in failures:
            print(f"- {failure}")
        print(f"- 当前契约指纹：{fingerprint}")
        return 1

    print("PASS: navigation_client 交付文档与当前公开契约一致")
    print(f"package_version={package_version}")
    print(f"docs_version={manifest['docs_version']}")
    print(f"contract_fingerprint={fingerprint}")
    print(f"public_methods={len(public_methods())}")
    print(f"automatic_cases={len(automatic_cases())}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
