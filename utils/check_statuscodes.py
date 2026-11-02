# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------
"""Validate the status-code catalog without rewriting its contents."""

import argparse
from pathlib import Path
import re
import sys

import yaml

DEFAULT_CONFIG_DIR = (Path(__file__).resolve().parent.parent /
                      "supervisor_interfaces/config")
ID_LENGTHS = {"module_id": 3, "submodule_id": 2, "status_code": 8}
SCHEMA = {
    "root": ((), "modules", "module"),
    "module": (
        ("module", "module_name", "module_id"), "submodules", "submodule"),
    "submodule": (("submodule", "submodule_name", "submodule_id"),
                  "status_codes", "status"),
    "status": (("status_code", "description", "solution"), None, None),
}


class UniqueKeyLoader(yaml.SafeLoader):
    """Reject duplicate keys instead of silently keeping the last value."""

    def construct_mapping(self, node, deep=False):
        keys = set()
        for key_node, _ in node.value:
            if key_node.tag != "tag:yaml.org,2002:str":
                raise yaml.constructor.ConstructorError(None, None, "字段名必须是字符串",
                                                        key_node.start_mark)
            key = key_node.value
            if key in keys:
                raise yaml.constructor.ConstructorError(None, None,
                                                        "重复字段: " + key,
                                                        key_node.start_mark)
            keys.add(key)
        return super().construct_mapping(node, deep=deep)


def validate_catalog(data):
    """Return all schema errors, with paths to the offending fields."""
    errors = []
    status_codes = {}

    def check_unique(value, field_path, seen):
        if value in seen:
            errors.append(f"{field_path}: 编号 {value!r} 重复，首次出现于 {seen[value]}")
        else:
            seen[value] = field_path

    def check_record(record,
                     kind,
                     path,
                     module_id=None,
                     submodule_id=None,
                     sibling_ids=None):
        if not isinstance(record, dict):
            errors.append(f"{path}: 必须是映射（字段: 值）")
            return
        fields, children, child_kind = SCHEMA[kind]
        required = fields + ((children,) if children else ())
        for key in required:
            if key not in record:
                errors.append(f"{path}.{key}: 缺少必填字段")
        for key in record:
            if key not in required:
                errors.append(f"{path}.{key}: 未知字段或层级错误")
        valid_ids = {}
        for key in fields:
            if key not in record:
                continue
            value = record[key]
            field_path = f"{path}.{key}"
            if not isinstance(value, str):
                errors.append(f"{field_path}: 必须是字符串，数字请加引号")
            elif not value.strip():
                errors.append(f"{field_path}: 不能是空字符串或纯空白")
            elif key in ID_LENGTHS:
                length = ID_LENGTHS[key]
                if re.fullmatch(r"[0-9]{%d}" % length, value) is None:
                    errors.append(
                        f"{field_path}: 必须是 {length} 位 ASCII 数字字符串，不能含空白")
                else:
                    valid_ids[key] = value
        if kind == "module":
            module_id = valid_ids.get("module_id")
            if module_id is not None:
                check_unique(module_id, f"{path}.module_id", sibling_ids)
        elif kind == "submodule":
            submodule_id = valid_ids.get("submodule_id")
            if submodule_id is not None:
                check_unique(submodule_id, f"{path}.submodule_id", sibling_ids)
        elif kind == "status" and "status_code" in valid_ids:
            code = valid_ids["status_code"]
            field_path = f"{path}.status_code"
            check_unique(code, field_path, status_codes)
            if module_id is not None and code[:3] != module_id:
                errors.append(
                    f"{field_path}: 前 3 位 {code[:3]!r} 与 module_id {module_id!r} 不匹配"
                )
            if submodule_id is not None and code[3:5] != submodule_id:
                errors.append(
                    f"{field_path}: 第 4–5 位 {code[3:5]!r} 与 submodule_id {submodule_id!r} 不匹配"
                )
        if children and children in record:
            items = record[children]
            child_path = f"{path}.{children}"
            if not isinstance(items, list) or not items:
                errors.append(f"{child_path}: 必须是非空列表")
                return
            child_ids = {}
            for index, item in enumerate(items):
                check_record(item, child_kind, f"{child_path}[{index}]",
                             module_id, submodule_id, child_ids)

    check_record(data, "root", "$")
    return errors


def check_file(path):
    """Read one YAML document safely and return validation errors."""
    try:
        with Path(path).open(encoding="utf-8") as stream:
            data = yaml.load(stream, Loader=UniqueKeyLoader)
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        return [str(error)]
    return validate_catalog(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "path",
        nargs="?",
        type=Path,
        default=DEFAULT_CONFIG_DIR,
        help="YAML 文件或目录路径，默认递归检查 supervisor_interfaces/config 下所有 YAML 文件")
    args = parser.parse_args()
    if args.path.is_dir():
        paths = sorted(
            path for path in args.path.rglob("*")
            if path.is_file() and path.suffix.lower() in (".yaml", ".yml"))
    else:
        paths = [args.path]
    if not paths:
        print(f"{args.path}: 未找到 YAML 文件", file=sys.stderr)
        return 1
    failed_files = 0
    error_count = 0
    for path in paths:
        errors = check_file(path)
        if errors:
            failed_files += 1
            error_count += len(errors)
            for error in errors:
                print(f"{path}: {error}", file=sys.stderr)
        else:
            print(f"状态码校验通过：{path}")
    if failed_files:
        print(
            f"状态码校验失败：检查 {len(paths)} 个文件，{failed_files} 个文件失败，共 {error_count} 个错误",
            file=sys.stderr)
        return 1
    print(f"状态码校验全部通过：{len(paths)} 个文件")
    return 0


if __name__ == "__main__":
    sys.exit(main())
