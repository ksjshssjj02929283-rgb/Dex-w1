#!/usr/bin/env python3
"""Check release versions against the release branch name."""

import re
import subprocess
import sys
import xml.etree.ElementTree as ET

REQUIRED_FILES = (
    "package.xml",
    "pyproject.toml",
    "CHANGELOG.md",
)


def read_version(path, content):
    try:
        if path == "package.xml":
            version = ET.fromstring(content).findtext("version")
        else:
            # Only read the literal version in [project], not dependencies or
            # other tables. Keep the hook dependency-free on Python 3.10.
            project = re.search(
                r"^[ \t]*\[project\][ \t]*(?:#[^\n]*)?\r?$\n"
                r"(.*?)(?=^[ \t]*\[|\Z)", content, re.M | re.S)
            versions = re.findall(
                r"^[ \t]*version[ \t]*=[ \t]*"
                r"(?:\"([0-9]+\.[0-9]+\.[0-9]+)\"|'([0-9]+\.[0-9]+\.[0-9]+)')"
                r"[ \t]*(?:#[^\n]*)?\r?$",
                project[1] if project else "", re.M)
            version = next(filter(None, versions[0])) if len(versions) == 1 else None
        if not isinstance(version, str) or not re.fullmatch(
                r"[0-9]+\.[0-9]+\.[0-9]+", version.strip()):
            raise ValueError("需要明确的 major.minor.patch 版本号")
        return version.strip()
    except (ValueError, SyntaxError, ET.ParseError) as error:
        raise RuntimeError(f"{path}：无法解析版本号（{error}）。") from error


def check_versions(version, commit):
    errors = []
    for path in REQUIRED_FILES[:-1]:
        actual = read_version(path, git("show", f"{commit}:{path}"))
        if actual != version:
            errors.append(f"{path}：版本号为 {actual}，应与分支版本 {version} 一致")
    changelog = git("show", f"{commit}:CHANGELOG.md")
    if not re.search(rf"^##\s+\[[vV]?{re.escape(version)}\]\s*$", changelog,
                     re.M):
        errors.append(f"CHANGELOG.md：缺少 ## [V{version}] 或 ## [{version}] 发布标题")
    if errors:
        raise RuntimeError("\n".join(errors) + "\n请修改并提交后再推送。")


def git(*args):
    return subprocess.run(
        ["git", *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    ).stdout.strip()


def check_push(lines):
    failed = False
    for line in lines:
        local_ref, local_oid, remote_ref, _ = line.split()
        if set(local_oid) == {"0"}:  # Branch deletion has no commit to check.
            continue
        if not any(
                ref.startswith("refs/heads/release/")
                for ref in (local_ref, remote_ref)):
            continue
        try:
            branches = list(
                dict.fromkeys(
                    ref.removeprefix("refs/heads/")
                    for ref in (local_ref, remote_ref)
                    if ref.startswith("refs/heads/release/")))
            for branch in branches:
                match = re.fullmatch(r"release/v([0-9]+\.[0-9]+\.[0-9]+)",
                                     branch)
                if not match:
                    raise RuntimeError(f"分支 {branch} 必须使用 release/vX.Y.Z 命名格式。")
                version = match[1]
                check_versions(version, local_oid)
                print(f"[release 检查] {branch}：版本 {version} 和 CHANGELOG 检查通过。",
                      file=sys.stderr)
        except (RuntimeError, subprocess.CalledProcessError) as error:
            detail = error.stderr.strip() if isinstance(
                error, subprocess.CalledProcessError) else str(error)
            print(f"[release 检查] 禁止推送 {remote_ref}：{detail}", file=sys.stderr)
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(check_push(sys.stdin))
