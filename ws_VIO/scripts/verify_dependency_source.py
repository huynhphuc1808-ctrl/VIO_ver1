#!/usr/bin/env python3
"""Verify pinned dependency checkouts or source files included in the repository."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


DEFAULT_LOCK = Path(__file__).resolve().parents[2] / "DEPENDENCIES.lock.json"
IGNORED_DIRECTORIES = {
    ".git", ".vscode", "build", "install", "log", "__pycache__", ".pytest_cache",
}


def git_output(source, *arguments):
    """Run Git only when a dependency has its own Git metadata."""
    return subprocess.check_output(
        ["git", "-C", str(source), *arguments], text=True, stderr=subprocess.PIPE
    ).strip()


def verify_source(source, expected_commit, lock_path=DEFAULT_LOCK):
    """Reject changes to the pinned dependency source before build checks."""
    source = Path(source).resolve()
    if not source.is_dir():
        raise ValueError(f"Dependency directory is missing: {source}")
    if (source / ".git").exists():
        checkout_root = Path(git_output(source, "rev-parse", "--show-toplevel")).resolve()
        actual = git_output(source, "rev-parse", "HEAD")
        if checkout_root != source or actual != expected_commit:
            raise ValueError(f"Dependency checkout does not match {expected_commit}: {source}")
        if git_output(source, "status", "--porcelain", "--untracked-files=no"):
            raise ValueError(f"Pinned dependency has modified tracked files: {source}")
        return actual

    lock_path = Path(lock_path).resolve()
    manifest = json.loads(lock_path.read_text())
    if manifest.get("schema_version") != 1:
        raise ValueError("Unsupported dependency source manifest version")
    relative = source.relative_to(lock_path.parent).as_posix()
    entry = manifest.get("dependencies", {}).get(relative)
    if not entry or entry.get("commit") != expected_commit:
        raise ValueError(f"No matching pinned source manifest entry: {relative}")
    expected_files = entry.get("files", {})
    if not expected_files:
        raise ValueError(f"Dependency manifest has no files: {relative}")
    for name, record in expected_files.items():
        local = Path(name)
        if local.is_absolute() or ".." in local.parts:
            raise ValueError(f"Invalid manifest path: {name}")
        path = source / local
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"Pinned source file is missing or replaced: {relative}/{name}")
        actual_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        actual_mode = "100755" if path.stat().st_mode & 0o111 else "100644"
        if actual_hash != record["sha256"] or actual_mode != record["mode"]:
            raise ValueError(f"Pinned source file changed: {relative}/{name}")
    actual_files = set()
    for folder, directories, names in os.walk(source):
        directories[:] = [name for name in directories if name not in IGNORED_DIRECTORIES]
        for name in names:
            if name == ".git" or name.endswith((".pyc", ".pyo")):
                continue
            actual_files.add((Path(folder) / name).relative_to(source).as_posix())
    unexpected = sorted(actual_files - expected_files.keys())
    if unexpected:
        raise ValueError(f"Unexpected dependency source files in {relative}: {unexpected[:5]}")
    return entry["commit"]


def main():
    """Verify one dependency for the build checker, or all included sources."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?")
    parser.add_argument("expected_commit", nargs="?")
    parser.add_argument("--all", action="store_true", help="verify all included dependencies")
    args = parser.parse_args()
    if args.all:
        if args.source or args.expected_commit:
            parser.error("--all does not accept a source directory or commit")
        manifest = json.loads(DEFAULT_LOCK.read_text())
        for name, entry in manifest["dependencies"].items():
            commit = verify_source(DEFAULT_LOCK.parent / name, entry["commit"])
            print(f"{name}: {commit} ({len(entry['files'])} files verified)")
    else:
        if not args.source or not args.expected_commit:
            parser.error("provide SOURCE EXPECTED_COMMIT, or --all")
        print(verify_source(args.source, args.expected_commit))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"Dependency verification failed: {error}", file=sys.stderr)
        sys.exit(1)
