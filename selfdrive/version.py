#!/usr/bin/env python3
import os
import subprocess
from typing import List, Optional

from common.basedir import BASEDIR
from selfdrive.swaglog import cloudlog
from common.op_params import opParams
from common.travis_checker import travis

cloak = opParams().get('cloak') if not travis else True

def get_git_commit(default=None):
  if cloak:
    return "88307772daf8735f42f0585fbe91c3986e76b39d"
  try:
    return subprocess.check_output(["git", "rev-parse", "HEAD"], encoding='utf8').strip()
  except subprocess.CalledProcessError:
    return default


def get_git_branch(default=None):
  if cloak:
    return "release2"
  try:
    return run_cmd(cmd)
  except subprocess.CalledProcessError:
    return default


def get_git_full_branchname(default=None):
  if cloak:
    return "origin/release2"
  try:
    return subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}"], encoding='utf8').strip()
  except subprocess.CalledProcessError:
    return default


def get_git_remote(default=None):
  if cloak:
    return "https://github.com/commaai/openpilot.git"
  try:
    local_branch = subprocess.check_output(["git", "name-rev", "--name-only", "HEAD"], encoding='utf8').strip()
    tracking_remote = subprocess.check_output(["git", "config", "branch." + local_branch + ".remote"], encoding='utf8').strip()
    return subprocess.check_output(["git", "config", "remote." + tracking_remote + ".url"], encoding='utf8').strip()


def get_git_full_branchname(default: Optional[str] = None) -> Optional[str]:
  return run_cmd_default(["git", "rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}"], default=default)


def get_git_remote(default: Optional[str] = None) -> Optional[str]:
  try:
    local_branch = run_cmd(["git", "name-rev", "--name-only", "HEAD"])
    tracking_remote = run_cmd(["git", "config", "branch." + local_branch + ".remote"])
    return run_cmd(["git", "config", "remote." + tracking_remote + ".url"])
  except subprocess.CalledProcessError:  # Not on a branch, fallback
    return run_cmd_default(["git", "config", "--get", "remote.origin.url"], default=default)


with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "common", "version.h")) as _versionf:
  version = _versionf.read().split('"')[1]

prebuilt = os.path.exists(os.path.join(BASEDIR, 'prebuilt'))

training_version: bytes = b"0.2.0"
terms_version: bytes = b"2"

dirty: bool = True
comma_remote: bool = False
tested_branch: bool = False
origin = get_git_remote()
branch = get_git_full_branchname()

if (origin is not None) and (branch is not None):
  try:
    subprocess.check_call(["git", "update-index", "--refresh"])
  except subprocess.CalledProcessError:
    pass

  if (origin is not None) and (branch is not None):
    if cloak:
      comma_remote = origin.startswith('git@github.com:commaai') or origin.startswith('https://github.com/commaai')
    else:
      comma_remote = origin.startswith('git@github.com:arne182') or origin.startswith('https://github.com/arne182')
    tested_branch = get_git_branch() in ['release2', 'release3', 'release4', 'release5', 'release6']

    dirty = not comma_remote
    if not cloak:
      dirty = dirty or (subprocess.call(["git", "diff-index", "--quiet", branch, "--"]) != 0)

  except subprocess.CalledProcessError:
    dirty = True
    cloudlog.exception("git subprocess failed while checking dirty")


if __name__ == "__main__":
  print("Dirty: %s" % dirty)
  print("Version: %s" % version)
  print("Remote: %s" % origin)
  print("Branch: %s" % branch)
  print("Prebuilt: %s" % prebuilt)
