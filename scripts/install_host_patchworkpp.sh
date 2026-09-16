#!/usr/bin/env bash
# Install Patchwork++ for the HOST offline harnesses (ground_ab / lean_ab / neighbour_ab).
#
# The node itself gets Patchwork++ from docker/Dockerfile.object_fusion; this is only for running
# the harnesses outside the container. It used to live in the session scratchpad, which does not
# survive a reboot -- hence this script.
#
#   scripts/install_host_patchworkpp.sh [target_dir]      # default ~/.local/lib/patchworkpp
#
# Then, for any harness that segments ground:
#
#   PYTHONPATH=$(scripts/install_host_patchworkpp.sh --path) python3 scripts/ground_ab.py ...
#
# Two constraints the plain pip command gets wrong, both learned the hard way:
#   * --target pulls numpy 2.x as a dependency, and numpy 2.x breaks the system scipy and mcap.
#     The numpy it installs must be deleted; the system numpy is the one to use.
#   * No venv: python3.12-venv is not installed and sudo needs a password, so --target it is.
set -euo pipefail

VERSION="1.4.1"
TARGET="${HOME}/.local/lib/patchworkpp"

if [[ "${1:-}" == "--path" ]]; then
  echo "$TARGET"
  exit 0
fi
[[ $# -gt 0 ]] && TARGET="$1"

if [[ -f "$TARGET/pypatchworkpp.cpython-312-x86_64-linux-gnu.so" ]]; then
  echo "already installed: $TARGET"
else
  echo "installing pypatchworkpp==$VERSION into $TARGET"
  mkdir -p "$TARGET"
  pip install --quiet --target "$TARGET" "pypatchworkpp==$VERSION"
  # numpy 2.x here shadows the system numpy on PYTHONPATH and breaks scipy and mcap.
  rm -rf "$TARGET"/numpy "$TARGET"/numpy-* "$TARGET"/numpy.libs
  echo "removed the bundled numpy; the system one is used instead"
fi

echo "verifying..."
PYTHONPATH="$TARGET" python3 -c "
import pypatchworkpp, numpy, scipy      # scipy import proves the numpy removal worked
p = pypatchworkpp.Parameters()
print(f'  pypatchworkpp ok, numpy {numpy.__version__}, sensor_height default {p.sensor_height}')
"
echo
echo "use it with:  PYTHONPATH=$TARGET python3 scripts/<harness>.py ..."
