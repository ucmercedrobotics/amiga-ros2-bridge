#!/usr/bin/env bash
# Install the SPIN model checker, which amiga_ros2_agents/verification/verify.py shells out
# to when it re-verifies a replanned mission against its LTL specification.
#
# Built from source rather than taken from upstream's Bin/ directory, which
# ships linux32, linux64, mac64 and windows only. The robots and the dev box are
# aarch64, so there is no prebuilt binary to download and the alternative is
# switching verification off on ARM -- which would mean the machine that flies
# the mission is the one that cannot check it. Src/ is small portable C and
# builds in a few seconds on both architectures.
#
# Not an apt install: Ubuntu ships spin in universe, but the version floats with
# the distro, and CI and the dev image are based on different ones. A pinned
# source build keeps every environment on the same checker.
#
# Used by the Dockerfile (dev image) and by run_tests.sh (CI), so the two agree
# by construction.
#
#   SPIN_VERSION    upstream tag to build (default below)
#   SPIN_PREFIX     where to install the binary (default /usr/local/bin)
#   SKIP_APT=1      don't touch apt; assume byacc/flex/gcc are present
set -euo pipefail

SPIN_VERSION="${SPIN_VERSION:-6.5.2}"
SPIN_PREFIX="${SPIN_PREFIX:-/usr/local/bin}"

# Already there and working? Nothing to do. Keeps a cached CI image and repeated
# local runs off the critical path.
if command -v spin >/dev/null 2>&1 && spin -V >/dev/null 2>&1; then
    echo "==> spin already installed: $(spin -V | head -1)"
    exit 0
fi

if [[ "${SKIP_APT:-0}" != "1" ]]; then
    echo "==> installing spin build dependencies"
    apt-get update
    apt-get install -y --no-install-recommends byacc flex gcc make curl ca-certificates
fi

workdir="$(mktemp -d)"
trap 'rm -rf "$workdir"' EXIT

echo "==> building spin ${SPIN_VERSION}"
curl -fsSL -o "$workdir/spin.tar.gz" \
    "https://github.com/nimble-code/Spin/archive/refs/tags/version-${SPIN_VERSION}.tar.gz"
tar -xzf "$workdir/spin.tar.gz" -C "$workdir"
make -C "$workdir/Spin-version-${SPIN_VERSION}/Src"
install -m 0755 "$workdir/Spin-version-${SPIN_VERSION}/Src/spin" "$SPIN_PREFIX/spin"

echo "==> $("$SPIN_PREFIX/spin" -V | head -1)"
