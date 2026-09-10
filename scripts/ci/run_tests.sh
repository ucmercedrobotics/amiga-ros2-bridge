#!/usr/bin/env bash
# Build and test the hardware-free subset of the workspace listed in
# packages.txt.
#
# Nothing in here is GitHub-specific: it runs in whatever ROS 2 container it is
# started in. CI uses a stock ros:humble-ros-base; `make ci-test` runs the same
# script in the same image locally, and it also works inside the dev image from
# the Dockerfile (pass SKIP_ROSDEP=1 there, since that image already has every
# dependency baked in).
#
#   SKIP_ROSDEP=1   don't touch apt; assume dependencies are present
#   SKIP_UNDERLAY=1 don't build BehaviorTree.ROS2; it is already on the path
#   UNDERLAY_WS     where that underlay lives (default /opt/underlay_ws)
#   PACKAGE_LIST    override the package list (default scripts/ci/packages.txt)
#   LINTERS         exclude (default) | only | include -- see below
#   BUILD_BASE      colcon build dir (default build)
#   INSTALL_BASE    colcon install dir (default install) -- `make ci-test`
#                   points both at */ci so a local run does not clobber the
#                   full workspace build in the dev container
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

PACKAGE_LIST="${PACKAGE_LIST:-scripts/ci/packages.txt}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
BUILD_BASE="${BUILD_BASE:-build}"
INSTALL_BASE="${INSTALL_BASE:-install}"
UNDERLAY_WS="${UNDERLAY_WS:-/opt/underlay_ws}"

# Dependency keys rosdep must not try to resolve, because something other than
# apt provides them: behaviortree_ros2 comes from the underlay built by
# build_underlay.sh, and has no humble debian to fall back on.
SKIP_KEYS="behaviortree_ros2"

# Strip comments and blank lines; what's left is one package path per line.
mapfile -t PACKAGE_PATHS < <(sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "$PACKAGE_LIST")
if (( ${#PACKAGE_PATHS[@]} == 0 )); then
    echo "no packages listed in $PACKAGE_LIST" >&2
    exit 1
fi

# A path from a submodule that was never checked out looks exactly like a
# healthy empty directory to colcon, which would then silently test less than
# it claims to. Fail loudly instead.
missing=()
for path in "${PACKAGE_PATHS[@]}"; do
    [[ -f "$path/package.xml" ]] || missing+=("$path")
done
if (( ${#missing[@]} )); then
    echo "no package.xml in: ${missing[*]}" >&2
    echo "submodules not checked out? try: git submodule update --init --recursive" >&2
    exit 1
fi

echo "==> testing ${#PACKAGE_PATHS[@]} packages: ${PACKAGE_PATHS[*]}"

# ROS's setup.bash reads variables it hasn't set yet, so nounset has to come
# off around every source in this script.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if [[ "${SKIP_ROSDEP:-0}" != "1" ]]; then
    echo "==> refreshing apt and rosdep"
    apt-get update
    # ros:* images ship rosdep already initialised; a plain ROS install may not.
    [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || rosdep init
    rosdep update --rosdistro "$ROS_DISTRO"
    # Tells build_underlay.sh not to repeat the two steps above.
    export ROSDEP_READY=1
fi

if [[ "${SKIP_UNDERLAY:-0}" != "1" ]]; then
    scripts/ci/build_underlay.sh
fi

# Sourced whether or not this run built it, so SKIP_UNDERLAY=1 still works
# against an underlay a previous run left behind.
if [[ -f "$UNDERLAY_WS/install/setup.bash" ]]; then
    echo "==> using underlay at $UNDERLAY_WS"
    set +u
    # shellcheck disable=SC1091
    source "$UNDERLAY_WS/install/setup.bash"
    set -u
fi

if [[ "${SKIP_ROSDEP:-0}" != "1" ]]; then
    echo "==> installing dependencies"
    rosdep install --from-paths "${PACKAGE_PATHS[@]}" --ignore-src -y \
        --rosdistro "$ROS_DISTRO" --skip-keys "$SKIP_KEYS"
fi

COLCON_BASE=(--base-paths "${PACKAGE_PATHS[@]}"
             --build-base "$BUILD_BASE" --install-base "$INSTALL_BASE")

echo "==> colcon build"
colcon build "${COLCON_BASE[@]}" --event-handlers console_direct+

# The tests import the messages they were just built against, so the overlay
# has to be on the path before colcon test runs them.
set +u
# shellcheck disable=SC1091
source "$INSTALL_BASE/setup.bash"
set -u

# ament's copyright/flake8/pep257 checks are ordinary tests: pytest marks them
# 'linter', ctest labels them 'linter'. They fail today on code that predates
# this pipeline, and style is already enforced on the way in by pre-commit, so
# they are split out rather than left to block every merge.
#
#   exclude   everything but the linters -- the check that gates main
#   only      just the linters -- advisory, reports the existing debt
#   include   both, which is what you want once the debt is paid off
#
# --no-tests=ignore keeps 'only' from failing the CMake packages that have no
# linter tests at all.
case "${LINTERS:-exclude}" in
    exclude) TEST_ARGS=(--pytest-args -m "not linter" --ctest-args -LE linter --no-tests=ignore) ;;
    only)    TEST_ARGS=(--pytest-args -m "linter" --ctest-args -L linter --no-tests=ignore) ;;
    include) TEST_ARGS=() ;;
    *)       echo "LINTERS must be exclude, only or include (got '$LINTERS')" >&2; exit 1 ;;
esac

echo "==> colcon test (LINTERS=${LINTERS:-exclude})"
colcon test "${COLCON_BASE[@]}" --event-handlers console_direct+ \
    --return-code-on-test-failure "${TEST_ARGS[@]}"
