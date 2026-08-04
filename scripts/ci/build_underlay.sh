#!/usr/bin/env bash
# Build the dependencies that have no humble apt package into an underlay, so
# run_tests.sh can source it before building the workspace.
#
# Today that is only BehaviorTree.ROS2: amiga_ros2_behavior_tree does
# find_package(behaviortree_ros2), and only behaviortree_cpp is released as a
# humble debian. This is the same source build the dev image does in the
# Dockerfile, with two differences that matter for CI:
#
#   * it is pinned to a commit, so a green run stays green tomorrow. Bump
#     BTCPP_ROS2_REF deliberately, not by accident.
#   * it stamps the ref it built and skips the compile when a restored cache
#     already matches, which is what keeps this off the critical path.
#
# The apt install is NOT part of what the stamp skips. behaviortree_ros2
# re-exports dependencies of its own -- parameter_traits, from
# generate_parameter_library -- and find_package() resolves those in whoever
# consumes it. A cache carries the install tree across runs but not the apt
# packages, so skipping this on a cache hit breaks the consumer's configure
# step, not this script's.
#
#   UNDERLAY_WS     where to build (default /opt/underlay_ws)
#   SKIP_ROSDEP=1   don't touch apt; assume dependencies are present
#   ROSDEP_READY=1  apt lists and the rosdep cache are already up to date
set -euo pipefail

UNDERLAY_WS="${UNDERLAY_WS:-/opt/underlay_ws}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

BTCPP_ROS2_REPO="https://github.com/BehaviorTree/BehaviorTree.ROS2.git"
BTCPP_ROS2_BRANCH="humble"
# Head of humble on 2025-11-25. To bump: pick a commit on the branch above and
# paste its full sha here; the cache key follows this file, so CI rebuilds.
BTCPP_ROS2_REF="6c6aa078ee7bc52fec98984bed4964556abf5beb"

SRC="$UNDERLAY_WS/src/BehaviorTree.ROS2"
STAMP="$UNDERLAY_WS/install/.underlay-ref"

if [[ ! -d "$SRC/.git" ]]; then
    echo "==> cloning BehaviorTree.ROS2"
    rm -rf "$SRC"
    mkdir -p "$(dirname "$SRC")"
    git clone --quiet --branch "$BTCPP_ROS2_BRANCH" "$BTCPP_ROS2_REPO" "$SRC"
fi
git -C "$SRC" checkout --quiet --detach "$BTCPP_ROS2_REF" 2>/dev/null || {
    git -C "$SRC" fetch --quiet origin "$BTCPP_ROS2_BRANCH"
    git -C "$SRC" checkout --quiet --detach "$BTCPP_ROS2_REF"
}

# ROS's setup.bash reads variables it hasn't set yet.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if [[ "${SKIP_ROSDEP:-0}" != "1" ]]; then
    echo "==> installing underlay dependencies"
    if [[ "${ROSDEP_READY:-0}" != "1" ]]; then
        apt-get update
        [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || rosdep init
        rosdep update --rosdistro "$ROS_DISTRO"
    fi
    rosdep install --from-paths "$SRC" --ignore-src -y --rosdistro "$ROS_DISTRO"
fi

if [[ -f "$STAMP" ]] && [[ "$(cat "$STAMP")" == "$BTCPP_ROS2_REF" ]]; then
    echo "==> underlay already built at $BTCPP_ROS2_REF"
    exit 0
fi

# btcpp_ros2_samples is not a dependency of anything here, and building it
# would roughly double this step.
echo "==> building underlay"
colcon build --packages-up-to behaviortree_ros2 \
    --base-paths "$SRC" \
    --build-base "$UNDERLAY_WS/build" \
    --install-base "$UNDERLAY_WS/install"

echo "$BTCPP_ROS2_REF" > "$STAMP"
