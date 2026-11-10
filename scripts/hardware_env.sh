#!/usr/bin/env bash
# Source in each hardware shell. Does not start drivers or enable control.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo 'Use: source scripts/hardware_env.sh' >&2
    exit 2
fi
USV_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export USV_WORKSPACE
if [[ -f "$USV_WORKSPACE/.env.boat" ]]; then
    source "$USV_WORKSPACE/.env.boat" || return 1
fi
for USV_SETUP_PATH in /opt/ros/humble/setup.bash \
    "${USV_LIVOX_SETUP:-$HOME/robotx_ws/install/setup.bash}" \
    "$USV_WORKSPACE/.venv/bin/activate" "$USV_WORKSPACE/install/local_setup.bash"; do
    if [[ ! -f "$USV_SETUP_PATH" ]]; then
        echo "Missing $USV_SETUP_PATH; complete the hardware build first." >&2
        unset USV_SETUP_PATH
        return 1
    fi
done
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash || return 1
# shellcheck source=/dev/null
source "${USV_LIVOX_SETUP:-$HOME/robotx_ws/install/setup.bash}" || return 1
source "$USV_WORKSPACE/.venv/bin/activate" || return 1
source "$USV_WORKSPACE/install/local_setup.bash" || return 1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
unset USV_SETUP_PATH
