#!/usr/bin/env bash
set -euo pipefail

if [[ "${INSTALL_ZED_SDK:-1}" != "1" ]]; then
  exit 0
fi

if [[ "$(dpkg --print-architecture)" != "arm64" ]]; then
  exit 0
fi

if [[ -f "/usr/local/zed/zed-config.cmake" ]]; then
  exit 0
fi

ZED_SDK_URL="${ZED_SDK_URL:-https://download.stereolabs.com/zedsdk/5.2/l4t36.4/jetsons}"

echo "[zed-sdk] Installing ZED SDK from: ${ZED_SDK_URL}"

tmp="/tmp/zed_sdk_jetson.zstd.run"
rm -f "${tmp}"
curl -L "${ZED_SDK_URL}" -o "${tmp}"
chmod +x "${tmp}"

# The installer refuses to run as root; run as an unprivileged user with passwordless sudo.
if ! id -u zedinst &>/dev/null; then
  useradd -m -s /bin/bash zedinst
  echo "zedinst ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/zedinst
  chmod 0440 /etc/sudoers.d/zedinst
fi

su - zedinst -c "sudo -E ${tmp} silent skip_tools"

rm -f "${tmp}"
rm -rf /usr/local/zed/resources/* || true

if [[ ! -f "/usr/local/zed/zed-config.cmake" ]]; then
  echo "[zed-sdk] Install finished, but /usr/local/zed/zed-config.cmake was not found."
  exit 1
fi

echo "[zed-sdk] OK: /usr/local/zed/zed-config.cmake present"

