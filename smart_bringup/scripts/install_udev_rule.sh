#!/usr/bin/env bash

# Install the chassis UART rule and refresh tty devices.
set -euo pipefail

if (( $# != 0 )); then
  echo "Usage: sudo $0" >&2
  exit 1
fi
if [[ $(id -u) != 0 ]]; then
  echo "Please run with sudo: sudo $0" >&2
  exit 1
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
install -m 0644 -- "${script_dir}/../udev/smart_base.rules" \
  /etc/udev/rules.d/99-smart-base.rules
udevadm control --reload-rules
udevadm trigger --action=change --subsystem-match=tty
udevadm settle --timeout=10

echo "Installed chassis udev rule. Connect the adapter and check: ls -l /dev/smart_base"
