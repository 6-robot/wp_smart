#!/usr/bin/env bash

# Install the one UART adapter rule shared by chassis, IMU and lidar traffic.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
rule_source="${script_dir}/../udev/smart_base.rules"
rules_dir="/etc/udev/rules.d"
target="${rules_dir}/99-smart-base.rules"
alias_path="/dev/smart_base"

usage()
{
  cat <<'HELP'
Usage: install_udev_rule.sh [--dry-run] [--device DEVICE]

Install udev/smart_base.rules as 99-smart-base.rules for the WCH chassis UART.
Match the vendor, product and serial number from that rule; never probe UART data.
Back up the old target and disable other local rules assigning smart_base.
Reload udev, apply the rule to the matched device and verify the alias.
An installation failure restores the previous rules. Other lidar rules are untouched.

  --dry-run        Check the rule and connected device without changing the system.
  --device DEVICE  Check this serial device instead of scanning ttyACM devices.
  -h, --help       Show this help.

Installation requires sudo. Stop any obsolete driver using the wrong port first.
HELP
}

fail()
{
  echo "Error: $*" >&2
  exit 1
}

read_rule_identity()
{
  local rule attribute expression value
  rule="$(sed '/^[[:space:]]*#/d; /^[[:space:]]*$/d' "${rule_source}")"
  [[ -n ${rule} && ${rule} != *$'\n'* ]] || fail "Expected one active rule in ${rule_source}"
  [[ ${rule} == *'KERNEL=="ttyACM*"'* && ${rule} == *'SYMLINK+="smart_base"'* ]] ||
    fail "The rule must map a ttyACM device to smart_base"
  for attribute in idVendor idProduct serial; do
    expression="ATTRS\\{${attribute}\\}==\"([A-Za-z0-9._-]+)\""
    [[ ${rule} =~ ${expression} ]] || fail "Missing or invalid ${attribute} in ${rule_source}"
    value="${BASH_REMATCH[1]}"
    case "${attribute}" in
      idVendor) expected_vendor="${value}" ;;
      idProduct) expected_product="${value}" ;;
      serial) expected_serial="${value}" ;;
    esac
  done
}

find_base_device()
{
  local candidate properties key value vendor product serial
  local -a candidates=() matches=()
  if [[ -n ${requested_device} ]]; then
    candidates=("${requested_device}")
  else
    shopt -s nullglob
    candidates=(/dev/ttyACM*)
    shopt -u nullglob
  fi
  for candidate in "${candidates[@]}"; do
    [[ -c ${candidate} ]] || continue
    properties="$(udevadm info --query=property --name="${candidate}" 2>/dev/null)" || continue
    vendor=""; product=""; serial=""
    while IFS='=' read -r key value; do
      case "${key}" in
        ID_VENDOR_ID) vendor="${value}" ;;
        ID_MODEL_ID) product="${value}" ;;
        ID_SERIAL_SHORT) serial="${value}" ;;
      esac
    done <<< "${properties}"
    if [[ ${vendor} == "${expected_vendor}" && ${product} == "${expected_product}" &&
          ${serial} == "${expected_serial}" ]]; then
      matches+=("$(readlink -f -- "${candidate}")")
    fi
  done
  (( ${#matches[@]} == 1 )) ||
    fail "Expected one ${expected_vendor}:${expected_product} adapter with serial ${expected_serial}; found ${#matches[@]}"
  base_device="${matches[0]}"
  [[ ${base_device##*/} == ttyACM* ]] || fail "The matching rule requires a ttyACM device"
}

find_conflicting_rules()
{
  local rule
  conflicting_rules=()
  shopt -s nullglob
  for rule in "${rules_dir}"/*.rules; do
    [[ ${rule} == "${target}" ]] && continue
    if sed 's/#.*//; /^[[:space:]]*$/d' "${rule}" |
       grep -Eq 'SYMLINK[[:space:]]*[+:]?=[[:space:]]*"([^"]*[[:space:]])?smart_base([[:space:]][^"]*)?"'; then
      conflicting_rules+=("${rule}")
    fi
  done
  shopt -u nullglob
}

refresh_device()
{
  udevadm control --reload-rules
  udevadm trigger --action=change --subsystem-match=tty --sysname-match="${base_device##*/}"
  udevadm settle --timeout=10
}

rollback()
{
  local result=$? index
  if (( result == 0 )) || [[ ${changes_started} != true ]]; then
    return
  fi
  trap - EXIT INT TERM
  set +e
  echo "Installation failed; restoring previous udev rules." >&2
  if [[ -n ${target_backup} ]]; then
    cp -a --remove-destination -- "${target_backup}" "${target}"
  else
    rm -f -- "${target}"
  fi
  for index in "${!disabled_originals[@]}"; do
    if [[ -e ${disabled_backups[${index}]} || -L ${disabled_backups[${index}]} ]]; then
      mv -- "${disabled_backups[${index}]}" "${disabled_originals[${index}]}"
    fi
  done
  refresh_device >/dev/null 2>&1
  exit "${result}"
}

apply_rule()
{
  local rule backup
  local stamp
  stamp="$(date +%Y%m%d-%H%M%S).$$"
  target_backup=""
  changes_started=false
  disabled_originals=()
  disabled_backups=()
  if [[ -e ${target} || -L ${target} ]]; then
    target_backup="${target}.bak.${stamp}"
    cp -a -- "${target}" "${target_backup}"
    echo "Backed up ${target} to ${target_backup}"
  fi
  trap rollback EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  changes_started=true
  for rule in "${conflicting_rules[@]}"; do
    backup="${rule}.disabled.${stamp}"
    disabled_originals+=("${rule}")
    disabled_backups+=("${backup}")
    mv -- "${rule}" "${backup}"
    echo "Disabled conflicting smart_base rule ${rule}; backup: ${backup}"
  done
  install -m 0644 -- "${rule_source}" "${target}"
  refresh_device
  if [[ ! -L ${alias_path} || $(readlink -f -- "${alias_path}") != "${base_device}" ]]; then
    fail "${alias_path} does not point to ${base_device}"
  fi
  changes_started=false
  trap - EXIT INT TERM
  echo "Installed ${target}"
  echo "${alias_path} -> ${base_device}"
}

main()
{
  local dry_run=false pids rule
  requested_device=""
  while (( $# )); do
    case "$1" in
      --dry-run) dry_run=true; shift ;;
      --device)
        (( $# >= 2 )) || fail "--device requires a path"
        requested_device="$2"; shift 2 ;;
      -h|--help) usage; return ;;
      *) usage >&2; fail "Unknown argument: $1" ;;
    esac
  done
  [[ -r ${rule_source} ]] || fail "Rule not found: ${rule_source}"
  command -v udevadm >/dev/null || fail "udevadm is required"
  if [[ ${dry_run} != true && ${EUID} -ne 0 ]]; then
    fail "Installation requires root. Run: sudo ${BASH_SOURCE[0]} (or use --dry-run)"
  fi
  read_rule_identity
  find_base_device
  find_conflicting_rules
  echo "Matched ${base_device}: ${expected_vendor}:${expected_product}, serial ${expected_serial}"
  echo "Rule: ${rule_source} -> ${target}"
  if command -v fuser >/dev/null; then
    pids="$(fuser "${base_device}" 2>/dev/null || true)"
    if [[ -n ${pids//[[:space:]]/} ]]; then
      echo "Device currently open by PID(s):${pids}; no UART reads, resets or process stops are needed."
    fi
  fi
  for rule in "${conflicting_rules[@]}"; do
    echo "Conflicting alias rule to back up and disable: ${rule}"
  done
  if [[ ${dry_run} == true ]]; then
    echo "Dry run complete. No files, rules or device events were changed."
    return
  fi
  apply_rule
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
