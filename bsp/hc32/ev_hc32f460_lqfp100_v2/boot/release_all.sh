#!/usr/bin/env bash
# Build BL2, bind its digest into BL1, build App, then create an L0 release.
set -euo pipefail

usage() {
  echo "Usage: $0 --version X.Y.Z --build-id 32_HEX --private-key FILE --public-key FILE [--uv4 FILE]"
}

VERSION=""
BUILD_ID=""
PRIVATE_KEY=""
PUBLIC_KEY=""
UV4="${UV4:-/d/soft/codeSoft/Keil_v5/UV4/UV4.exe}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version) VERSION="$2"; shift 2 ;;
    --build-id) BUILD_ID="$2"; shift 2 ;;
    --private-key) PRIVATE_KEY="$2"; shift 2 ;;
    --public-key) PUBLIC_KEY="$2"; shift 2 ;;
    --uv4) UV4="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done

[[ -n "$VERSION" && "$BUILD_ID" =~ ^[0-9a-fA-F]{32}$ ]] || { usage; exit 2; }
[[ -f "$PRIVATE_KEY" && -f "$PUBLIC_KEY" && -f "$UV4" ]] || { echo "密钥或 Keil UV4.exe 路径不存在" >&2; exit 2; }

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
mkdir -p build/ntn-ota

build() {
  local project="$1" target="${1%.uvprojx}" log="build/ntn-ota/release-${1%.uvprojx}.log"
  # Some MDK installations leave their batch process resident after logging the
  # result. Track only this child and end it after its result appears.
  rm -f "$log"
  "$UV4" -b "$project" -q -j0 -t "$target" -o "$log" &
  local pid=$!
  for ((second = 0; second < 600; second++)); do
    if [[ -f "$log" ]] && grep -qE '[0-9]+ Error\(s\)' "$log"; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
      grep -q "0 Error(s)" "$log" || { echo "$project 编译失败，查看 $log" >&2; exit 1; }
      return
    fi
    sleep 1
  done
  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  echo "$project 在 600 秒内没有生成构建结果，查看 $log" >&2
  exit 1
}

python ./boot/prepare_mdk.py --build-id "$BUILD_ID" --public-key "$PUBLIC_KEY"
build ntn-ota-bl2.uvprojx
python ./boot/package_image.py extract-bl2
python ./boot/prepare_mdk.py --bl2-bin ./build/ntn-ota/bl2.bin --watchdog
build ntn-ota-bl1.uvprojx
build ntn-ota-app.uvprojx
python ./boot/package_image.py provision --version "$VERSION" --key "$PRIVATE_KEY" --public-key "$PUBLIC_KEY"
sha256sum build/ntn-ota/ntn-ota-l0.hex build/ntn-ota/app.signed.bin build/ntn-ota/factory.signed.bin
