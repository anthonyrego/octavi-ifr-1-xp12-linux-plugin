#!/usr/bin/env bash
# Download the X-Plane SDK headers into ./sdk (only the CHeaders are needed).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SDK_URL="https://developer.x-plane.com/wp-content/plugins/code-sample-generation/sdk_zip_files/XPSDK430.zip"

cd "$ROOT"
if [ -f sdk/CHeaders/XPLM/XPLMDefs.h ]; then
  echo "SDK already present (sdk/CHeaders/XPLM)."
  exit 0
fi

echo "Downloading X-Plane SDK..."
mkdir -p downloads
curl -fsSL -o downloads/XPSDK.zip "$SDK_URL"

echo "Extracting headers..."
rm -rf downloads/sdk_tmp
unzip -q -o downloads/XPSDK.zip -d downloads/sdk_tmp
rm -rf sdk
mv downloads/sdk_tmp/SDK sdk
rm -rf downloads/sdk_tmp

echo "SDK ready at sdk/CHeaders/XPLM"
