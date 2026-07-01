#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

export GZ_PARTITION=tesi_live
export GZ_IP=192.168.0.140
export GZ_RELAY=192.168.0.49

echo " VFH Controller remoto"
echo "   ip     = $GZ_IP"
echo "   relay  = $GZ_RELAY"
echo ""

"$PROJECT_DIR/tools/standalone/build/standalone_controller" tethys_0 "$1" "$2" "$3"