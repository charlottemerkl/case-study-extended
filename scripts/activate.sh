#!/usr/bin/env bash

# This script must be sourced:
# source scripts/activate.sh

export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Activate Python environment
source "$PROJECT_ROOT/.venv/bin/activate"

# acados
export ACADOS_SOURCE_DIR="$PROJECT_ROOT/external/acados"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"

echo "MPC environment activated"
echo "PROJECT_ROOT=$PROJECT_ROOT"
echo "ACADOS_SOURCE_DIR=$ACADOS_SOURCE_DIR"