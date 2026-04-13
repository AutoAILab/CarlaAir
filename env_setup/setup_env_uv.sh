#!/bin/bash
# ============================================================
#  CarlaAir v0.1.7 — One-click Environment Setup
#  Usage: bash setup_env.sh
# ============================================================
set -e

ENV_NAME="carlaAir"
PYTHON_VER="3.10"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TARBALL="$SCRIPT_DIR/carla_python_module.tar.gz"

# ---------- colors ----------
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
warn() { echo -e "${YELLOW}[!!]${NC} $1"; }
fail() { echo -e "${RED}[FAIL]${NC} $1"; exit 1; }

echo ""
echo "=========================================="
echo "  CarlaAir v0.1.7 Environment Setup"
echo "=========================================="
echo ""

# ---------- 1. Find uv ----------
UV_EXE=""
if command -v uv &>/dev/null; then
    UV_EXE="uv"
elif [ -x "$HOME/.cargo/bin/uv" ]; then
    UV_EXE="$HOME/.cargo/bin/uv"
fi
[ -z "$UV_EXE" ] && fail "uv not found. Install uv first: https://docs.astral.sh/uv/"
ok "uv found: $UV_EXE"

# ---------- 2. Check tarball ----------
[ ! -f "$TARBALL" ] && fail "carla_python_module.tar.gz not found in $SCRIPT_DIR"
ok "carla module tarball found ($(du -h "$TARBALL" | cut -f1))"

# ---------- 3. Create or reuse uv env ----------
VENV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/.venv"
if [ -d "$VENV_DIR" ]; then
    warn "uv env '.venv' already exists, will update it"
else
    echo "Creating uv env '.venv' (python $PYTHON_VER)..."
    "$UV_EXE" venv "$VENV_DIR" --python "$PYTHON_VER" -q
    ok "uv env '.venv' created"
fi

source "$VENV_DIR/bin/activate"
ok "activated env: .venv ($(python3 --version))"

# ---------- 4. Install pip packages ----------
echo "Installing pip dependencies..."
"$UV_EXE" pip install -q numpy setuptools msgpack-rpc-python
"$UV_EXE" pip install -q pygame airsim Pillow --no-build-isolation
ok "pip packages installed via uv"

# ---------- 5. Remove official carla if present ----------
# Remove from conda env
if pip show carla &>/dev/null; then
    warn "Removing pip-installed carla from conda env (incompatible with CarlaAir)..."
    pip uninstall carla -y -q
    ok "official carla package removed from conda env"
fi
# Also remove from --user site if present (it shadows conda packages)
LOCAL_SITE="$HOME/.local/lib/python${PYTHON_VER}/site-packages"
if [ -d "$LOCAL_SITE/carla" ] && pip show --user carla &>/dev/null 2>&1; then
    warn "Removing pip-installed carla from ~/.local ..."
    pip uninstall carla -y -q 2>/dev/null || true
fi

# ---------- 6. Install CarlaAir carla module ----------
SITE_PKG="$(python3 -c 'import site; print(site.getsitepackages()[0])')"

# Clean old carla/ and carla.libs/ in target
for d in "$SITE_PKG/carla" "$SITE_PKG/carla.libs"; do
    [ -d "$d" ] && rm -rf "$d"
done

echo "Installing CarlaAir carla module to $SITE_PKG ..."
tar xzf "$TARBALL" -C "$SITE_PKG"
ok "carla + carla.libs installed ($(ls "$SITE_PKG/carla.libs/" | wc -l) shared libs)"

# ---------- 7. Verify ----------
echo ""
echo "Verifying imports..."

VERIFY_OK=true

python3 -c "
import carla
print('  carla loaded from:', carla.__file__)
c = carla.Client('localhost', 9999)
print('  client version:', c.get_client_version())
" 2>/dev/null && ok "carla verified" || {
    # Import might work even if no server — test import alone
    python3 -c "import carla; print('  carla import OK')" 2>&1 && ok "carla import OK (no server running)" || { fail "carla import failed!"; VERIFY_OK=false; }
}

python3 -c "import airsim; print('  airsim OK')" && ok "airsim verified" || { warn "airsim import failed"; VERIFY_OK=false; }
python3 -c "import pygame" 2>/dev/null && ok "pygame verified" || { warn "pygame import failed"; VERIFY_OK=false; }
python3 -c "import numpy" && ok "numpy verified" || { warn "numpy import failed"; VERIFY_OK=false; }
python3 -c "import PIL" && ok "Pillow verified" || { warn "Pillow import failed"; VERIFY_OK=false; }

# ---------- Done ----------
echo ""
if $VERIFY_OK; then
    echo "=========================================="
    echo -e "  ${GREEN}Setup complete!${NC}"
    echo "=========================================="
else
    echo "=========================================="
    echo -e "  ${YELLOW}Setup complete with warnings${NC}"
    echo "=========================================="
fi
echo ""
echo "  Usage:"
echo "    source .venv/bin/activate"
echo "    cd $SCRIPT_DIR"
echo "    ./CarlaAir.sh                        # Start simulator"
echo "    python3 examples/record_drone.py"
echo ""
