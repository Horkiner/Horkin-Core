#!/usr/bin/env bash
# One-shot Ubuntu amd64 environment for Horkin Core.
# Toolchain (cmake, g++, pkg-config) plus the libraries CMake looks up
# when configuring Pinocchio 2.7.0. Pinocchio and hpp-fcl stay pinned;
# do not install 3.x / 4.x.
#
#   ./scripts/setup_deps.sh
#   source scripts/env.sh
#   cmake -S . -B build

set -euo pipefail

PINOCCHIO_VER="2.7.0"
HPP_FCL_VER="2.4.4"
PREFIX="/opt/openrobots"

if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "This script pins robotpkg amd64 binaries (Ubuntu x86_64)." >&2
  echo "On aarch64/Jetson, install matching Pinocchio ${PINOCCHIO_VER} for that arch." >&2
  exit 1
fi

if ! command -v apt-get >/dev/null; then
  echo "Need apt (Debian/Ubuntu)." >&2
  exit 1
fi

if ! command -v sudo >/dev/null; then
  echo "Need sudo to install packages." >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v curl >/dev/null; then
  echo "==> Installing curl"
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl
fi

# shellcheck disable=SC1091
source /etc/os-release
CODENAME="${VERSION_CODENAME:-}"
if [[ -z "${CODENAME}" ]]; then
  echo "Could not read VERSION_CODENAME from /etc/os-release." >&2
  exit 1
fi

echo "==> Ubuntu ${CODENAME}  Pinocchio ${PINOCCHIO_VER}  hpp-fcl ${HPP_FCL_VER}"

if [[ ! -f /etc/apt/keyrings/robotpkg.asc ]]; then
  echo "==> Adding robotpkg apt key"
  sudo mkdir -p /etc/apt/keyrings
  curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc >/dev/null
fi

if [[ ! -f /etc/apt/sources.list.d/robotpkg.list ]]; then
  echo "==> Adding robotpkg apt source"
  echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub ${CODENAME} robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list >/dev/null
fi

echo "==> Pinning apt versions (block 3.x/4.x upgrades)"
sudo cp "${SCRIPT_DIR}/apt-preferences-horkin" /etc/apt/preferences.d/horkin

sudo apt-get update

# Do not grep -q a pipeline under pipefail: grep exits at the first hit,
# apt-cache then dies with SIGPIPE, and the version looks missing.
has_exact_version() {
  local pkg="$1" ver="$2" listing
  listing="$(apt-cache show "${pkg}" 2>/dev/null || true)"
  grep -Fqx "Version: ${ver}" <<<"${listing}"
}

if ! has_exact_version robotpkg-pinocchio "${PINOCCHIO_VER}"; then
  echo "robotpkg-pinocchio ${PINOCCHIO_VER} is not in the apt index for ${CODENAME}." >&2
  apt-cache policy robotpkg-pinocchio >&2 || true
  exit 1
fi
if ! has_exact_version robotpkg-hpp-fcl "${HPP_FCL_VER}"; then
  echo "robotpkg-hpp-fcl ${HPP_FCL_VER} is not in the apt index for ${CODENAME}." >&2
  apt-cache policy robotpkg-hpp-fcl >&2 || true
  exit 1
fi

echo "==> Installing toolchain, CMake dependencies, and pinned robotpkg packages"
sudo apt-get install -y \
  build-essential \
  ca-certificates \
  cmake \
  curl \
  pkg-config \
  libeigen3-dev \
  liburdfdom-dev \
  liburdfdom-headers-dev \
  libconsole-bridge-dev \
  libtinyxml-dev \
  libassimp-dev \
  libqhull-dev \
  liboctomap-dev \
  libboost-filesystem-dev \
  libboost-serialization-dev \
  libboost-system-dev \
  libboost-chrono-dev \
  "robotpkg-pinocchio=${PINOCCHIO_VER}" \
  "robotpkg-hpp-fcl=${HPP_FCL_VER}"

sudo apt-mark hold robotpkg-pinocchio robotpkg-hpp-fcl

echo "==> Verifying"
for cmd in cmake c++ pkg-config; do
  if ! command -v "${cmd}" >/dev/null; then
    echo "Missing ${cmd} after install." >&2
    exit 1
  fi
done

export PKG_CONFIG_PATH="${PREFIX}/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
GOT="$(pkg-config --modversion pinocchio)"
if [[ "${GOT}" != "${PINOCCHIO_VER}" ]]; then
  echo "pkg-config reported pinocchio ${GOT}, expected ${PINOCCHIO_VER}" >&2
  exit 1
fi
test -f "${PREFIX}/lib/cmake/pinocchio/pinocchioConfig.cmake"

eigen_cfg=""
for f in \
  /usr/share/eigen3/cmake/Eigen3Config.cmake \
  /usr/lib/cmake/eigen3/Eigen3Config.cmake
do
  if [[ -f "${f}" ]]; then
    eigen_cfg="${f}"
    break
  fi
done
if [[ -z "${eigen_cfg}" ]]; then
  echo "Eigen3Config.cmake not found. libeigen3-dev did not install it." >&2
  exit 1
fi

echo
echo "OK: Pinocchio ${GOT} at ${PREFIX}"
echo "In this shell (or add to ~/.bashrc):"
echo "  source ${SCRIPT_DIR}/env.sh"
echo "Then:"
echo "  cmake -S . -B build"
echo "  cmake --build build"
