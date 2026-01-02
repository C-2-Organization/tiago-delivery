#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

REPOS_FILE="$PROJECT_ROOT/dependencies.repos"
EXTERNAL_DIR="$PROJECT_ROOT/external"
WS_DIR="$PROJECT_ROOT/ros2_ws"
WS_SRC="$WS_DIR/src"

require_cmd() {
  if ! command -v "$1" &> /dev/null; then
    echo -e "${RED}Error: '$1' not found.${NC}"
    exit 1
  fi
}

safe_link() {
  local src="$1"
  local dst="$2"

  # name collision check
  if [ -e "$dst" ] && [ ! -L "$dst" ]; then
    echo -e "${YELLOW}⚠ exists (not symlink), skip: $dst${NC}"
    return 0
  fi
  if [ -L "$dst" ]; then
    rm -f "$dst"
  fi
  ln -s "$src" "$dst"
}

echo -e "\n${YELLOW}[1/4] Fetching external dependencies...${NC}"
require_cmd vcs
cd "$PROJECT_ROOT"
mkdir -p "$EXTERNAL_DIR"

if [ ! -f "$REPOS_FILE" ]; then
  echo -e "${RED}Error: repos file not found: $REPOS_FILE${NC}"
  exit 1
fi

vcs import < "$REPOS_FILE"

echo -e "\n${YELLOW}[2/4] Setting up ROS2 workspace...${NC}"
mkdir -p "$WS_SRC"

echo -e "\n${YELLOW}[3/4] Linking ROS packages (package.xml folders only)...${NC}"

linked=0
skipped=0

# 1) local packages in this repo (interfaces, manipulation, perception/* 등)
#    -> 이미 직접 src에 있으니 여기선 건드리지 않음

# 2) external repos: find package.xml directories and link them
for repo in "$EXTERNAL_DIR"/*; do
  [ -d "$repo" ] || continue
  repo_name="$(basename "$repo")"

  # repo 안에서 package.xml이 있는 디렉토리들 찾기
  # depth는 넉넉히(6~8) 잡는 게 안전
  mapfile -t pkgs < <(find "$repo" -maxdepth 8 -name package.xml -print 2>/dev/null)

  if [ ${#pkgs[@]} -eq 0 ]; then
    echo -e "${YELLOW}⚠ no ROS packages found in $repo_name (no package.xml)${NC}"
    continue
  fi

  for pkg_xml in "${pkgs[@]}"; do
    pkg_dir="$(dirname "$pkg_xml")"
    pkg_base="$(basename "$pkg_dir")"
    dst="$WS_SRC/$pkg_base"

    # 이름 충돌이 생기면 repo명을 prefix로 붙여서 안전하게 링크
    if [ -e "$dst" ] || [ -L "$dst" ]; then
      # 이미 같은 이름이 존재하면 repo prefix 붙이기
      dst="$WS_SRC/${repo_name}__${pkg_base}"
      if [ -e "$dst" ] || [ -L "$dst" ]; then
        echo -e "${YELLOW}⚠ collision, skip: $dst${NC}"
        skipped=$((skipped+1))
        continue
      fi
    fi

    safe_link "$pkg_dir" "$dst"
    echo -e "${GREEN}✓ linked: $(basename "$dst")  <-  $repo_name/${pkg_base}${NC}"
    linked=$((linked+1))
  done
done

echo -e "${GREEN}Total linked packages: $linked${NC}"
if [ "$skipped" -gt 0 ]; then
  echo -e "${YELLOW}Skipped (collisions): $skipped${NC}"
fi

echo -e "\n${YELLOW}[4/4] Setup complete!${NC}"
echo ""
echo "Next:"
echo "  cd $WS_DIR"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo "  colcon list | head"
echo "  colcon build --symlink-install"
