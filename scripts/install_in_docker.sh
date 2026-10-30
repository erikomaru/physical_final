#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PAI_WS="${PAI_WS:-$HOME/pai_ws}"
PAI_TRAINING="${PAI_TRAINING_ROOT:-$HOME/pai_training}"
TELEOP="$PAI_WS/src/physicalai_teleop"

mkdir -p "$PAI_WS/src"
if [[ ! -d "$TELEOP/.git" ]]; then
  git clone https://github.com/matsuolab/physicalai_teleop.git "$TELEOP"
fi
if [[ ! -d "$PAI_TRAINING/.git" ]]; then
  git clone --recurse-submodules https://github.com/matsuolab/pai_training.git "$PAI_TRAINING"
else
  git -C "$PAI_TRAINING" submodule update --init --recursive
fi

link_source() {
  local source="$1" destination="$2"
  mkdir -p "$(dirname "$destination")"
  if [[ -e "$destination" && ! -L "$destination" ]]; then
    echo "Refusing to replace existing file/directory: $destination" >&2
    echo "Move it aside once, then rerun this installer." >&2
    exit 1
  fi
  ln -sfn "$source" "$destination"
}
link_source "$ROOT/ros2_ws/src/surry_hills_rover" "$PAI_WS/src/surry_hills_rover"
link_source "$ROOT/ros2_ws/src/surry_hills_rover/surry_hills_rover/teleop_rover.py" \
  "$TELEOP/physicalai_teleop/teleop_rover.py"
python3 - "$TELEOP/setup.py" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
s = p.read_text()
entry = "            'teleop_rover = physicalai_teleop.teleop_rover:main',\n"
if "teleop_rover = physicalai_teleop.teleop_rover:main" not in s:
    key = "            'teleop_keyboard = physicalai_teleop.teleop_keyboard:main',\n"
    if key not in s:
        raise SystemExit(f"Could not find expected entry point in {p}")
    p.write_text(s.replace(key, key + entry))
PY

mkdir -p "$PAI_TRAINING/scripts" "$PAI_TRAINING/configs"
link_source "$ROOT/scripts/convert_rover_bags.py" "$PAI_TRAINING/scripts/convert_rover_bags.py"
link_source "$ROOT/scripts/train_rover.py" "$PAI_TRAINING/scripts/train_rover.py"
link_source "$ROOT/scripts/infer_rover.py" "$PAI_TRAINING/scripts/infer_rover.py"
link_source "$ROOT/configs/rover_features.py" "$PAI_TRAINING/configs/rover_features.py"
link_source "$ROOT/configs/rover_train_config.yaml" "$PAI_TRAINING/configs/rover_train_config.yaml"

cd "$PAI_WS"
colcon build --symlink-install --packages-select physicalai_teleop surry_hills_rover
echo "Linked rover packages and Pai helpers to the shared project folder."
echo "Source with: source $PAI_WS/install/setup.bash"
