SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

for f in $SCRIPT_DIR/../build/robot_server_lib/test_*; do
  if [[ -x "$f" ]]; then
    echo "Running $f"
    "$f"
  fi
done