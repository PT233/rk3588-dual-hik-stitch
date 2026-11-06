#!/usr/bin/env bash
set -euo pipefail

runner=$1
work=$(mktemp -d)
trap 'rm -f -- "$work/fixture.calib" "$work/capture" "$work/encoder" "$work/log"; rmdir -- "$work/runtime" "$work"' EXIT
mkdir "$work/runtime"
printf '640 480 0 0\n' > "$work/fixture.calib"
printf '#!/usr/bin/env bash\nexit 23\n' > "$work/capture"
printf '#!/usr/bin/env bash\nexec sleep 30\n' > "$work/encoder"
chmod +x "$work/capture" "$work/encoder"

status=0
LEFT_SERIAL=left RIGHT_SERIAL=right CALIBRATION="$work/fixture.calib" \
RTSP_URL=rtsp://127.0.0.1:8554/stitch SYNC_MODE=host RUNTIME_DIRECTORY="$work/runtime" \
CAPTURE_BIN="$work/capture" FFMPEG_BIN="$work/encoder" bash "$runner" > "$work/log" 2>&1 || status=$?
if [[ $status != 1 ]] || ! grep -q 'video worker exited (status=23)' "$work/log"; then
    cat "$work/log" >&2
    exit 1
fi
if [[ -n $(ls -A "$work/runtime") ]]; then
    echo 'supervisor left a runtime FIFO behind' >&2
    exit 1
fi
