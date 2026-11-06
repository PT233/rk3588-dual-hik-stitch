#!/usr/bin/env bash
set -euo pipefail

: "${LEFT_SERIAL:?set LEFT_SERIAL}"
: "${RIGHT_SERIAL:?set RIGHT_SERIAL}"
: "${CALIBRATION:?set CALIBRATION}"
: "${RTSP_URL:?set RTSP_URL}"
: "${SYNC_MODE:?set SYNC_MODE to device or host}"
: "${RUNTIME_DIRECTORY:?set RUNTIME_DIRECTORY or use the systemd unit}"

capture_bin=${CAPTURE_BIN:-/usr/local/bin/dual-hik-stitch}
ffmpeg_bin=${FFMPEG_BIN:-ffmpeg}
encoder=${VIDEO_ENCODER:-h264_rkmpp}
fps=${VIDEO_FPS:-30}
read -r width height _ < "$CALIBRATION"
if [[ ! $width =~ ^[1-9][0-9]{0,3}$ || ! $height =~ ^[1-9][0-9]{0,3}$ ]] ||
   (( width % 2 || height % 2 )); then
    echo 'RTSP output requires even calibration dimensions' >&2
    exit 1
fi
if [[ $RTSP_URL != rtsp://* ]]; then
    echo 'RTSP_URL must start with rtsp://' >&2
    exit 1
fi

capture=(--left "$LEFT_SERIAL" --right "$RIGHT_SERIAL" --calibration "$CALIBRATION"
         --require-bound-calibration --stitch-threads "${STITCH_THREADS:-2}"
         --max-skew-us "${MAX_SKEW_US:-1000}" --camera-timeout-ms "${CAMERA_TIMEOUT_MS:-2000}"
         --pair-timeout-ms "${PAIR_TIMEOUT_MS:-30000}" --write-timeout-ms "${WRITE_TIMEOUT_MS:-2000}")
case "$SYNC_MODE" in
    device) capture+=(--device-clock-hz "${DEVICE_CLOCK_HZ:?set the verified camera clock frequency}") ;;
    host) ;;
    *) echo 'SYNC_MODE must be device or host' >&2; exit 1 ;;
esac
if [[ -n ${TRIGGER_SOURCE:-} ]]; then capture+=(--trigger "$TRIGGER_SOURCE"); fi
if [[ -n ${LEFT_PROFILE:-} ]]; then capture+=(--left-profile "$LEFT_PROFILE"); fi
if [[ -n ${RIGHT_PROFILE:-} ]]; then capture+=(--right-profile "$RIGHT_PROFILE"); fi
if [[ -n ${CAMERA_AUTO_EXPOSURE:-} ]]; then capture+=(--camera-auto-exposure "$CAMERA_AUTO_EXPOSURE"); fi
for setting in AUTO_EXPOSURE AUTO_COLOR GRAYSCALE; do
    case "${!setting:-0}" in
        0) ;;
        1) flag=${setting,,}; capture+=("--${flag//_/-}") ;;
        *) echo "$setting must be 0 or 1" >&2; exit 1 ;;
    esac
done

case "$encoder" in
    h264_rkmpp) pixel_format=nv12 ;;
    libx264) pixel_format=yuv420p ;;
    *) echo 'VIDEO_ENCODER must be h264_rkmpp or libx264' >&2; exit 1 ;;
esac

work=$(mktemp -d "${RUNTIME_DIRECTORY}/video.XXXXXX")
capture_pid= encoder_pid=
cleanup() {
    if [[ -n $capture_pid ]]; then kill "$capture_pid" 2>/dev/null || true; fi
    if [[ -n $encoder_pid ]]; then kill "$encoder_pid" 2>/dev/null || true; fi
    rm -f -- "$work/frames"
    rmdir -- "$work"
}
trap cleanup EXIT
trap 'exit 0' INT TERM
mkfifo -m 600 -- "$work/frames"

"$capture_bin" "${capture[@]}" > "$work/frames" &
capture_pid=$!
"$ffmpeg_bin" -nostdin -hide_banner -loglevel warning -use_wallclock_as_timestamps 1 \
    -f rawvideo -pixel_format bgr24 -video_size "${width}x${height}" -framerate "$fps" \
    -i pipe:0 -an -vf "format=$pixel_format" -c:v "$encoder" -b:v "${VIDEO_BITRATE:-4M}" \
    -fps_mode vfr -f rtsp -rtsp_transport tcp "$RTSP_URL" < "$work/frames" &
encoder_pid=$!

while kill -0 "$capture_pid" 2>/dev/null && kill -0 "$encoder_pid" 2>/dev/null; do
    sleep 0.2
done
status=0
if kill -0 "$capture_pid" 2>/dev/null; then
    wait "$encoder_pid" || status=$?
    encoder_pid=
else
    wait "$capture_pid" || status=$?
    capture_pid=
fi
echo "video worker exited (status=$status)" >&2
exit 1
