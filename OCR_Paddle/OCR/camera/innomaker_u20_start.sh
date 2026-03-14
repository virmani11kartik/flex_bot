#!/usr/bin/env bash
set -euo pipefail

# Prefer persistent device path (recommended)
if ls /dev/v4l/by-id/*InnoMaker*U20-16MP-AF*video-index0 >/dev/null 2>&1; then
  DEV="$(ls -1 /dev/v4l/by-id/*InnoMaker*U20-16MP-AF*video-index0 | head -n 1)"
else
  # Fallback (your current mapping)
  DEV="/dev/video2"
fi

# --------- PRESET: pick ONE ---------
# Best quality + real-time (recommended)
W=3264; H=2448; FPS=30

# SLAM / low-latency alternative:
# W=1920; H=1080; FPS=30

# Full-res still/preview (heavy):
# W=4656; H=3496; FPS=15
# ------------------------------------

echo "[InnoMaker] Using device: ${DEV}"
echo "[InnoMaker] Setting: ${W}x${H} @ ${FPS}fps, MJPEG + AF + AE"

# Set format + FPS (MJPEG is what gives you high FPS)
v4l2-ctl -d "${DEV}" --set-fmt-video=width=${W},height=${H},pixelformat=MJPG
v4l2-ctl -d "${DEV}" --set-parm=${FPS}

# Autofocus + auto exposure (names depend on firmware; ignore failures safely)
v4l2-ctl -d "${DEV}" --set-ctrl=focus_auto=1 2>/dev/null || true
# v4l2-ctl -d "${DEV}" --set-ctrl=exposure_auto=1 2>/dev/null || true
# v4l2-ctl -d "${DEV}" --set-ctrl=white_balance_temperature_auto=0 2>/dev/null || true

# Optional: small “sane” defaults (comment out if you want pure auto)
v4l2-ctl -d "${DEV}" --set-ctrl=sharpness=80 2>/dev/null || true

# Print summary (helpful debug)
v4l2-ctl -d "${DEV}" --get-fmt-video || true
v4l2-ctl -d "${DEV}" --get-parm || true

echo "[InnoMaker] Done."

if [[ "${1:-}" == "preview" ]]; then
  echo "[InnoMaker] Starting live preview…"
  exec gst-launch-1.0 v4l2src device="${DEV}" \
    ! image/jpeg,width=${W},height=${H},framerate=${FPS}/1 \
    ! jpegdec ! videoconvert ! autovideosink
fi