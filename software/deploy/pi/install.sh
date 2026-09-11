#!/usr/bin/env bash
# Fresh Pi provisioning only. Does not start robot services or change calibration.
set -euo pipefail
TASK_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TASK_VENV=/home/pi/dogv3-venv
if [[ "$(id -un)" != pi || "$TASK_REPO" != /home/pi/dogv3 ]]; then
    echo 'Run as user pi with the contents of software/ in /home/pi/dogv3.' >&2
    exit 1
fi
for unit in dogv3-operate dogv3-camera dogv3-lidar dogv3-audio; do
    if systemctl is-active --quiet "$unit"; then
        echo "Stop $unit deliberately with the robot supported before installing." >&2
        exit 1
    fi
done
sudo apt-get update
sudo apt-get install -y python3-venv python3-picamera2 espeak-ng alsa-utils i2c-tools avahi-daemon
sudo raspi-config nonint do_i2c 0
sudo systemctl enable --now avahi-daemon
if [[ ! -x "$TASK_VENV/bin/python" ]]; then
    python3 -m venv --system-site-packages "$TASK_VENV"
fi
"$TASK_VENV/bin/python" -m pip install -e "$TASK_REPO[pi]"
"$TASK_VENV/bin/python" "$TASK_REPO/start.py" init
sudo usermod -aG dialout,i2c,audio,video pi
for unit in dogv3-operate dogv3-camera dogv3-lidar dogv3-audio; do
    sudo install -m 0644 "$TASK_REPO/deploy/pi/$unit.service" "/etc/systemd/system/$unit.service"
done
sudo systemctl daemon-reload
echo 'Installed; robot services have not been enabled or started.'
echo 'Follow COMMISSIONING.md to identify serial devices and copy your commissioned configuration.'
echo 'Log out and back in for group changes. Configure hostname/network in Raspberry Pi Imager.'
