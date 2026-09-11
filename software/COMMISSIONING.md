# Commission a matching DogV3

This is a source-grounded setup guide for the documented hardware baseline. A new robot must be commissioned individually even when it uses the same CAD and servo model. The owner's current local source is included; this release has not been installed and physically commissioned on a fresh robot.

## 1. Match the hardware and wiring

The current code documents a Raspberry Pi 3B+, a classic ESP32 DevKit/WROOM board, 12 Feetech STS3215-C047 12 V bus servos and two Waveshare bus adapters. The exact ESP32 carrier and adapter revision must be checked against the physical boards. The older BOM disagrees with the newer code about the IMU; this software's Pi IMU implementation is for **BNO085**, not MPU6050. See the [BOM](../BOM.md).

| Connection | Current firmware/code definition |
| --- | --- |
| Pi to ESP32 | USB serial, 1,000,000 baud |
| Bus A: rear legs BL and BR | ESP32 RX GPIO16, TX GPIO17, 1,000,000 baud |
| Bus B: front legs FL and FR | ESP32 RX GPIO26, TX GPIO25, 1,000,000 baud |
| Host coordinates | +Y forward; retain the supplied side and knee-sign conventions |
| Optional BNO085 on Pi | I2C SDA GPIO2 / SCL GPIO3; enable only after matching the sensor and mounting orientation |
| Optional camera | OV5647 CSI camera; the supplied Pi camera service uses picamera2 |
| Optional lidar | RPLIDAR C1, USB serial at 460800 baud |
| Optional audio | USB audio device, ALSA and espeak-ng |

The GPIO table identifies the **software endpoints**, not a universal pin-to-pin adapter harness. Confirm the adapter's TX/RX labels, voltage levels, power input and ground from the exact vendor board documentation before wiring. The repository does not establish a complete wiring diagram, fuse values, battery connector or current-rated power harness. Do not guess these from the CAD models.

The robot's documented build uses one switch powering everything. Use a stable support with all feet clear, keep the switch reachable and work with the robot disarmed. Rebooting or stopping controls can release torque and let the body fall. The initial template leaves optional peripherals disabled.

The public firmware now identifies itself as `DOGV3-MUX v1.0`, and the host expects that same banner. For an existing robot, preserve its private calibration first and use the documented explicit flash path before connecting the renamed host. This public-package change does not flash or change the owner's deployed robot.

## 2. Install and identify the ESP32

Run **Install-Windows.bat**. Connect the identified ESP32 by USB and read its COM port from Windows Device Manager. The firmware target `esp32dev` is for a classic ESP32; do not use it for a different ESP32 family without changing and verifying the board target.

From `software/`, run:

```powershell
.\.venv\Scripts\python.exe start.py flash
```

Enter the verified port and explicitly confirm the flash. The existing flash tool runs offline preflight tests, builds the USB firmware, uploads to the selected port, checks VERSION/PING/SCAN, then performs a no-motion bench check that writes torque-off to discovered servos. This workflow is an intentional hardware operation. Firmware and physical link success must be observed on that board.

## 3. Commission every joint

1. New servos can share a factory ID. Connect **one servo at a time** for ID assignment. The commissioning GUI has a guarded **Set a servo ID** utility; number 1–12 across the robot for clarity. Do not assign IDs to an entire chain of duplicates at once.
2. Open **Commission-Windows.bat**, enter the ESP32 port, click **Run verify_firmware**, then **Scan both buses**.
3. Confirm Bus A contains the six rear joints and Bus B the six front joints. For each detected servo, use the deliberate **Wiggle** action and map the observed joint to the body diagram.
4. Select **Torque off**, move that joint to its actual mechanical neutral and use **Set center**. Confirm the displayed position is around 2048.
5. Use **Probe** for a small direction check. A positive hip probe moves outward, a positive femur probe moves forward, and a positive tibia probe flexes the knee. Choose **Correct** or **Invert** based on physical observation. Do not change the left/right side flag to compensate for reversed motor direction.
6. With torque off, capture safe min/max positions inside the physical stops. Set the soft range. EEPROM limit writes are an explicit separate choice.
7. Verify all 12 joints have their Assigned, Center, Direction and Range checks complete, then select **Emit robot_config.json**.

```powershell
.\.venv\Scripts\python.exe start.py check
```

Resolve errors and assess warnings. A readiness pass confirms the software's checks; it does not prove balance, clearances or walking performance. The starting gait parameters are unvalidated defaults for the new build, not a promise of a stable gait.

## 4. Prepare the onboard Pi

The deployment scripts target **Raspberry Pi OS Bookworm Lite 64-bit**, Python 3.11+, a user named `pi`, and the software directory at `/home/pi/dogv3`. Set a unique password/SSH key and your own Wi-Fi in Raspberry Pi Imager. Set hostname `dogv3`, or change `network.pi_host` in the local configuration to your chosen name/address.

Copy the **contents** of `software/` into `/home/pi/dogv3`; do not copy a Windows `.venv`. The robot must remain supported and disarmed. On the Pi:

```bash
cd /home/pi/dogv3
bash deploy/pi/install.sh
```

The installer creates `/home/pi/dogv3-venv` with access to apt's picamera2, installs dependencies and the service files, and enables I2C. It leaves the robot services stopped and does not install automatic serial-device rules. Log out and back in for serial/I2C/audio/video group membership.

Privately copy the **commissioned** Windows `robot_config.json` to `/home/pi/dogv3/robot_config.json`, preserving its generated token. Back up any existing commissioned Pi configuration first. Keep this local configuration out of GitHub, MakerWorld, issue attachments and shared archives.

## 5. Assign the actual serial device

With the ESP32 and lidar identifiable, inspect:

```bash
ls -l /dev/serial/by-id/ /dev/serial/by-path/
```

Unplug/replug the identified ESP32 while disarmed if needed to distinguish it from the lidar. The original build's assumed CH340/CP2102 identities are **not** installed as generic rules: an ESP32 and lidar can use the same converter.

Edit the operator service:

```bash
sudo nano /etc/systemd/system/dogv3-operate.service
```

Replace `/dev/dogv3-esp32` in `ExecStart` with the verified persistent `/dev/serial/by-id/...` path. If the device has no unique serial, use its verified fixed `/dev/serial/by-path/...` path. Do not substitute a guessed `/dev/ttyUSB0`.

```bash
sudo systemctl daemon-reload
/home/pi/dogv3-venv/bin/python start.py check
sudo systemctl start dogv3-operate
systemctl status dogv3-operate --no-pager
```

If there is a configuration, port or firmware mismatch, resolve it while supported before using ARM. Once tested, `sudo systemctl enable dogv3-operate` makes the service start on boot. Service startup does not replace commissioning or deliberately arming in the UI.

## 6. Add peripherals, then make the first supported test

Enable and configure each installed peripheral in the local JSON only after identifying it. For lidar, set `peripherals.lidar.device` to its verified persistent USB path. For the IMU, confirm BNO085 address and mounting transform before enabling its correction. Camera, lidar and audio services can then be started individually with `sudo systemctl start dogv3-camera`, `dogv3-lidar` or `dogv3-audio` as applicable.

On Windows, use **Operate-Windows.bat → Open onboard Pi**. The PC and Pi must use the same commissioned config token. Use a trusted robot LAN; these HTTP interfaces are not a public internet service. The launcher uses the token without printing it.

While the body remains supported, verify disarm/torque-off, deliberate ARM, servo position capture and small joint motions. Confirm all signs and ranges, peripheral status and the physical stop switch. Only then progress to supported stance and supervised floor tests with room around the robot. Stop for binding, unexpected movement or loss of control. Record a working stance/gait on the physical robot before advertising it as commissioned.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Launcher cannot find Python | Install Python 3.11+ and rerun Install-Windows.bat |
| Port busy during setup | Stop the operator process/service deliberately while the robot is supported |
| Pi UI asks for a key or rejects commands | Use the same private commissioned config/token on PC and Pi |
| Cannot ARM | Finish every joint's commissioning checks and inspect readiness/firmware status |
| Wrong or missing serial device | Re-identify physical adapters; do not rely on the previous robot's USB assignments |
| IMU unavailable | Confirm BNO085; MPU6050 is not a drop-in match for this code |
