"""Pi-side onboard services: camera, lidar, audio, IMU.

These run on the Raspberry Pi 3B+ as systemd units (see deploy/pi/). All
hardware imports are lazy so the package installs and tests run on any
machine; every service has a --dry mode that needs no hardware.
"""
