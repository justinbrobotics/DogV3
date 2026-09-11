# Run DogV3

Start with [COMMISSIONING.md](COMMISSIONING.md) for hardware, wiring and the full setup sequence. The supported release route is **Windows setup → Raspberry Pi onboard controls → ESP32 USB mux → two servo buses**.

## Windows: three launchers

1. Install Python **3.11 or newer**, with the Python launcher or Python on PATH.
2. Double-click **Install-Windows.bat**. Internet access is needed to install the declared dependencies into `.venv`.
3. Double-click **Commission-Windows.bat** to set up the servo IDs, centers, directions and limits. Leave the serial port blank for a dry preview.
4. Double-click **Operate-Windows.bat**. Its default is a dry preview; the menu also opens the onboard Pi or starts explicitly selected USB bench controls.

These are readable executable launch scripts. Installation also creates Windows console `.exe` entry points such as `.venv\Scripts\dogv3-setup.exe`. There is no opaque, bundled application binary to keep synchronized with the source.

Keep the command window open while using a local browser interface. Close it to stop that local server. Opening the Pi interface only opens a browser; the Pi runs its own service.

## Command line

```powershell
# From software/, after running Install-Windows.bat:
.\.venv\Scripts\python.exe start.py commission
.\.venv\Scripts\python.exe start.py operate
.\.venv\Scripts\python.exe start.py check

# Explicit first flash; asks for the verified port and confirmation:
.\.venv\Scripts\python.exe start.py flash

# Offline preflight tests; no real robot is connected by these tests:
.\.venv\Scripts\python.exe -m pytest -q
```

`start.py init` creates `robot_config.json` from the template and generates a random control token locally. It never overwrites an existing configuration. **Keep this file private:** it contains that builder's credentials and calibration. The example is intentionally uncommissioned and has no password or control token. The installer does not connect, flash, arm or move hardware.

## What belongs in this folder

| Path | Purpose |
| --- | --- |
| `dogv3/` | Python runtime, commissioning UI and the modules those interfaces import |
| `firmware/dogv3_mux/` | Source for the classic ESP32 USB-to-dual-UART bridge |
| `deploy/pi/` | Fresh Pi installer and four service definitions |
| `tests/` | Small offline preflight suite used before a firmware upload |
| `robot_config.example.json` | Measured geometry and bus conventions; no servo calibration or credentials |
| `simulation_params.example.json`, `trot_candidates.example.json` | Nonprivate inputs used by retained setup/analysis tools |
| `start.py` and the `.bat` files | Portable launchers |

Some `simulation/` modules are required by the operator UI's preview and gait analysis. Their presence does not mean a physics simulator must be installed to drive the robot. Standalone simulation launchers and the personal Pi updater are omitted.

Edit the Python or firmware source directly; the editable installation uses those files. Core motion, direction conventions, disarm checks and firmware protocol are retained from CODEBASE. Public-copy changes are the DogV3 package/import/command/service names and firmware banner, clean configuration template, generic Pi hostname, portable launchers and provisioning, and removal of the preconfigured Wi-Fi build/password. The USB build is the intended Pi route.

Original software uses [PolyForm Noncommercial 1.0.0](LICENSE.md), allowing modification and sharing under its noncommercial terms. Third-party browser code and dependencies retain their own licenses; see [THIRD_PARTY.md](THIRD_PARTY.md).

All 149 retained offline tests passed after the DogV3 rename. The USB ESP32 firmware compiled, and a Python wheel built with its browser assets and console entry points. Desktop dependencies and the ESP32 platform are pinned to those checked versions. The software folder passed the publication-file/credential scan. A fresh Raspberry Pi installation and physical commissioning have not been performed for this release.
