# GFS-X YOLO Runtime

This folder contains the portable YOLO runtime for the штатный ноутбук.

Files:
- `install.ps1` creates or reuses `.venv`, installs dependencies, copies `config.example.yaml` to `config.yaml` if needed, and verifies `models/best_detect.pt`.
- `run_yolo.ps1` starts the YOLO bridge.
- `test_udp.ps1` sends fake UDP packets to Unity.
- `config.example.yaml` is the tracked template. `config.yaml` is local and ignored by Git.

Model:
- `models/best_detect.pt`

Recommended Unity flow:
1. Open the project.
2. Run `Tools/GFSX YOLO/Use Simulated Vision` for training in simulation.
3. Run `Tools/GFSX YOLO/Use Real Vision` when the UDP sender is connected.
