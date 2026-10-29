# Decision log

One entry per choice: what was chosen, the alternatives, and why. This becomes
the methods chapter (brief Section 11). Newest at the bottom.

| Date | Decision | Alternatives considered | Reason | Evidence / link |
|---|---|---|---|---|
| 2026-10-29 | Repo layout: one `src/lidar_scanning` package with a subpackage per phase (P1–P6) | Separate repos for rig and pipeline | Keeps the regression test able to run the whole pipeline end to end | — |
| 2026-10-29 | Build against PiDAR commit `2c7e6af` (2024-03-12, "stepper fix") | Fork PiDAR now | Read-only clone until rig code starts; fork when changes need to go upstream | third_party/README.md |
| 2026-10-29 | (Cameron) Stay in Python; C++ only for the Pi LiDAR serial reader if a test shows packet loss | Rewrite the pipeline in C++ | Open3D's registration already runs in C++ (0.2–0.3 s per pair); Honeybee is Python-only; PiDAR reads serial one byte at a time, which may be too slow at 921600 baud | Conversation 2026-10-29 |
| 2026-10-29 | Develop P3/P4 code against Open3D DemoICPPointClouds until the rig produces scans | Wait for hardware | Brief risk table: develop against sample clouds during lidar delivery delay | scripts/load_sample_clouds.py |
