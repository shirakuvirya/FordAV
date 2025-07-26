# AVData ROS 2 Port

**AVData** is a collection of ROS 2 packages originally developed by Ford to drive and visualize their Multi‑AV Seasonal Dataset. This repository contains a complete port of the original ROS 1 AVData code to ROS 2 (tested on Humble and Rolling, but compatible with Jazzy, etc.).

## Repository Structure

```

AVData/
├── ford\_demo/              # ROS 2 demo package: launch files, scripts, RViz configs
├── fusion\_description/     # URDF & meshes for the Ford Fusion vehicle
└── map\_loader/             # Nodes to load & publish reflectivity & point‑cloud maps

````

## Features

- **Multi‑LiDAR converter** (`ford_demo/multi_lidar_convert.launch.py`)  
  Transforms raw Velodyne packets into per‑laser pointcloud topics.

- **Reflectivity & 3D‑map loaders** (`map_loader`)  
  Dynamically load and publish tiles of your reflectivity or 3D point‑cloud map, based on vehicle pose.

- **Full demo launch** (`ford_demo/demo.launch.py`)  
  Brings up both map loaders, extrinsics broadcaster, and RViz2 with a ready‑to‑use configuration.

## Sensors

This dataset was collected on a Ford Fusion with the following suite of sensors:

- **4 × Velodyne HDL‑32E 3D Lidars** :contentReference[oaicite:0]{index=0}  
- **6 × Point Grey 1.3 MP Cameras** (roof‑mounted for 360°) :contentReference[oaicite:1]{index=1}  
- **1 × Point Grey 5 MP Dash Camera** (forward view) :contentReference[oaicite:2]{index=2}  
- **Applanix POS‑LV GNSS/INS** (IMU + RTK‑corrected GNSS) :contentReference[oaicite:3]{index=3}

## Published Topics

When you run the ROS 2 ported nodes, you’ll see (among others):

- **LiDAR conversion**  
  - `lidar_red_scan` & `lidar_red_pointcloud`  
  - `lidar_yellow_scan` & `lidar_yellow_pointcloud`  
  - `lidar_blue_scan` & `lidar_blue_pointcloud`  
  - `lidar_green_scan` & `lidar_green_pointcloud`

- **Map loaders**  
  - `/pointcloud_map` (3D point‑cloud tiles)  
  - `/reflectivity_map` (ground‑reflectivity tiles)  

- **Pose subscriber**  
  - All map loaders subscribe to `/pose_ground_truth` (6 DoF vehicle pose).

- **TF frames**  
  - Frames such as `lidar_red`, `lidar_yellow`, etc., as well as `map`.

## Ground Truth Accuracy

The provided “ground truth” trajectory comes from the **post‑processed Applanix POS‑LV solution** (POSPac MMS), which delivers:

- **Horizontal accuracy**: up to **0.020 m**  
- **Vertical accuracy**: typically within **0.030 m**  
- **High‑rate** (up to 200 Hz) IMU/GNSS fusion to bridge GNSS outages :contentReference[oaicite:4]{index=4}  

These centimetre‑level specs allow you to treat the published pose as reliable ground truth for mapping, localization, and SLAM benchmarking.

## Dataset Download

Data are hosted on Amazon S3. For maximum throughput and auto‑resume:

```bash
# Tune AWS CLI for high concurrency
aws configure set default.s3.max_concurrent_requests 20
aws configure set default.s3.multipart_threshold 64MB
aws configure set default.s3.multipart_chunksize 16MB

# Sync only .bag files for date path (e.g. 2017-10-26/V2)
aws s3 sync \
  s3://ford-multi-av-seasonal/2017-10-26/V2 \
  ~/ford_bags \
  --exclude "*" \
  --include "*.bag" \
  --no-sign-request \
  --only-show-errors
````

Or use the helper script:

```bash
chmod +x fast_ford_download.sh
./fast_ford_download.sh 2017-10-26/V2 ~/ford_bags
```

## Installation & Build

1. **Clone your fork** into a ROS 2 workspace:

   ```bash
   mkdir -p ~/avdata_ws/src
   cd ~/avdata_ws/src
   git clone https://github.com/ShirakuGIT/AVData.git
   ```

2. **Install dependencies**:

   ```bash
   cd ~/avdata_ws
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build**:

   ```bash
   colcon build --symlink-install
   ```

4. **Source**:

   ```bash
   source install/setup.bash
   ```

## Usage

* **Full demo**:

  ```bash
  ros2 launch ford_demo demo.launch.py \
    map_dir:=/path/to/map_folder \
    calibration_dir:=/path/to/calibration_folder
  ```

* **Point‑cloud loader**:

  ```bash
  ros2 run map_loader point_cloud_map_loader /path/to/3d_point_cloud_folder
  ```

* **Reflectivity loader**:

  ```bash
  ros2 run map_loader reflectivity_map_loader /path/to/ground_reflectivity_folder
  ```

* **Multi‑LiDAR converter**:

  ```bash
  ros2 launch ford_demo multi_lidar_convert.launch.py \
    calibration:=/path/to/lidarIntrinsics.yaml
  ```

## Contributing

1. Fork the repo on GitHub.
2. Create a branch:

   ```bash
   git checkout -b feature/my-feature
   ```
3. Commit & push, then open a PR.

## License

This port is under the **BSD 3‑Clause License**.
Original Ford AVData content © Ford Motor Company under their repository license.

---

*Questions or issues? Feel free to open an issue on GitHub!*
