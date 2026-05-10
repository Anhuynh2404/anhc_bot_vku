# Hướng dẫn sử dụng `anhc_bot`

Tài liệu này hướng dẫn chi tiết cách build, kiểm tra (verify), và chạy các thành phần của package `anhc_bot`.

---

## 1. Hướng dẫn Build (Biên dịch)

Trước tiên, đảm bảo bạn đã cài đặt các dependency cần thiết và source môi trường ROS 2 Jazzy.

```bash
# 1. Đi tới workspace
cd ~/anhc_ws

# 2. Source môi trường ROS 2
source /opt/ros/jazzy/setup.bash

# 3. Cài đặt dependencies bằng rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build package với symlink-install
colcon build --symlink-install --packages-select anhc_bot

# 5. Source môi trường workspace sau khi build
source install/setup.bash
```

---

## 2. Hướng dẫn Verify (Kiểm tra)

Sau khi build xong, bạn có thể chạy các lệnh sau để đảm bảo package đã được cấu hình và nhận diện đúng:

### 2.1 Kiểm tra ROS 2 nhận diện package
```bash
ros2 pkg list | grep anhc_bot
```
*(Nếu terminal in ra `anhc_bot` là thành công).*

### 2.2 Kiểm tra URDF hợp lệ (Xacro parse)
```bash
ros2 run xacro xacro ~/anhc_ws/src/anhc_bot/urdf/anhc_bot.xacro sim_gz:=true two_d_lidar_enabled:=true > /dev/null && echo "URDF is valid!"
```
*(Nếu terminal in ra `URDF is valid!` là thành công).*

---

## 3. Hướng dẫn Run (Khởi chạy)

Tất cả các lệnh dưới đây yêu cầu bạn đã source môi trường workspace trước khi chạy:
```bash
source ~/anhc_ws/install/setup.bash
```

Dưới đây là quy trình đầy đủ gồm **3 giai đoạn**: tạo bản đồ → lưu bản đồ → điều hướng tự động.

---

### Giai đoạn 1 — Tạo bản đồ (SLAM Toolbox)

Mở **Terminal 1** — Khởi động Gazebo simulation:
```bash
source ~/anhc_ws/install/setup.bash
ros2 launch anhc_bot simulation.launch.py \
  camera_enabled:=True \
  two_d_lidar_enabled:=True \
  world_file:=small_warehouse.sdf
```

Mở **Terminal 2** — Khởi động SLAM để vẽ bản đồ (RViz2 sẽ tự động mở):
```bash
source ~/anhc_ws/install/setup.bash
ros2 launch anhc_bot mapping.launch.py
```

Mở **Terminal 3** — Điều khiển robot đi khám phá môi trường:
```bash
source ~/anhc_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  cmd_vel:=/anhc_bot/cmd_vel
```

> Dùng phím `i/,` để tiến/lùi, `j/l` để quay. Lái robot đi khắp warehouse để SLAM vẽ đầy đủ bản đồ.

---

### Giai đoạn 2 — Lưu bản đồ

Sau khi đã lái robot khám phá hết khu vực, lưu bản đồ lại:
```bash
cd ~/anhc_ws/src/anhc_bot/config/maps
ros2 run nav2_map_server map_saver_cli -f anhc_map
```

> Lệnh này tạo ra 2 file: `anhc_map.pgm` (ảnh bản đồ) và `anhc_map.yaml` (metadata).

Sau đó **Ctrl+C** tất cả các terminal ở Giai đoạn 1.

---

### Giai đoạn 3 — Điều hướng tự động với Nav2

Mở **Terminal 1** — Khởi động lại Gazebo:
```bash
source ~/anhc_ws/install/setup.bash
ros2 launch anhc_bot simulation.launch.py \
  camera_enabled:=True \
  two_d_lidar_enabled:=True \
  world_file:=small_warehouse.sdf
```

Mở **Terminal 2** — Khởi động Nav2 (RViz2 sẽ tự động mở kèm giao diện Nav2):
```bash
source ~/anhc_ws/install/setup.bash
ros2 launch anhc_bot navigation.launch.py
```

---

### 🖱️ Cách đặt mục tiêu trong RViz2

1. Trong RViz2, click **"2D Pose Estimate"** → click vào vị trí robot đang đứng trên bản đồ để khởi tạo vị trí ban đầu
2. Click **"Nav2 Goal"** trên toolbar
3. Click và kéo chuột vào vị trí đích trên bản đồ → robot sẽ tự lên kế hoạch đường đi và di chuyển đến đó

---

### ⚠️ Lưu ý

Nếu thiếu `slam_toolbox` hoặc `nav2`, cài thêm:
```bash
sudo apt install ros-jazzy-slam-toolbox \
                 ros-jazzy-navigation2 \
                 ros-jazzy-nav2-bringup -y
```

Sau đó rebuild:
```bash
cd ~/anhc_ws
colcon build --symlink-install
source install/setup.bash
```
