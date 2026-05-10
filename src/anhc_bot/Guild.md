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
colcon build --symlink-install --packages-select anhc_astar_planner anhc_multi_planner anhc_bot

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

**Terminal 1 — Khởi động Gazebo simulation:**
Mặc định sẽ mở môi trường **Factory**. Để mở môi trường Warehouse cũ, hãy thêm `world_file:=small_warehouse.sdf`.

```bash
ros2 launch anhc_bot simulation.launch.py \
  camera_enabled:=True \
  two_d_lidar_enabled:=True
```

**Terminal 2 — Khởi động SLAM để vẽ bản đồ (RViz2 sẽ tự động mở):**
```bash
ros2 launch anhc_bot mapping.launch.py
```

**Terminal 3 — Điều khiển robot đi khám phá môi trường:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  cmd_vel:=/anhc_bot/cmd_vel
```

> Dùng phím `i/,` để tiến/lùi, `j/l` để quay. Lái robot đi khắp môi trường để SLAM vẽ đầy đủ bản đồ.

---

### Giai đoạn 2 — Lưu bản đồ

Sau khi đã lái robot khám phá hết khu vực, lưu bản đồ lại:
```bash
cd ~/anhc_ws/src/anhc_bot/config/maps
ros2 run nav2_map_server map_saver_cli -f factory_map
```

> Lệnh này tạo ra 2 file: `factory_map.pgm` và `factory_map.yaml`.

---

### Giai đoạn 3 — Điều hướng tự động với Nav2

**Terminal 1 — Khởi động Gazebo:**
```bash
cd ~/anhc_ws
source ~/anhc_ws/install/setup.bash
ros2 launch anhc_bot simulation.launch.py \
  camera_enabled:=True \
  two_d_lidar_enabled:=True \
  world_file:=factory.sdf
```

**Terminal 2 — Khởi động Navigation:**
Mặc định hệ thống sẽ sử dụng `factory_map.yaml`.
```bash
ros2 launch anhc_bot navigation.launch.py
```

---

## 4. Tùy biến nâng cao (Command Line Arguments)

Hệ thống đã được cấu hình linh hoạt để bạn có thể thay đổi môi trường và vị trí robot mà không cần sửa code.

### 4.1 Thay đổi Vị trí Robot ban đầu
Sử dụng các tham số `position_x`, `position_y`, và `orientation_yaw` (đơn vị: mét và radian):
```bash
ros2 launch anhc_bot simulation.launch.py \
  position_x:=5.0 \
  position_y:=-2.0 \
  orientation_yaw:=3.14
```

### 4.2 Chạy với Map và World khác
Bạn có thể kết hợp các tham số để chạy bất kỳ môi trường nào:
```bash
# Chạy Warehouse cũ
ros2 launch anhc_bot simulation.launch.py world_file:=small_warehouse.sdf

# Chạy Navigation với bản đồ cũ
ros2 launch anhc_bot navigation.launch.py map:=/home/anhuynh/anhc_ws/src/anhc_bot/config/maps/anhc_map.yaml
```

---

## 5. 🚀 Multi-Algorithm Global Planner

Hệ thống sử dụng package `anhc_multi_planner` — một Nav2 GlobalPlanner plugin hỗ trợ **7 thuật toán tìm đường** có thể chuyển đổi nóng (hot-swap).

#### Chuyển đổi thuật toán khi đang chạy
```bash
# Publish topic để đổi sang thuật toán khác ngay lập tức:
ros2 topic pub --once /planning/set_algorithm std_msgs/msg/String "data: 'jps'"
ros2 topic pub --once /planning/set_algorithm std_msgs/msg/String "data: 'rrt_star'"
ros2 topic pub --once /planning/set_algorithm std_msgs/msg/String "data: 'astar'"
```

#### Danh sách thuật toán hỗ trợ:
`astar`, `dijkstra`, `greedy_bfs`, `theta_star`, `jps`, `rrt_star`, `dstar_lite`.

---

### 🖱️ Cách đặt mục tiêu trong RViz2

1. Trong RViz2, click **"2D Pose Estimate"** → click và kéo chuột đúng hướng robot đang đứng trên bản đồ.
2. Click **"Nav2 Goal"** trên toolbar.
3. Click và kéo chuột vào vị trí đích trên bản đồ → robot sẽ tự lên kế hoạch đường đi.

---

### ⚠️ Khắc phục sự cố (Troubleshooting)

**Nếu Gazebo bị crash (Segmentation Fault):**
Thử chạy với biến môi trường ép sử dụng card NVIDIA hoặc render engine Ogre:
```bash
GZ_RENDERING_BACKEND=ogre ros2 launch anhc_bot simulation.launch.py
```

**Xóa cache Gazebo nếu môi trường không hiển thị đúng:**
```bash
rm -rf ~/.gz/sim/main
```
