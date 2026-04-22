# 点云转激光扫描 - FAST-LIO

本包提供了 launch 文件，用于将 FAST-LIO 的 3D 点云转换为 2D 激光扫描，供 Cartographer SLAM 和其他基于 2D 的导航算法使用。

## 文件概述

### Launch 文件

#### 1. `pointcloud_to_laserscan_fastlio.launch.py`
独立的点云转激光扫描转换节点。将 FAST-LIO 的 `/cloud_registered` 话题转换为 `/scan`。

**使用方法：**
```bash
ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_fastlio.launch.py
```

**带自定义参数：**
```bash
ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_fastlio.launch.py \
    min_height:=-0.1 max_height:=0.1 range_max:=25.0
```

**关键参数：**
- `min_height`: 激光扫描提取的最小高度（默认：-0.15 m）
- `max_height`: 激光扫描提取的最大高度（默认：0.15 m）
- `range_min`: 最小扫描范围（默认：0.5 m）
- `range_max`: 最大扫描范围（默认：30.0 m）
- `target_frame`: 目标坐标系（默认：base_link）

---

#### 2. `fastlio_cartographer_slam.launch.py`
集成 launch 文件，用于完整的 SLAM 流程：
- 点云转激光扫描转换
- Cartographer 2D SLAM 节点
- Cartographer 占用栅格节点

**使用方法：**
```bash
ros2 launch pointcloud_to_laserscan fastlio_cartographer_slam.launch.py
```

**带参数：**
```bash
ros2 launch pointcloud_to_laserscan fastlio_cartographer_slam.launch.py \
    use_sim_time:=true min_height:=-0.15 max_height:=0.15
```

---

### 配置文件

#### `config/fastlio_default.yaml`
点云转激光扫描转换的默认配置参数。包含：
- 高度阈值（min/max height）
- 角度分辨率（angle_min, angle_max, angle_increment）
- 范围设置（range_min, range_max）
- 坐标系和变换设置

---

## 数据流

```
FAST-LIO 点云 (/cloud_registered)
           ↓
点云转激光扫描节点
           ↓
2D 激光扫描 (/scan)
           ↓
Cartographer SLAM
           ↓
占用栅格地图 (/map)
```

---

## 关键话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/cloud_registered` | sensor_msgs/PointCloud2 | FAST-LIO 输入（3D 点云） |
| `/scan` | sensor_msgs/LaserScan | 输出 2D 激光扫描 |
| `/map` | nav_msgs/OccupancyGrid | Cartographer 输出地图 |

---

## 重要参数配置

### 高度范围配置
根据平台调整 `min_height` 和 `max_height`：

- **地面机器人**：使用 `min_height: -0.2, max_height: 0.2`（扫描水平平面）
- **空中机器人**：使用 `min_height: -0.1, max_height: 0.1`（垂直范围窄）
- **室内导航**：使用 `min_height: -0.15, max_height: 0.15`（默认，平衡）

### 范围配置
根据雷达规格设置 `range_max`：
- Livox Mid360：30 m
- Livox Mid40：38 m  
- Sick S300：25 m
- RPLidar S1：12 m

---

## 故障排查

### 问题：没有激光扫描输出
- 检查 FAST-LIO 是否发布到 `/cloud_registered`
- 验证 TF 树连接
- 检查高度范围设置（可能过滤掉了所有点）

### 问题：激光扫描数据稀疏
- 增加 `range_max` 值
- 验证 FAST-LIO 点云质量
- 调整 `min_height` / `max_height` 范围

### 问题：Cartographer 无法初始化
- 确保 `/scan` 话题有持续数据
- 检查 TF 树是否有 base_link → odom → map 链
- 验证 use_sim_time 参数与系统匹配

---

## 项目集成

在 Sentry 机器人上使用：

1. **启动 FAST-LIO SLAM：**
   ```bash
   ros2 launch fast_lio mapping.launch.py
   ```

2. **在另一个终端启动点云转激光扫描：**
   ```bash
   ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_fastlio.launch.py
   ```

3. **或使用集成 launch 文件：**
   ```bash
   ros2 launch pointcloud_to_laserscan fastlio_cartographer_slam.launch.py
   ```

4. **在 RViz 中验证：**
   - 添加 `/scan` 显示（LaserScan）
   - 添加 `/map` 显示（OccupancyGrid）
   - 在 Transforms 显示中检查 TF 树

---

## 参考资源

- [pointcloud_to_laserscan 文档](https://github.com/ros-perception/pointcloud_to_laserscan)
- [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/)
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO2)
