livox_ws包编译配置

rm@rm-NUC10i7FNH:~/sentry_nav_V3.0$ bash sh/slam.sh 
# Error creating terminal: Failed to get screen from object path /org/gnome/Terminal/screen/5b3b0b14_5d23_404a_9624_879027c7c9ba
# Error creating terminal: Failed to get screen from object path /org/gnome/Terminal/screen/5b3b0b14_5d23_404a_9624_879027c7c9ba
# Error creating terminal: Failed to get screen from object path /org/gnome/Terminal/screen/5b3b0b14_5d23_404a_9624_879027c7c9ba


追踪参数传递流程


<!--
  Example launch file for octomap_server mapping:
  Listens to incoming PointCloud2 data and incrementally builds an octomap.
  The data is sent out in different representations.

  Copy this file into your workspace and adjust as needed, see
  www.ros.org/wiki/octomap_server for details
-->


<launch>
  <node pkg="octomap_server" exec="octomap_server_node" name="octomap_server" output="screen">

    <!-- 基础参数 -->
    <param name="resolution" value="0.05" />                <!-- 3D地图分辨率 (单位：米) -->
	<!-- fixed map frame (set to 'map' if SLAM or localization running!) -->
    <param name="frame_id" value="map" />                    <!-- 地图坐标系 -->
	<!-- maximum range to integrate (speedup!) -->
    <param name="sensor_model.max_range" value="5.0" />      <!-- 传感器最大有效范围 -->

    <!-- 输入点云话题配置 --><!-- data source to integrate (PointCloud2) -->
    <remap from="cloud_in" to="/cloud_registered" />         <!-- 订阅的点云话题 -->

    <!-- 点云高度过滤 -->
    <param name="pointcloud_min_z" value="-0.5" />          <!-- 最低高度（地面以下保留部分） -->
    <param name="pointcloud_max_z" value="1.0" />           <!-- 最高高度（过滤天花板等） -->

    <!-- 2D 栅格地图配置 -->
    <param name="publish_2d_map" value="true" />            <!-- 启用 2D 投影地图 -->
    <param name="project_complete_map" value="true" />      <!-- 投影完整地图（非实时扫描区域） -->
    <param name="occupancy_min_z" value="0.9" />            <!-- 2D地图投影的最低高度 -->
    <param name="occupancy_max_z" value="1.0" />            <!-- 2D地图投影的最高高度 -->
    <param name="filter_ground" value="true" />             <!-- 启用地面过滤 -->
    <param name="ground_filter/distance" value="0.1" />     <!-- 地面平面拟合阈值 -->

    <!-- 占据概率阈值 -->
    <param name="occupancy_threshold" value="0.7" />        <!-- 占据概率阈值（高于此值视为障碍物） -->
    <param name="prob_hit" value="0.7" />                   <!-- 击中概率（传感器检测到障碍物的置信度） -->
    <param name="prob_miss" value="0.4" />                  <!-- 未击中概率（传感器未检测到障碍物的置信度） -->
  </node>
</launch>
