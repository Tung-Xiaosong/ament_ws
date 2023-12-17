# ros2 run nav2_map_server map_saver_cli --prefix
-f, --file: 指定保存地图的文件名（必需参数）。
--occ <threshold>: 指定地图中障碍物的阈值，默认值为 0.65。
--free <threshold>: 指定地图中自由空间的阈值，默认值为 0.196。
--mode <mode>: 指定保存地图的模式，可选值为trinary（三值）或scale（比例尺），默认值为trinary。
--resolution <resolution>: 指定地图的分辨率，默认值为 0.05。
--origin_x <origin_x>: 指定地图原点的 x 坐标，默认值为 0.0。
--origin_y <origin_y>: 指定地图原点的 y 坐标，默认值为 0.0。