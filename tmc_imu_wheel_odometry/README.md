tmc_imu_wheel_odometry
=========================

imu_odom
-------------------------

概要
-------------------------
IMUとホイールオドメトリを統合したオドメトリを出版する  
IMUとホイールオドメトリを時刻同期して受信する  
並進はホイールオドメトリ、回転はIMUのyaw角度を使用し位置と姿勢を計算する  

インターフェース
-------------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "imu_odom" [nav_msgs/msg/Odometry] : 出力オドメトリ

#### 購読するトピック (トピック名 [型] : 説明)
- "imu" [sensor_msgs/msg/Imu] : 入力IMU

- "wheel_odom" [nav_msgs/msg/Odometry] : 入力オドメトリ

<br>
<br>

lower_imu_odom
-------------------------

概要
-------------------------
全方位台車を構成するオフセット台車の台車軸による角度と、IMUによる角度を統合したオドメトリを出版する

インターフェース
-------------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "imu_odom" [nav_msgs/msg/Odometry] : 出力オドメトリ

#### 購読するトピック (トピック名 [型] : 説明)
- "imu" [sensor_msgs/msg/Imu] : 入力IMU

- "joint_state" [control_msgs/msg/JointTrajectoryControllerState] : 台車軸のステート

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- wheel_radius_l [double] [m] : 左ホイール半径 (default : 0.1)
- wheel_radius_r [double] [m] : 右ホイール半径 (default : 0.1)
- caster_offset [double] [m] : キャスターのオフセット距離 (default : 0.0)
- thread [double] [m] : 左右車輪間の距離 (default : 0.3)
- valid_joint_vel_ths [double[3]] [m/s, m/s, rad/s] : ジョイント速度の有効閾値リスト (default : [100000.0, 100000.0, 100000.0])
- base_angle_offset [double] [rad] : 台車部とホイールのオフセット角度 (default : 0.0)
- odom_name [string] : odomフレーム名 (default : "odom")
- base_link_name [string] : base_linkフレーム名 (default : "base_footprint")
- vel_std [double[3]] [m/s, m/s, rad/s] : 速度の標準偏差 (default : [0.0, 0.0, 0.0])

<br>
<br>

integrated_imu_yaw_publisher
-------------------------

概要
-------------------------
入力IMUのyaw角速度を単純積分しyaw角度を求め、orientationに上書きし出版する

インターフェース
-------------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "integrated_imu" [sensor_msgs/msg/Imu] : 出力IMU

#### 購読するトピック (トピック名 [型] : 説明)
- "imu" [sensor_msgs/msg/Imu] : 入力IMU

<br>
<br>

imu_reset_bias_node
-------------------------

概要
-------------------------
静止状態にあるときのIMUメッセージを蓄積し、連続した一定サンプル数が集まった時点でバイアスを計算する  
バイアス補正済みのIMUメッセージを出版する

インターフェース
-------------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "bias" [geometry_msgs/msg/Vector3] : 累積バイアス

- "data_raw_br" [sensor_msgs/msg/Imu] : バイアス補正済みのIMU

#### 購読するトピック (トピック名 [型] : 説明)
- "cmd_vel" [geometry_msgs/msg/Twist] : 台車速度。静止状態であるかの判別に使用

- "data_raw" [sensor_msgs/msg/Imu] : 入力IMU

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- reset_data_num [int] : バイアス計算に使うサンプル数 (default : 1000)
- cmd_vel_angular_threshold [double] [rad/s] : 旋回速度が静止状態かを判別する閾値。command_velocityのangularの各成分がこの閾値を超えれば静止していないと判定 (default : 0.001)
- cmd_vel_linear_threshold [double] [m/s] : 並進速度が静止状態かを判別する閾値。command_velocityのlinearの各成分がこの閾値を超えれば静止していないと判定 (default : 0.001)
- imu_angular_threshold [double[]] [rad/s] : 旋回速度が静止状態かを判別する閾値。imuのangularの各成分がこの閾値を超えれば静止していないと判定 (default : [0.010])  
   listに2つ値を設定すればリセット前後で閾値を切り替えることができる
- use_init_bias [bool] : バイアス初期値を使用するか (default : false)
- init_bias [double[3]] [rad/s] : バイアス初期値 (default : [0.0,0.0,0.0])

<br>
<br>

odom_info_changer
-------------------------

概要
-------------------------
購読したオドメトリのchild_frame_idをパラメータ new_child_frame_idに置換し出版する

インターフェース
-------------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "out_odom" [nav_msgs/msg/Odometry] : 出力オドメトリ

#### 購読するトピック (トピック名 [型] : 説明)
- "in_odom" [nav_msgs/msg/Odometry] : 入力オドメトリ

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- new_child_frame_id [string] : child_frame_idに設定するフレーム名 (default : "base_footprint")

<br>
<br>

static_odom
-------------------------

概要
-------------------------
固定のOdometryを作成し定期的に出版する

インターフェース
-------------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "static_odom" [nav_msgs/msg/Odometry] : 出力オドメトリ

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- rate [double] [hz] : 駆動周期 (default : 30.0)
- frame_id [string] : frame_idに設定するフレーム名 (default : "odom")
- child_frame_id [string] : child_frame_idに設定するフレーム名 (default : "base_footprint")
