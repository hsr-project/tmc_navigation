tmc_laser_2d_localizer
========================

開発関係者
-----------------------
- Yoshiaki Asahara

目的
-----------------------
2次元距離データと，2次元地図を用いて，ロボット自己位置を補正する．

背景
-----------------------
オドメトリでも充分精度は高いが，長距離を移動すると誤差が蓄積するため，本ノードを用いて誤差を補正する必要がある．

前提条件
-----------------------
- 2次元レーザデータを必要とする．
- 2次元グリッド地図を必要とする．
- オドメトリを必要とする．ノード内ではオドメトリの差分しか使用しないため，座標系は特に問わない．
- グリッド地図の大きさは任意．
- 自己位置推定はMCL(Monte Carlo Localization)手法を利用している．


振る舞い
-----------------------
- オドメトリ，グリッド地図，２次元距離データを入力とし，グローバル座標系における座標(x,y,θ)を出力する．
- オドメトリ，グリッド地図，２次元距離データのすべてのデータを一度受け取ると初期化を終了し，自己位置補正処理を開始する．
  ただし，初期位置のみ，ノード起動時に無条件でpublishする．
- 自己位置補正は周期駆動ではなく，イベントドリブンとなっている．イベントトリガは以下の通り．
    - 並進オドメトリ差分の累積が，並進距離閾値を越えたとき．
    - 回転オドメトリ差分の累積が，回転角度閾値を越えたとき．
- 上記イベントが発生したとき，MCLの処理を１回実行してその結果を出版し，各オドメトリ差分の累積値をリセットする．
- 地図はトピックを受けることで動的に変更することができる．
  
インターフェース
-----------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "laser_2d_pose" [geometry_msgs/msg/PoseWithCovarianceStamped] : 自己位置推定結果を出力．初期位置の出力も兼ねているため，latchをONにしている．PoseWithCovarianceStamped型を利用しているが，実質は2次元平面状の位置で表現されている．言い換えると，z, pitch, rollに有効な値は入っていない．

- "particle_positions" [sensor_msgs/msg/PointCloud] : デバッグ用．MCLのパーティクルの位置を表示に利用．
  
#### 購読するトピック (トピック名 [型] : 説明)
- "input_cloud" [sensor_msgs/msg/PointCloud2] : ２次元ポイントクラウド．

- "static_distance_ros_map" [nav_msgs/msg/OccupancyGrid] : グリッド地図．グリッド地図の各グリッドには，Free(0), Unknwon(-1), Wall(100)の三値が格納されていることを前提とする．

- "laser_2d_correct_pose" [geometry_msgs/msg/PoseWithCovarianceStamped] : 自己位置. 取得しリセットに使用.
  

#### 提供するサービス (サービス名 [型] : 説明)
- "start_sending_localized_pose" [std_msgs/srv/Empty] : 自己位置（laser_2d_pose）を出版する

- "stop_sending_localized_pose" [std_msgs/srv/Empty] : 自己位置（laser_2d_pose）の出版を停止する

- "check_localizer_running" [tmc_navigation_msgs/srv/BoolResponse] : ローカライザが稼働中であるかを真偽値で返す

- "set_localization_score_limit" [tmc_navigation_msgs/srv/SetLocalizationScoreLimit] : ローカライザのスコア閾値を設定する

#### 利用する tf (<親フレーム> -> <子フレーム> : 説明)
- <robot_tf_name> -> <input_cloudのフレーム> : センサー位置を取得

- <map_tf_name> -> <odometry_tf_name> : 自己位置算出に使用

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))

- localization_score_limit [double] : マップマッチングのスコアしきい値:尤度 (default : 0.0)
- number_of_particles [double] : パーティクル数 (default : 200.0)
- effective_particle_ratio [double] : 有効なパーティクル数の割合．リサンプリングの閾値で利用:0.0～1.0 (default : 0.5)
- standard_deviation_xy_to_yx [double] [m] : x(y)移動量がy(x)移動量に与えるオドメトリ誤差の標準偏差 (default : 0.1)
- standard_deviation_xy_to_xy [double] [m] : x(y)移動量がx(y)移動量に与えるオドメトリ誤差の標準偏差 (default : 0.1)
- standard_deviation_xy_to_theta [double] [rad] : 並進移動量が角度移動量に与えるオドメトリ誤差の標準偏差 (default : 0.01)
- standard_deviation_theta_to_xy [double] [m] : 角度移動量がx(y)座標に与えるオドメトリ誤差の標準偏差 (default : 1.0)
- standard_deviation_theta_to_theta [double] [rad] : 角度移動量が角度移動量に与えるオドメトリ誤差の標準偏差 (default : 0.1)
- standard_deviation_init_xy [double] [m] : 位置リセット時のパーティクル位置の標準偏差 (default : 0.0)
- standard_deviation_init_theta [double] [rad] : 位置リセット時のパーティクル位置の標準偏差 (default : 0.0)
- init_x [double] [m] : 初期位置 (default : 0.0)
- init_y [double] [m] : 初期位置 (default : 0.0)
- init_theta_deg [double] [deg] : 初期方向 (default : 0.0)
- distance_triggering_filter [double] [m] : パーティクルフィルタを実行するトリガとなる並進移動量 (default : 0.1)
- angle_triggering_filter [double] [rad] : パーティクルフィルタを実行するトリガとなる角度移動量 (default : 0.5)
- potential_width [double] [m] : 地図のポテンシャル幅 (default : 3.0)
- laser_filter [double] [m] : 地図にない障害物を消すための閾値.壁から閾値以上離れたら消す (default : 1.0)
- odometry_tf_name [string] : オドメトリtf名:文字列 (default : "odom")
- base_tf_name [string] : 台車tf名:文字列 (default : "base_footprint")
- robot_tf_name [string] : ロボットtf名:文字列 (default : "base_link")
  
使用例
--------------
- 通常起動
    1. tmc_laser_2d_localizer/config/laser_2d_localizer.yamlのパラメータ設定
    2. $ ros2 launch tmc_laser_2d_localizer laser_2d_localizer.launch.py

