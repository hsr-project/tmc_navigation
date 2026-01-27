開発関係者
================
- Yoshiaki Asahara

ToDo
================
- C++コーディングルール準拠
- 自己位置を外部リセットするサービスを提供する．
- KD-Samplingの実装
- センサモデルの改良（ノイズ除去）
- ライブラリをリエントラントにする．
- パラメータがライブラリ内部にグローバルに持っているものもあれば，
  APIで都度渡すケースもある．統一させる．
- icSlam_Fd_setInitParticlesが実行されていない場合にも安全であるようにする
- オドメトリが十分移動した後，初めて地図とレーザを受け取った場合の安全性を確認する
- ポイントクラウドとオドメトリと自己位置のタイムスタンプずれを吸収できるようにする．
  ポイントクラウドデータが1s遅れでやってくることへの対応．

目的
=================
2次元距離データと，2次元地図を用いて，ロボット自己位置を補正する．

背景
=================
オドメトリでも充分精度は高いが，長距離を移動すると誤差が蓄積するため，本ノードを用いて誤差を補正する必要がある．

前提条件
================
- 2次元レーザデータを必要とする．
- 2次元グリッド地図を必要とする．
- オドメトリを必要とする．ノード内ではオドメトリの差分しか使用しないため，座標系は特に問わない．
- グリッド地図の大きさは任意．
- 自己位置推定はMCL(Monte Carlo Localization)手法を利用している．

ハイレベルアーキテクチャ
================
（準備中）

振る舞い
================
- オドメトリ，グリッド地図，２次元距離データを入力とし，グローバル座標系における座標(x,y,θ)を出力する．
- オドメトリ，グリッド地図，２次元距離データのすべてのデータを一度受け取ると初期化を終了し，自己位置補正処理を開始する．
  ただし，初期位置のみ，ノード起動時に無条件でpublishする．
- 自己位置補正は周期駆動ではなく，イベントドリブンとなっている．イベントトリガは以下の通り．
    - 並進オドメトリ差分の累積が，並進距離閾値を越えたとき．
    - 回転オドメトリ差分の累積が，回転角度閾値を越えたとき．
- 上記イベントが発生したとき，MCLの処理を１回実行してその結果を出版し，各オドメトリ差分の累積値をリセットする．
- 地図はトピックを受けることで動的に変更することができる．
  
インターフェース
================
- 出版するトピック
    - */laser_2d_pose* [geometry_msgs/PoseWithCovarianceStamped]
      自己位置推定結果を出力．初期位置の出力も兼ねているため，latchをONにしている．
      PoseWithCovarianceStamped型を利用しているが，実質は2次元平面状の位置で表現されている．
      言い換えると，z, pitch, rollに有効な値は入っていない．
    - */particle_positions* [sensor_msgs/PointCloud]
      デバッグ用．MCLのパーティクルの位置を表示に利用．
  
- 購読するトピック
    - */input_cloud* [sensor_msgs/PointCloud2]
      ２次元ポイントクラウド．
    - */static_distance_ros_map* [nav_msgs/OccupancyGrid]
      グリッド地図．グリッド地図の各グリッドには，Free(0), Unknwon(-1), Wall(100)の三値が格納されていることを前提とする．
  
- 提供するサービス
    - */set_laser_2d_pose* [geometry_msgs/PoseStamped]
      :自己位置をリセットする場合に利用する．
    - */start_sending_localized_pose* [std_msgs/Empty]
      :自己位置（laser_2d_pose）を出版する
    - */stop_sending_localized_pose* [std_msgs/Empty]
      :自己位置（laser_2d_pose）の出版を停止する

- 利用するグローバルパラメータ（パラメータ名:説明:単位）
    - localization_score_limit:マップマッチングのスコアしきい値:尤度
    - robot_tf_name:ロボットtf名:文字列

- 固有パラメータ（パラメータ名:説明:単位）
    - number_of_particles:パーティクル数:個
    - effective_particle_ratio:有効なパーティクル数の割合．リサンプリングの閾値で利用:0.0～1.0
    - standard_deviation_xy_to_yx:x(y)移動量がy(x)移動量に与えるオドメトリ誤差の標準偏差:m/m
    - standard_deviation_xy_to_xy:x(y)移動量がx(y)移動量に与えるオドメトリ誤差の標準偏差:m/m
    - standard_deviation_xy_to_theta:並進移動量が角度移動量に与えるオドメトリ誤差の標準偏差:rad/m
    - standard_deviation_theta_to_xy:角度移動量がx(y)座標に与えるオドメトリ誤差の標準偏差:m/rad
    - standard_deviation_theta_to_theta:角度移動量が角度移動量に与えるオドメトリ誤差の標準偏差:rad/rad
    - standard_deviation_init_xy:位置リセット時のパーティクル位置の標準偏差:m
    - standard_deviation_init_theta:位置リセット時のパーティクル位置の標準偏差:rad
    - init_x:初期位置:m
    - init_y:初期位置:m
    - init_theta_deg:初期方向:deg
    - distance_triggering_filter:パーティクルフィルタを実行するトリガとなる並進移動量:m
    - angle_triggering_filter:パーティクルフィルタを実行するトリガとなる角度移動量:rad
    - potential_width:地図のポテンシャル幅:m
    - laser_filter:地図にない障害物を消すための閾値.壁から閾値以上離れたら消す:m
    - odometry_tf_name:オドメトリtf名:文字列
    - map_tf_name:マップtf名:文字列
  
- 利用するtfのフレーム
    - 購読トピック"*/input_cloud*"のフレーム -> "robot_tf_name"に指定のフレーム
    - "map_tf_name"に指定のフレーム -> "odometry_tf_name"に指定のフレーム

- 発行するtfのフレーム
    - なし
  
既知の問題
==============
- パラメータ設定を試行錯誤しなければならない
- その他はTODOに記載．
  
懸念事項
==============
- TODOに記載
  
使用例
==============
- 通常起動
    1. tmc_laser_2d_localizer/config/laser_2d_localizer.yamlのパラメータ設定
    2. $roslaunch tmc_laser_2d_localizer laser_2d_localizer.launch
  
- 単体試験起動
    1. グリッド地図ファイルを指定する．グリッド地図へのパスはtest_laser_2d_localizer.launchに記載されている．
    2. $roslaunch tmc_laser_2d_localizer laser_2d_localizer_single.launch
    3. $roslaunch tmc_laser_2d_localizer test_laser_2d_localizer.launch
