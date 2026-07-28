tmc_marker_based_localizer
============================

開発関係者
----------------
朝原佳昭

目的
---------------
- マーカーを認識したら、マーカーの位置を参考にしてロボットの位置を出力する

使用例
--------
- ros2 launch tmc_marker_based_localizer marker_based_localizer_example.launch.py

前提条件
----------------
- マーカー認識が働いており、yamlファイルにマーカー位置が保存されている
- 台車からカメラフレームまでのtfがbroadcastされている
- オドメトリの速度が停止しているときにマーカー位置による自己位置補正を実行するので、オドメトリが出版されていないといけない

振る舞い
----------------
- マーカー認識結果のトピックを受けとると、マーカーIDを使ってyamlファイルからマーカー位置を取得する
- yamlファイルのマーカーのグローバル位置、マーカーの位置姿勢の認識結果、台車とカメラフレームの座標関係から、フロア座標系に対するロボットの位置を算出する
- 上記処理はロボットが移動していないときにのみ行われる。移動中のマーカー認識は誤差が大きいため

インタフェース
-----------------
#### 出版するトピック (トピック名 [型] : 説明)
- "localized_pose" [geometry_msgs/msg/PoseWithCovarianceStamped] : マーカー位置から推定した自己位置によって更新された物体の情報

#### 購読するトピック (トピック名 [型] : 説明)
- "marker/object_info" [tmc_vision_msgs/msg/RecognizedObject] : 認識されたマーカー情報

- "odometry" [nav_msgs/msg/Odometry] : オドメトリ。ロボットが移動しているかどうかの判定に使用

- "joint_states" [sensor_msgs/msg/JointState] : 軸情報 ロボットの停止状態を判定するために必要

#### 提供するサービス (サービス名 [型] : 説明)
- "start_marker_based_localizer" [std_srvs/srv/Empty] : 補正を有効にする
- "stop_marker_based_localizer" [std_srvs/srv/Empty] : 補正を無効にする

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- base_tf_name [string] : ロボットベースのフレーム名 (default : "base_footprint")
- travel_distance_threshold [double] [m] : 補正を有効にする台車移動量の閾値 (default : 5.0)
- marker_to_base_distance_threshold [double] [m] : 補正を有効にするマーカーまでの距離閾値 (default : 1.5)
- joints_list [string[]] : 停止状態を判定する軸の名前 (必須パラメータ)
- joint_stopping_vel [double] [rad/s] : 停止状態と判定する各軸の速度 (default : 0.001)

- marker_objects [dict] : マーカーオブジェクトの定義
    - 入力名 : 任意の名前を指定する。
        - object_id [int]  : マーカーID (必須パラメータ)
        - translation [double[3]] : 位置 (必須パラメータ)
        - rotation [double[4]] : 姿勢 (必須パラメータ)