開発関係者
================
朝原佳昭

ノード一覧
============
- marker\_based\_localizer

目的
===============
- マーカーを認識したら、マーカーの位置を参考にしてロボットの位置を出力する

使用例
========
- ros2 launch tmc_marker_based_localizer marker_based_localizer_example.launch.py

前提条件
================
- マーカー認識が働いており、yamlファイルにマーカー位置が保存されている
- 台車からカメラフレームまでのtfがbroadcastされている
- オドメトリの速度が停止しているときにマーカー位置による自己位置補正を実行するので、オドメトリが出版されていないといけない

ハイレベルアーキテクチャ
================
特になし

振る舞い
================
- マーカー認識結果のトピックを受けとると、マーカーIDを使ってyamlファイルからマーカー位置を取得する
- yamlファイルのマーカーのグローバル位置、マーカーの位置姿勢の認識結果、台車とカメラフレームの座標関係から、フロア座標系に対するロボットの位置を算出する
- 上記処理はロボットが移動していないときにのみ行われる。移動中のマーカー認識は誤差が大きいため

インタフェース
=================
- 購読するトピック
  - "/marker/object\_info" (tmc\_vision\_msgs/RecognizedObject) : 認識されたマーカー情報
  - "/odometry" (nav\_msgs/Odometry) : オドメトリ。ロボットが移動しているかどうかの判定に使用
  - "/joint\_states" (sensor\_msgs/JointState) : 軸情報 ロボットの停止状態を判定するために必要
- 発行するトピック
  - "/localized\_pose" (geometry\_msgs/PoseWithCovarianceStamped) : マーカー位置から推定した自己位置
よって更新された物体の情報

- 提供するサービス 
  - "start\_marker\_based\_localizer" (std\_srvs/Empty) : 補正を有効にする
  - "stop\_marker\_based\_localizer" (std\_srvs/Empty) : 補正を無効にする

- 利用する外部パラメータ 
  - "robot\_tf\_name" (std::string) : ロボットベースのフレーム名

- 使用する内部パラメータ
  - ~travel\_distance\_threshold (double, default: 5.0[m]): 補正を有効にする台車移動量の閾値
  - ~marker\_to\_base\_distance\_threshold (double, default: 1.5[m]): 補正を有効にするマーカーまでの距離閾値
  - ~joints\_list (string[]) : 停止状態を判定する軸の名前
  - ~joint\_stopping\_vel (double, default: 0.001[rad/sec]) : 停止状態と判定する各軸の速度
