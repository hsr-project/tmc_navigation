tmc_base_velocity_adjuster
===============================================================================

概要
-------------------------------
* 入力された障害物情報、速度から障害物に干渉しない方向に速度を補正する

開発関係者
-------------------------------
* 田中 和仁


ノードの振る舞い
-------------------------------
* 障害物情報と速度を受け取る。
* 入力速度より障害物に接触する可能性のスコアを算出
* スコアが閾値以上であれば、スコアが閾値以下となる方向の補正速度を計算する

インターフェース
------------------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "adjusted_velocity" [geometry_msgs/msg/Twist] : 補正された速度

#### 購読するトピック (トピック名 [型] : 説明)
- "command_velocity" [geometry_msgs/msg/Twist] : 入力速度

- "obstacle" [sensor_msgs/msg/PointCloud2] : 障害物情報. 現在はsensor_msgs/msg/PointCloud2型のみ指定可

#### 提供するサービス (サービス名 [型] : 説明)
- "~/enable" [std_srvs/srv/Empty] : 速度調整機能の有効化

- "~/disable" [std_srvs/srv/Empty] : 速度調整機能の無効化

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- enable_adjustment_default [bool] : 速度補正機能のデフォルト有効/無効設定 (default : true)
- base_velocity_optimizer : 台車速度の最適化に関する設定
    - collision_score_threshold [double] : 干渉と判定するスコア閾値 (default : 0.0)
    - search_direction_range [double] [rad] : 速度補正方向探索範囲 (default : 0.0)
    - avoidance_direction_offset [double] [rad] : 障害物に対して避けようと動き出す方向にかかるオフセット(正値：左方向優先、負値：右方向優先) (default : 0.0)
- collision_estimator : 干渉推定に関する設定
    - collision_area_radius [double] [m] : 干渉を判定する領域の半径 (default : 0.5)
    - collision_area_increase_rate [double] [m/(m/s)] : 入力速度の大きさに対する干渉判定領域半径増加率 (default : 0.5)
    - estimation_time [double] [s] : 干渉予測を行う時間区間 (default : 1.5)
- obstacle_converter : 障害物データ変換に関する設定
    - filter_leaf_size [double] [m] : ダウンサンプル間隔 (default : 0.025)
    - filter_area_radius [double] [m]: データを切り出す範囲の半径 (default : 1.5)
