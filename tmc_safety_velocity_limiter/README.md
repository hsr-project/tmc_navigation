tmc_safety_velocity_limiter
===============================================================================

概要
-------------------------------
周囲の障害物、occupancy値をもとに、入力した速度に制限をかけて出力するバーチャルバンパ機能。
制限をかけた場合はその要因となった座標を出力する。
バーチャルバンパの形状や、減速方式はパラメータにて設定することが出来る。詳細はパラメータ項目参照。


背景
-------------------------------
走行経路上および経路周辺に障害物が存在した場合、衝突しないように減速および停止する必要がある。
そのため、障害物点群が近くにある、または障害物Mapのoccupancy値が高い場所では速度を落として走行させる。

インターフェース
-------------------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "output_velocity" [geometry_msgs/msg/Twist] : 出力速度指令値。入力速度に対して制限をかけた結果。stopサービスにより速度制限機能が停止しているときはinput_velocityをそのままoutput_velocityに出力する。

- "slowing_down" [std_msgs/msg/Bool] : 急減速検出。パラメータで指定した閾値以上の減速を検出した場合trueを発行する。一定時間検出がないと、falseに戻る。速度制限が行われたかによらず出力速度のみを監視しているため、入力速度の変化も検出対象となる。ただし、stopサービスにより速度制限機能が停止しているときは本機能も停止し、常にfalseを発行する。

- "~/observed_obstacle_pose" [geometry_msgs/msg/PoseStamped] : バーチャルバンパの減速計算のために観測している障害物の座標。速度制限がかけられているときのみ出版される。

- "~/ratio" [std_msgs/msg/Float64] : 減速比率。入力速度指令値に対する出力速度指令値の割合。

- "zero_velocity" [std_msgs/msg/Bool] : 速度0検出

#### 購読するトピック (トピック名 [型] : 説明)
- "input_velocity" [geometry_msgs/msg/Twist] : 入力速度指令値

- "obstacle_map" [nav_msgs/msg/OccupancyGrid] : 障害物Map

- "obstacle_cloud" [sensor_msgs/msg/PointCloud2] : 障害物点群


#### 提供するサービス (サービス名 [型] : 説明)
- "~/start" [std_srvs/srv/Empty] : 速度制限有効化。起動直後の状態はenable_functionパラメータに依存する。

- "~/stop" [std_srvs/srv/Empty] : 速度制限停止。 停止中はinput_velocityをそのままoutput_velocityに出力する

- "~/switch_bumper_set" [tmc_navigation_msgs/srv/SwitchBumperSet] : バーチャルバンパの定義セットを切り替える。  
    起動直後の状態はdefault_bumper_setパラメータに依存する。  
    start/stop中問わずコール可能。  
    disable_bumpersに無効化したいバンパ名を指定できる。

- "~/reset_to_default" [std_srvs/srv/Empty] : デフォルトの設定にリセットする。  
    機能有効/無効設定を、enable_functionパラメータ値にリセット。  
    バンパセット設定を、default_bumper_setパラメータ値にリセット。  
    バンパは全て有効な状態となる。

- "~/get_current_setting" [tmc_navigation_msgs/srv/GetCurrentSetting] : 現在の設定を取得する。
    enable_functionに機能有効/無効を出力する。
    bumper_setにバンパセット名を出力する。
    disable_bumpersに無効化されているバンパ名を出力する。

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- base_frame [string] : 台車フレーム名。このフレームを自己位置として障害物との距離を計算 (default : "base_link")
- enable_function [bool] : 機能有効/無効。trueの場合デフォルトで有効。falseの場合、~startサービスがコールされるまで無効(default : false)
- robot_radius [double] [m] : ロボットの半径 (default : 0.22)
- slowdown_velocity_threshold [double] [m/s]: 急減速検出の最低速度。速度がこの値以下のときは、急減速検出を行わない (default : 0.5)
- slowdown_deceleration_threshold [double] [m/s^2] : 急減速と判定する減速度。0より大きい値を設定すること(default : 2.0)
- slowdown_detection_time [double] [s] : 急減速判定時間。slowdown_deceleration_thresholdを満たす状態がこの時間以上続くとslowing_downトピックをtrueにする。(default : 0.0)
- default_bumper_set [string] : 起動時にデフォルトで選択されるバンパセットの定義名 (必須パラメータ)
- maximum_acceleration [double] [m/s^2] : 最大加速度。入力速度の加速度がこれを超えた場合に加速度を抑制する。0より大きい値を設定すること (default : 0.5)
- maximum_deceleration [double] [m/s^2] : 最大減速度。入力速度の減速度がこれを超えた場合に減速度を抑制する。但し、入力された速度が0(停止)の場合は即反映する。0より大きい値を設定すること (default : 1.0)
- timeout_interval [double] [s] : 速度タイムアウト判定時間。速度トピックがこの時間以上停止していた場合、前回速度を0にし、最小周期として計算する (default : 0.5)

virtual_bumpers以下は可変の構造を取る
    
    virtual_bumpers:                                        : 対象定義。任意個数の以下の構造の繰り返し
        XXX_bumper:                                         : バンパセット定義名。任意のユニークな名称で、複数個定義可能。この名前をswitch_bumper_setの引数に渡す。
            bumpers:                                        : バンパー設定群を定義。複数個定義可能。
                type_0:                                     : バンパセット内の個別バンパの定義名。任意のユニークな名称で、複数個定義可能。
                                                              複数個定義した場合、それぞれ独立して評価を行い、
                                                              最も低い(低速な)値を返したものを採用する。
                    type [string]                           : バーチャルバンパのタイプ(クラス)名。以下が使用可能。(必須パラメータ)
                                                              TriangleBumper, CupBumper, EllipseBumper,
                                                              OccupancyPointBumper, OccupancyEllipseBumper

以下のパラメータはバーチャルバンパタイプに依存する。座標系はいずれもbase_frame基準。

TriangleBumper: 自己位置を基準とした扇型の範囲
    
                    obstacle_search_distance [double] [m]   : 障害物検索範囲の半径
                                                              0より大きい値を指定すること (必須パラメータ)
                    obstacle_search_angle [double] [rad]    : 正面を基準とした、扇型の左右の角度
                                                              内角ではなく、正面±この角度となる
                                                              0より大きい値を指定すること (default : 0.5236)

CupBumper: 自己位置を底に含むカップ形状の範囲
    
                    bottom_length [double] [m]              : 底辺の長さ (default : 0.44)
                    obstacle_search_distance [double] [m]   : 障害物検索範囲の半径
                                                              0より大きい値を指定すること(default : 1.2)
                    obstacle_search_angle [double] [rad]    : 正面を基準とした、カップ形状の内角
                                                              0以上の値を指定すること (default : 0.1745)
EllipseBumper: 自己位置を基準とした楕円範囲
    
                    radius_x [double] [m]                   : 楕円のX軸方向の半径
                                                              0より大きい値を指定すること(default : 2.4)
                    radius_y [double] [m]                   : 楕円のY軸方向の半径
                                                              0より大きい値を指定すること(default : 1.2)
                    center_position_x [double] [m]          : 楕円の中心のX座標 (default : 1.9)

OccupancyPointBumper: 自己位置のoccupancy値により速度制限をかける。固有パラメータなし。

OccupancyEllipseBumper: 自己位置周辺の円形範囲のoccupancy値の平均値により速度制限をかける。
    
                    radius_x [double] [m]                   : 楕円のX軸方向の半径
                                                              0より大きい値を指定すること (default : 2.4)
                    radius_y [double] [m]                   : 楕円のY軸方向の半径
                                                              0より大きい値を指定すること (default : 1.2)
                    center_position_x [double] [m]          : 楕円の中心のX座標 (default : 1.9)

バーチャルバンパタイプ共通パラメータ
    
                    auto_scaling:                           : 入力速度に応じてバンパのサイズを変化させる。省略時は100%固定となる。
                        max_scale_velocity [double] [m/s]   : サイズ比が1(100%)となる速度 (default : 0.4)
                        min_scale_velcoity [double] [m/s]   : サイズ比がmin_scaleとなる速度 (default : 0.2)
                        min_scale [double]                  : 速度がmin_scale_velocity以下のときのサイズ倍率
                                                              0 <= min_scale <= 1.0の範囲で指定する。(default : 1.0)
                                                              サイズ倍率は、速度がmax_scale_velocity以上のときは1.0、
                                                              min_scale_velocity以下のときはmin_scale、
                                                              max_scale_velocity～min_scale_velocity間は線形に変化する。
                                                              サイズ倍率により変化する値はバンパタイプにより異なり、
                                                              以下のパラメータの値が算出したサイズ倍率をかけた値になる。
                                                              TriangleBumper, CupBumper:
                                                                  obstacle_search_distance
                                                              EllipseBumper, OccupancyEllipseBumper:
                                                                  radius_x
                                                                  radius_y
                                                                  center_position_x
                    velocity_slope:                         : 各バンパ形状の評価結果に対する、傾斜の付け方(スロープ)を指定する
                        type [string]                       : スロープのタイプ(クラス)名。以下が使用可能。
                                                              FixedSlope, LinearSlope, LogarithmSlope (必須パラメータ)

以下のパラメータはスロープタイプに依存する。

FixedSlope: 入力によらず、常に同じ値を返す。

                        fixed_ratio [double]                : 固定値(0.0～1.0) (default : 0.0)

LinearSlope: 入力に対して出力を線形に変化させる
    
                        upper_limit_threshold [double]      : 速度制限をかけ始める評価値(0.0～1.0) (default : 1.0)
                        lower_limit_threshold [double]      : 速度制限を終了する評価値(0.0～1.0) (default : 0.0)
                        upper_limit_ratio [double]          : upper_limit_threshold以上のときの出力値(0.0～1.0) (default : 1.0)
                        lower_limit_ratio [double]          : upper_limit_thresholdのときの出力値(0.0～1.0)
                                                              入力がupper_limit_threshold～lower_limit_thresholdのとき、
                                                              出力をupper_limit_ratio～lower_limit_tresholdの範囲で線形に変化させる。(default : 0.0)
                        minimum_ratio [double]              : 入力がlower_limit_threshold以下のときの出力値(0.0～1.0) (default : 0.0)

LogarithmSlope: 入力に対して出力を対数的に変化させる
    
                        upper_limit_threshold [double]      : 速度制限をかけ始める評価値(0.0～1.0)  (default : 1.0)
                        lower_limit_threshold [double]      : 速度制限を終了する評価値(0.0～1.0) (default : 0.0)
                        upper_limit_ratio [double]          : upper_limit_threshold以上のときの出力値(0.0～1.0) (default : 1.0)
                        lower_limit_ratio [double]          : upper_limit_thresholdのときの出力値(0.0～1.0) (default : 0.0)
                        minimum_limit_ratio [double]        : 入力がlower_limit_threshold以下のときの出力値(0.0～1.0) (default : 0.0)
                        logarithm_base [double]             : 対数の底(>1) 1に近いほど直線に近く、大きいほどカーブが強くなる (default : 10)


使用例
-------------------------------
1.起動
$ ros2 launch tmc_safety_velocity_limiter safety_velocity_limiter.launch.py

2.速度制限開始/停止
$ ros2 service call /safety_velocity_limiter/start std_srvs/srv/Empty "{}"
$ ros2 service call /safety_velocity_limiter/stop std_srvs/srv/Empty "{}"

3.バーチャルバンパの定義セットを切り替える
$ ros2 service call /safety_velocity_limiter/switch_bumper_set tmc_navigation_msgs/srv/SwitchBumperSet "{bumper_set:  {data: 'triangle_bumper'}, disable_bumpers: []}"

速度制限機能
-------------------------------
速度制限は、バーチャルバンパとスロープの組み合わせで実現し、以下のしくみで行われる。
1. バーチャルバンパが、周囲の状況を評価し、(0.0:障害最大～1.0:障害なし)の評価値を生成する
2. スロープが、評価値から速度に掛ける比率(0.0～1.0)を計算する

バーチャルバンパは複数重ねてセットにすることができ、セット内で最も低い値を返したものが採用される。