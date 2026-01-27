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
### 購読するトピック
input_velocity        (geometry_msgs::msg::Twist)
 入力速度指令値

obstacle_map          (nav_msgs::msg::OccupancyGrid)
 障害物Map

obstacle_point_cloud  (sensor_msgs::msg::PointCloud2)
 障害物点群

### 出版するトピック
output_velocity       (geometry_msgs::msg::Twist)
 出力速度指令値。入力速度に対して制限をかけた結果。
 stopサービスにより速度制限機能が停止しているときはinput_velocityをそのままoutput_velocityに出力する。

slowing_down          (std_msgs::msg::Bool)
 急減速検出。パラメータで指定した閾値以上の減速を検出した場合trueを発行する。一定時間検出がないと、falseに戻る。
 速度制限が行われたかによらず出力速度のみを監視しているため、入力速度の変化も検出対象となる。
 ただし、stopサービスにより速度制限機能が停止しているときは本機能も停止し、常にfalseを発行する。

~observed_obstacle_pose          (geometry_msgs::msg::PoseStamped)
 バーチャルバンパの減速計算のために観測している障害物の座標。
 速度制限がかけられているときのみ出版される。

~ratio          (std_msgs::msg::Float64)
 減速比率。入力速度指令値に対する出力速度指令値の割合。

zero_velocity          (std_msgs::msg::Bool)
 速度0検出

### 提供するサービス
#### ~start(std_srvs::srv::Empty)
速度制限有効化  
起動直後の状態はenable_functionパラメータに依存する。

#### ~stop(std_srvs::srv::Empty)
速度制限停止  
停止中はinput_velocityをそのままoutput_velocityに出力する。

#### ~switch_bumper_set(tmc_navigation_msgs::srv::SwitchBumperSet)
バーチャルバンパの定義セットを切り替える。  
起動直後の状態はdefault_bumper_setパラメータに依存する。  
start/stop中問わずコール可能。  
disable_bumpersに無効化したいバンパ名を指定できる。

#### ~reset_to_default(std_srvs::srv::Empty)
デフォルトの設定にリセットする。  
機能有効/無効設定を、enable_functionパラメータ値にリセット。  
バンパセット設定を、default_bumper_setパラメータ値にリセット。  
バンパは全て有効な状態となる。

#### ~get_current_setting(tmc_navigation_msgs::srv::GetCurrentSetting)
現在の設定を取得する。
- enable_functionに機能有効/無効を出力する。
- bumper_setにバンパセット名を出力する。
- disable_bumpersに無効化されているバンパ名を出力する。

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

パラメータ
-------------------------------
    enable_function                 : 機能有効/無効[bool]
                                      trueの場合デフォルトで有効。
                                      falseの場合、~startサービスがコールされるまで無効
    robot_radius                    : ロボットの半径[m]
    slowdown_velocity_threshold     : 急減速検出の最低速度[m/s]
                                      速度がこの値以下のときは、急減速検出を行わない
    slowdown_deceleration_threshold : 急減速と判定する減速度[m/s^2]
                                      0より大きい値を設定すること
    slowdown_detection_time         : 急減速判定時間[s]
                                      slowdown_deceleration_thresholdを満たす状態がこの時間以上続くと
                                      slowing_downトピックをtrueにする。
    default_bumper_set              : 起動時にデフォルトで選択されるバンパセットの定義名
    maximum_acceleration            : 最大加速度[m/s^2]
                                      入力速度の加速度がこれを超えた場合に加速度を抑制する。
                                      0より大きい値を設定すること
    maximum_deceleration            : 最大減速度[m/s^2]
                                      入力速度の減速度がこれを超えた場合に減速度を抑制する。
                                      但し、入力された速度が0(停止)の場合は即反映する。
                                      0より大きい値を設定すること
    timeout_interval                : 速度タイムアウト判定時間[s]
                                      速度トピックがこの時間以上停止していた場合、前回速度を0にし、
                                      最小周期として計算する

virtual_bumpers以下は可変の構造を取る

    virtual_bumpers:                : 対象定義。任意個数の以下の構造の繰り返し
      triangle_bumper:              : バンパセット定義名。任意のユニークな名称で、複数個定義可能。
                                      この名前をswitch_bumper_setの引数に渡す。
        type_0:                     : バンパセット内の個別バンパの定義名。任意のユニークな名称で、複数個定義可能。
                                      複数個定義した場合、それぞれ独立して評価を行い、
                                      最も低い(低速な)値を返したものを採用する。
          type                      : バーチャルバンパのタイプ(クラス)名。以下が使用可能。
                                      TriangleBumper, CupBumper, EllipseBumper,
                                      OccupancyPointBumper, OccupancyEllipseBumper

以下のパラメータはバーチャルバンパタイプに依存する。座標系はいずれもbase_link基準。

TriangleBumper: 自己位置を基準とした扇型の範囲

          obstacle_search_distance  : 障害物検索範囲の半径[m]
                                      0より大きい値を指定すること
          obstacle_search_angle     : 正面を基準とした、扇型の左右の角度[rad]
                                      内角ではなく、正面±この角度となる
                                      0より大きい値を指定すること

CupBumper: 自己位置を底に含むカップ形状の範囲

          bottom_length             : 底辺の長さ[m]
          obstacle_search_distance  : 障害物検索範囲の半径[m]
                                      0より大きい値を指定すること
          obstacle_search_angle     : 正面を基準とした、カップ形状の内角[rad]
                                      0以上の値を指定すること
EllipseBumper: 自己位置を基準とした楕円範囲

          radius_x                  : 楕円のX軸方向の半径[m]
                                      0より大きい値を指定すること
          radius_y                  : 楕円のY軸方向の半径[m]
                                      0より大きい値を指定すること
          center_position_x         : 楕円の中心のX座標

OccupancyPointBumper: 自己位置のoccupancy値により速度制限をかける。固有パラメータなし。

OccupancyEllipseBumper: 自己位置周辺の円形範囲のoccupancy値の平均値により速度制限をかける。

          radius_x                  : 楕円のX軸方向の半径[m]
                                      0より大きい値を指定すること
          radius_y                  : 楕円のY軸方向の半径[m]
                                      0より大きい値を指定すること
          center_position_x         : 楕円の中心のX座標

バーチャルバンパタイプ共通パラメータ

          auto_scaling:             : 入力速度に応じてバンパのサイズを変化させる。省略時は100%固定となる。
            max_scale_velocity      : サイズ比が1(100%)となる速度[m/s]
            min_scale_velcoity      : サイズ比がmin_scaleとなる速度[m/s]
            min_scale               : 速度がmin_scale_velocity以下のときのサイズ倍率
                                      0 <= min_scale <= 1.0の範囲で指定する。

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
          velocity_slope:           : 各バンパ形状の評価結果に対する、傾斜の付け方(スロープ)を指定する
            type                    : スロープのタイプ(クラス)名。以下が使用可能。
                                      FixedSlope, LinearSlope, LogarithmSlope

以下のパラメータはスロープタイプに依存する。

FixedSlope: 入力によらず、常に同じ値を返す。

            fixed_ratio             : 固定値(0.0～1.0)

LinearSlope: 入力に対して出力を線形に変化させる

            upper_limit_threshold   : 速度制限をかけ始める評価値(0.0～1.0)
            lower_limit_threshold   : 速度制限を終了する評価値(0.0～1.0)
            upper_limit_ratio       : upper_limit_threshold以上のときの出力値(0.0～1.0)
            lower_limit_ratio       : upper_limit_thresholdのときの出力値(0.0～1.0)
                                      入力がupper_limit_threshold～lower_limit_thresholdのとき、
                                      出力をupper_limit_ratio～lower_limit_tresholdの範囲で線形に変化させる。
            minimum_limit_ratio     : 入力がlower_limit_threshold以下のときの出力値(0.0～1.0)

LogarithmSlope: 入力に対して出力を対数的に変化させる

            upper_limit_threshold   : 速度制限をかけ始める評価値(0.0～1.0)
            lower_limit_threshold   : 速度制限を終了する評価値(0.0～1.0)
            upper_limit_ratio       : upper_limit_threshold以上のときの出力値(0.0～1.0)
            lower_limit_ratio       : upper_limit_thresholdのときの出力値(0.0～1.0)
            minimum_limit_ratio     : 入力がlower_limit_threshold以下のときの出力値(0.0～1.0)
            logarithm_base          : 対数の底(>1) 1に近いほど直線に近く、大きいほどカーブが強くなる
