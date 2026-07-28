tmc_viewpoint_controller
============================

概要
-----------------------------
* 自律移動時、視点をコントロールする機能。
* 本機能のOn,Offをサービスで提供する。
* 視点を移動経路に向けるか、トラッキングターゲットに向けるかの切り替えをサービスで提供する。
* 移動経路に向ける場合、経路の形状から首を向ける角度を決める。
* トラッキングターゲットに向ける場合、ターゲットの軌跡から首を向ける角度を決める

開発関係者
----------------
* 高岡 豊
* 田中 和仁
* 城 崇平

viewpoint_controller
--------------------
* 視点を移動経路に向ける場合、経路、現在位置から視点を計算、首を動かす。
* 経路点が０だと何も発行しない。
* 視点をトラッキングターゲットにに向ける場合、トラッキングターゲットの最新位置、現在位置から視点を計算、首を動かす。
* トラッキング出来ていないなら何も発行しない。
* ロボット位置、各軸情報トピックがないと起動しない。
* trajectory_filterが起動していること。

インターフェース
----------------
#### 出版するトピック (トピック名 [型] : 説明)
- "command" [trajectory_msgs/msg/JointTrajectory] : 首パン、チルト軸指令値

#### 購読するトピック (トピック名 [型] : 説明)
- "base_local_path"[nav_msgs/msg/Path] : 経路

- "joint_states" [sensor_msgs/msg/JointState] : 各軸の現在情報

- "target_path" [nav_msgs/msg/Path] : トラッキングターゲットの軌跡

#### 提供するサービス (サービス名 [型] : 説明)
- "~/start" [std_srvs/srv/Empty] : 本機能Onするサービス

- "~/stop" [std_srvs/srv/Empty] : 本機能Offするサービス

- "~/set_viewpoint_mode_path" [std_srvs/srv/Empty] : 視点を向ける先を、移動経路にするサービス

- "~/set_viewpoint_mode_tracking" [std_srvs/srv/Empty] : 視点を向ける先を、トラッキングターゲットにするサービス

#### 利用するtf (<親フレーム> -> <子フレーム> : 説明)
- <map_frame> -> <base_frame> : 地図上の自己位置

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- robot_pose [string] : ロボット自己位置のトピック名 (default : "global_pose")
- joint_states [string] : 各関節情報トピック名 (default : "joint_states")
- command [string] : 指令値トピック名 (default : "command")
- map_frame [string] : マップフレーム名 (default : "map")
- base_frame [string] : ベースフレーム名 (default : "base_footprint")
- focus_path_length [double] [m] : 経路上何m先を見るか (default : 0.5 )
- max_rotation_once [double] [rad] : 1周期における首パン軸最大旋回量 (default : 0.6)
- head_pan_min [double] [rad] : 首パン軸最小メカリミット (default : -210*M_PI/180)
- head_pan_max [double] [rad] : 首パン軸最大メカリミット (default : 110*M_PI/180)
- fixed_neck_tilt_angle [double] [rad] : 首チルト軸固定角度 (default : 0.0)
- rate [double] [hz] : 駆動周期 (default : 1.0)
- neck_pan_name [string] : 首パン軸名称 (default : "head_pan_joint")
- neck_tilt_name [string] : 首チルト軸名称 (default : "head_tilt_joint")
