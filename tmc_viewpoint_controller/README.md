概要
=============================
* 自律移動時、視点をコントロールする機能。
* 本機能のOn,Offをサービスで提供する。
* 視点を移動経路に向けるか、トラッキングターゲットに向けるかの切り替えをサービスで提供する。
* 移動経路に向ける場合、経路の形状から首を向ける角度を決める。
* トラッキングターゲットに向ける場合、ターゲットの軌跡から首を向ける角度を決める

開発関係者
================
* 高岡 豊
* 田中 和仁
* 城 崇平

ノードの振る舞い
================

viewpoint_controller
--------------------
* 視点を移動経路に向ける場合、経路、現在位置から視点を計算、首を動かす。
* 経路点が０だと何も発行しない。
* 視点をトラッキングターゲットにに向ける場合、トラッキングターゲットの最新位置、現在位置から視点を計算、首を動かす。
* トラッキング出来ていないなら何も発行しない。
* ロボット位置、各軸情報トピックがないと起動しない。
* trajectory_filterが起動していること。

インターフェース
================
### Published Topics ###
trajectory_msgs/JointTrajectory : "/command" : 首パン、チルト軸指令値

### Subscribed Topics ###
nav_msgs/Path : "/base_local_path" : 経路

sensor_msgs/JointState :  "/joint_states" : 各軸の現在情報

nav_msgs/Path : "/target_path" : トラッキングターゲットの軌跡

### tf ###
(map_frame) -> (base_frame) のtfが必要

### Services ###
std_srvs/Empty : "~start" : 本機能Onするサービス

std_srvs/Empty : "~stop" : 本機能Offするサービス

std_srvs/Empty : "~set_viewpoint_mode_path" : 視点を向ける先を、移動経路にするサービス

std_srvs/Empty : "~set_viewpoint_mode_tracking" : 視点を向ける先を、トラッキングターゲットにするサービス

### Parameters ###
~robot_pose : ロボット自己位置のトピック名(string, default:"global_pose")

~joint_states : 各関節情報トピック名(string, default:"joint_states")

~command : 指令値トピック名(string, default:"command")

~map_frame : マップフレーム名(string, default:"map")

~base_frame : ベースフレーム名(string, default:"base_footprint")

~focus_path_length : 経路上何m先を見るか[m] (double, default:0.5)

~max_rotation_once : 1周期における首パン軸最大旋回量[rad] (double, default:35*M_PI/180)

~head_pan_min : 首パン軸最小メカリミット[rad] (double, default:-210*M_PI/180)

~head_pan_max : 首パン軸最大メカリミット[rad] (double, default:110*M_PI/180)

~fixed_neck_tilt_angle : 首チルト軸固定角度[rad] (double, default:0.0)

~rate : 駆動周期[sec] (double, default:1.0)

~neck_pan_name : 首パン軸名称(string, default:head_pan_joint)

~neck_tilt_name : 首チルト軸名称(string, default:head_tilt_joint)
