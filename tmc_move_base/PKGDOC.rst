Overview
++++++++

提供機能
--------
ロボット台車を障害物を回避しながら指定位置まで自律移動させる機能を提供します。


ROS Interface
++++++++++++++

Nodes
-----

- **move_base** 自律移動機能提供ノード

Subscribed Topics
^^^^^^^^^^^^^^^^^
- **/move_base_simple/goal** (:ros:msg:`geometry_msgs/PoseStamped`) 自律移動ゴール位置

.. note:: 入力ゴール位置は、/move_base_simple/goal/header/frame_id にて指定されたTFフレーム基準で入力されます。

- **/global_pose** (:ros:msg:`geometry_msgs/PoseStamped`) ロボットの現在位置


Actions
^^^^^^^^
- **~move** (:ros:msg:`nav2_msgs/NavigateToPose`) 自律移動を提供するアクション

.. note:: 入力ゴール位置は、~move_base/goal/goal/target_pose/header/frame_id にて指定されたTFフレーム基準で入力されます。

Parameter
^^^^^^^^^
- **~planning_timeout** (``double``: ``10.0``) 経路計画内部エラー時自律移動をキャンセルするタイムアウト[s]

- **~floor_frame** (``string``: ``"map"``) フロア基準TFフレーム名

.. note:: 上記パラメータの他に、自律移動の速度・加減速度等を変更したい場合は、下記のパラメータを変更ください。

- **/omni_path_follower/base_max_linear_velocity** (``double``: ``"0.2"``) 台車の最大並進速度[m/s]。この値が大きいほど速く並進移動を行います。
- **/omni_path_follower/base_max_angular_velocity** (``double``: ``"0.5"``) 台車の最大旋回速度[rad/s]。この値が大きいほど速く旋回を行います。
- **/omni_path_follower/base_max_linear_acceleration** (``double``: ``"0.3"``) 台車の最大並進加減速度[m/s^2]。この値が大きいほど大きく加減速を行います。
- **/omni_path_follower/base_max_angular_acceleration** (``double``: ``"0.5"``) 台車の最大旋回加減速度[rad/s^2]。この値が大きいほど大きく加減速を行います。
- **/map_merger/inputs/{input_source_name}/obstacle_circle/forbid_radius** (``double``: ``0.25``) input_source_nameで示されるそれぞれのセンサで検知した障害物周辺に張られる侵入禁止領域の幅[m]。この値が大きいほど障害物に対し余裕を持って回避するようになりますが、障害物の隙間等の通過が難しくなります。




How to use
++++++++++
以下のチュートリアルを参考にしてください。

:ref:`自律移動のチュートリアル <navigation-tutorial>`
