tmc_base_path_planner
===============================================================================

概要
-------------------------------
* 経路計画機能。現在位置から与えられたゴールまでの経路を、障害物マップに干渉しないよう計画する。

開発関係者
-------------------------------
* 高岡 豊
* 田中 和仁
* 城　崇平

ノードの振る舞い
-------------------------------
* 経路計画IFはアクションで提供する。
* 経路計画に成功したら経路追従の要求を発行する。
* ゴールに到達するまでの間は経路を計画し続け、経路に変更が発生すれば経路追従の要求を再発行する。
* 経路計画に失敗した場合は経路追従の要求をキャンセルする。
* 経路計画に成功しており経路追従要求中であるのか、経路計画に失敗して再経路計画中であるのかは、アクションフィードバックで通知する。
* ゴールに到達したら、アクションをSUCCEEDEDで終了する。
* 経路計画がキャンセルされたり他の要求に割り込まれた場合は、アクションをPREEMPTEDで終了する。
* 経路計画が不可能となった場合は、アクションをABORTEDで終了する。
* 静的マップが事前に入力されていること、動的マップと自己位置が定期的に入力されることを前提とする。これらを満たさない場合経路計画は不可。
* 自己位置のフレームは「/map」基準を前提とする。

インターフェース
-------------------------------

#### 出版するトピック (トピック名 [型] : 説明)
- "base_local_path" [nav_msgs/msg/Path] : 計画した経路

#### 購読するトピック (トピック名 [型] : 説明)
- "static_obstacle_ros_map" [nav_msgs/msg/OccupancyGrid] : 静的マップ

- "dynamic_obstacle_map" [nav_msgs/msg/OccupancyGrid] : 動的マップ

- "global_pose" [geometry_msgs/msg/PoseStamped] : 自己位置(/mapフレーム以外は非対応)

#### 提供するアクション(アクション名 [型] : 説明)
- "base_path_plan" [tmc_navigation_msgs/action/BasePathPlan] : 経路計画アクション

#### 利用するアクション (アクション名 [型] : 説明)
- "path_follow_action" [tmc_navigation_msgs/action/PathFollower] : 経路追従アクション

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- node : ノードの基本設定
    - rate [double] [hz] : 駆動周期 (default : 10.0)
    - dynamic_map_timeout [double] [s] : 動的マップタイムアウト時間 (default : 3.0)
    - global_pose_timeout [double] [s] : 自己位置タイムアウト時間 (default : 2.0)
    - static_map_potential_width [double] [m] : 静的マップの障害物領域を壁から膨張させる距離 (default : 3.0)
- base_path_planner : 経路計画に関する設定
    - type [string] : 経路計画手法(下記参照)
    - astar_path_planner : A*アルゴリズムの設定
        - exclusive_size [double] [m] : 禁止領域 (default : 0.2)
        - potential_size [double] [m] : ポテンシャル領域 (default : 0.7)
        - single_cost [int]: 隣接グリッド間の移動コスト(整数) (default : 50)
        - diagonal_cost [int] : 斜め方向グリッドの移動コスト(整数) (default : 71)
        - cost_unknown [int] : 未知領域を意味するグリッドのコスト値(整数) (default : 255)
        - cost_factor [double]: スタート-ゴール間の距離に対する最大コスト見積もりの倍率(小数)  {(Xグリッド差分 + Yグリッド差分) * cost_factor}以内のコストでゴールに着かなければ到達不可とする。 (default : 150.0)
        - cost_on_preferred_path [int] : 前回計画した経路を優先するために、前回経路上に設定する補正コスト(0以下の整数) (default : 0)
        - cost_around_preferred_path [int] : 前回計画した経路を優先するために、前回経路周辺に設定する補正コスト(0以下の整数).cost_on_preferred_path <= cost_around_preferred_pathであること。(default : 0)
    - map_filter : 地図フィルタリングに関する設定
        - map_filter_range_around_start [double] [m] : スタート周辺の禁止領域を削除する範囲 (default : 0.0)
        - map_filter_range_around_goal [double] [m] : ゴール周辺の禁止領域を削除する範囲 (default : 0.5)
        - map_filter_distance_goal_limit [double] [m] : ゴール周辺禁止領域フィルタを行うゴールまでの距離 (default : 1.0)
    - path_updater : 経路更新に関する設定
        - distance_on_prev_path [double] [m] : 前回の経路上にいるとみなす前回経路までの距離 (default : 0.15)
        - grid_error [double] [m] : 経路点同士のずれ許容値 (default : 0.1)
        - same_point_num_merge_path [int] : 何点目まで一致していたら前回経路をマージするかの閾値 (default : 3)

- 経路計画手法
  base_path_planner/typeパラメータにより、経路計画手法を切り替える。下記手法が存在する
  ※現状はastar_path_plannerのみだが、新たな経路計画手法を追加する場合はここに列挙する
    - "astar_path_planner"
      A*グリッド探索により、スタートからゴールまでの最短のグリッド経路を探索する。
      探索したグリッド経路を５次補間で平滑化する。
