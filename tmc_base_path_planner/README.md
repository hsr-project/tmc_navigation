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
- 提供するアクション (アクション名 [型] : 説明)
    - "base_path_plan" [tmc_base_path_planner/BasePathPlan] : 経路計画アクション

- 利用するアクション (アクション名 [型] : 説明)
    - "path_follow_action" [tmc_omni_path_follower/PathFollower] : 経路追従アクション

- 出版するトピック (トピック名 [型] : 説明)
    - "base_local_path" [nav_msgs/Path] : 計画した経路

- 購読するトピック (トピック名 [型] : 説明)
    - "static_obstacle_ros_map" [nav_msgs/OccupancyGrid] : 静的マップ
    - "dynamic_obstacle_map" [nav_msgs/OccupancyGrid] : 動的マップ
    - "global_pose" [geometry_msgs/PoseStamped] : 自己位置(/mapフレーム以外は非対応)

- パラメータ (パラメータ名 [単位] : 説明)
    - ~node/rate [hz] : 駆動周期
    - ~node/dynamic_map_timeout [sec] : 動的マップタイムアウト時間
    - ~node/global_pose_timeout [sec] : 自己位置タイムアウト時間
    - ~node/static_map_potential_width [m] : 静的マップの障害物領域を壁から膨張させる距離[m]
    - ~base_path_planner/type [string] : 経路計画手法(下記参照)
    - ~base_path_planner/astar_path_planner/exclusive_size [m] : 禁止領域
    - ~base_path_planner/astar_path_planner/potential_size [m] : ポテンシャル領域
    - ~base_path_planner/astar_path_planner/single_cost : 隣接グリッド間の移動コスト(整数)
    - ~base_path_planner/astar_path_planner/diagonal_cost : 斜め方向グリッドの移動コスト(整数)
    - ~base_path_planner/astar_path_planner/cost_unknown : 未知領域を意味するグリッドのコスト値(整数)
    - ~base_path_planner/astar_path_planner/cost_factor : スタート-ゴール間の距離に対する最大コスト見積もりの倍率(小数)  
                                        {(Xグリッド差分 + Yグリッド差分) * cost_factor}以内のコストでゴールに着かなければ到達不可とする。
    - ~base_path_planner/astar_path_planner/cost_on_preferred_path : 前回計画した経路を優先するために、前回経路上に設定する補正コスト(0以下の整数)
    - ~base_path_planner/astar_path_planner/cost_around_preferred_path : 前回計画した経路を優先するために、前回経路周辺に設定する補正コスト(0以下の整数)  
      cost_on_preferred_path <= cost_around_preferred_pathであること。
    - ~base_path_planner/map_filter/map_filter_range_around_start [m] : スタート周辺の禁止領域を削除する範囲
    - ~base_path_planner/map_filter/map_filter_range_around_goal [m] : ゴール周辺の禁止領域を削除する範囲
    - ~base_path_planner/map_filter/map_filter_distance_goal_limit [m] : ゴール周辺禁止領域フィルタを行うゴールまでの距離
    - ~base_path_planner/path_updater/distance_on_prev_path [m] : 前回の経路上にいるとみなす前回経路までの距離
    - ~base_path_planner/path_updater/grid_error [m] : 経路点同士のずれ許容値
    - ~base_path_planner/path_updater/same_point_num_merge_path [num] : 何点目まで一致していたら前回経路をマージするかの閾値

- 経路計画手法
  base_path_planner/typeパラメータにより、経路計画手法を切り替える。下記手法が存在する
  ※現状はastar_path_plannerのみだが、新たな経路計画手法を追加する場合はここに列挙する
    - "astar_path_planner"
      A*グリッド探索により、スタートからゴールまでの最短のグリッド経路を探索する。
      探索したグリッド経路を５次補間で平滑化する。
