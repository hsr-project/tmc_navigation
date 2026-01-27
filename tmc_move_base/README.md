tmc_move_base
===============================================================================

概要
-------------------------------
* 自律移動機能をactionlibインタフェースで提供する。
* ROS move_baseと同じインタフェースを提供する。

開発関係者
-------------------------------
* 高岡 豊
* 田中 和仁
* 城 崇平

ノードの振る舞い
-------------------------------
* ゴール位置(PoseStamped型)を指定すると、そこまで自律移動する
* 移動中、ロボットの現在位置をフィードバックする
* 移動中キャンセルすると、その場で停止する
* 移動中、新しいゴールが与えられると古いゴールを破棄し新しいゴールを採用する
* トピック("/move_base_simple/goal")でゴールを与えることができる

インターフェース
-------------------------------
- 提供するアクション (アクション名 [型] : 説明)
    - "move_base/move" [nav2_msgs::NavigateToPose] : 自律移動アクション

- 利用するアクション (アクション名 [型] : 説明)
    - "base_path_plan" [tmc_base_path_planner/BasePathPlan] : 経路計画アクション

- 購読するトピック (トピック名 [型] : 説明)
    - "global_pose" [geometry_msgs/PoseStamped] : 自己位置
    - "move_base_simple/goal" [geometry_msgs/PoseStamped] : 目的地

- 出版するトピック (トピック名 [型] : 説明)
    - "move_base/move/goal" [move_base_msgs::MoveBaseActionGoal] : 自律移動アクションゴール。目的地トピックを購読したら自身にアクションゴールを出版。

- パラメータ (パラメータ名 [単位] : 説明)
    - planning_timeout [double] : 経路計画内部エラー時のタイムアウト(default:"10.0")
    - floor_frame [string] : floorのフレーム名(default:"map")
    - navigation_log_record_service_name [string] : 異常発生時のログ記録サービス名
    - path_planner_name [string] : path_plannerのノード名
    - path_follower_name [string] : path_followerのノード名

- 利用するtfのフレーム
    - floor_frame -> ゴールのフレーム

ログ機能
-------------------------------
- 自律移動中に異常が発生し、move_base/moveアクションがabortedとなった場合には、デバッグ用にログを記録する。
    - navigation_log_record_service_nameパラメータで名前を指定された外部サービスにログの記録を要求する。
      外部サービスはstd_srvs/Empty型限定。(tmc_rosbag_tools/topic_collection_nodeを使用することを想定し作成)
    - path_planner_nameパラメータ、path_follower_nameパラメータで指定された各ノード及び、自身のrosパラメータをdumpする。
      dumpしたファイルは'(ROS_LOG_DIR)/latest/'に格納する。'move_base_20200401T123456.yaml'のようなファイル名で出力される。
    - navigation_log_record_service_nameパラメータに何も指定されていない、もしくは指定されたサービスが存在しなければ、何も出力しない
