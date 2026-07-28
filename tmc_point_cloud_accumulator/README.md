tmc_point_cloud_accumulator
===============================================================================


概要
-------------------------------

センサから取得されたポイントクラウドを2次元平面に投影・ダウンサンプリングしたうえで、
一定時間蓄積・保存を行う


振る舞い
--------------------------------

センサから取得したポイントクラウド"input_point_cloud"を購読し、
2次元平面に投影・ダウンサンプリングを行いパラメータpoint_cloud_keep_periodで指定した秒数保持し
ポイントクラウドを蓄積する。
周期ごとに、蓄積されているポイントクラウドをマージしたトピック"accumulated_point_cloud"を発行する。
ただし、パラメータcut_point_cloud_angleで指定されるロボット前方視野角内のものは、
現在購読されているポイントクラウドのデータで上書きされる。  

ポイントクラウド蓄積を行うかどうかのスイッチを持ち、サービス経由で変更することができる。  

開発者
----------------------------

- 田中　和仁(kazuhito_tanaka@mail.toyota.co.jp)


インターフェース
--------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "accumulated_point_cloud" [sensor_msgs/msg/PointCloud2] : 一定時間蓄積されたポイントクラウド

#### 購読するトピック (トピック名 [型] : 説明)
- "input_point_cloud" [sensor_msgs/msg/PointCloud2] : 入力ポイントクラウド

#### 提供するサービス (サービス名 [型] : 説明)
- "start_accumulate_point_cloud" [std_srvs/srv/Empty] : ポイントクラウド蓄積の開始

- "stop_accumulate_point_cloud" [std_srvs/srv/Empty] : ポイントクラウド蓄積の停止

#### 利用する tf (<親フレーム> -> <子フレーム> : 説明)
- <map_frame_name> -> <base_frame_name> : map座標系におけるbase_linkのtf

- <odom_frame_name> -> <base_frame_name> : odom座標系におけるbase_linkのtf

- <base_frame_name> -> <入力メッセージのフレーム> : base_link座標系における入力メッセージフレームのtf

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- map_frame_name [string] : 基準座標tf名。(default : 'map')
- odom_frame_name [string] : オドメトリtf名。(default : 'odom')
- base_frame_name [string] : ロボット位置tf名。(default : 'base_link')
- bottom_of_valid_space [double] [m] : 蓄積させるポイントクラウドの高さの下限値。(default : 0.15)
- top_of_valid_space [double] [m] : 蓄積させるポイントクラウドの高さの上限値。(default : 1.5)
- voxel_leaf_size [double] [m] : ダウンサンプリング時の点群間隔。(default : 0.025)
- min_saved_area_radius [double] [m] : ポイントクラウドを保存する際のロボットからの距離最小半径閾値。(default : 0.2)
- max_saved_area_radius [double] [m] : ポイントクラウドを保存する際のロボットからの距離最大半径閾値。(default : 3.0)
- cut_point_cloud_angle [double] [deg] : ロボット正面基準として過去ポイントクラウドデータをカットする視野角。(default : 60.0)
- point_cloud_keep_period [double] [s] : 過去のポイントクラウドを蓄積する期間。(default : 6.0)
- trimming_data_radius [double] [m] : 取得したポイントクラウドをトリミング判定するロボット中心からの距離半径。(default : 0.3)
- noise_filter_radius_search [double] [m] : ポイントクラウドノイズフィルターのパラメータ 探索範囲 (default : 0.1)
- noise_filter_neighbors [int] : ポイントクラウドノイズフィルターのパラメータ 探索範囲内に含まれる点の数 (default : 1)
<br>
<br>
<br>

tmc_point_cloud_merger
===============================================================================


概要
-------------------------------

指定の範囲内にある2つのポイントクラウドを、座標系を合わせてマージする。

振る舞い
--------------------------------

センサから取得した2つのポイントクラウド"input_point_cloud_1"および"input_point_cloud_2"を購読する。
指定の範囲内にあるデータに関して、input_point_cloud_1の座標系でマージする。
マージ後のポイントクラウドを"merged_point_cloud"として発行する。

開発者
----------------------------

- 田中　和仁(kazuhito_tanaka@mail.toyota.co.jp)


インターフェース
--------------------

#### 出版するトピック (トピック名 [型] : 説明)
- "merged_point_cloud" [sensor_msgs/msg/PointCloud2] : マージされたポイントクラウド

#### 購読するトピック (トピック名 [型] : 説明)
- "input_point_cloud_1" [sensor_msgs/msg/PointCloud2] : 入力ポイントクラウド1

- "input_point_cloud_2" [sensor_msgs/msg/PointCloud2] : 入力ポイントクラウド2

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- cloud_1_range [double] [m] : 入力ポイントクラウド1の有効計測範囲 (default : 30.0)
- cloud_2_range [double] [m] : 入力ポイントクラウド2の有効計測範囲 (default : 30.0)