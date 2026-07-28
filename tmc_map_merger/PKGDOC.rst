Overview
=============


提供機能
--------
複数の入力メッセージから2D占有マップを作成し、それらをマージした2D占有マップを出力する。

- 任意の個数の入力メッセージを許容する。
- 自律移動用途で想定されるROSメッセージタイプをサポートする。
- 入力毎に生成する2D占有マップの大きさ・サンプリング周期等の設定ができる。
- 任意の入力の障害物領域の大きさを実行中に変更できる。

Nodes
-----

- tmc_map_merger

  2D占有マップをマージするノード。

インターフェース
--------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "merged_map" [nav_msgs/msg/OccupancyGrid] : 2D占有マップ

#### 購読するトピック (トピック名 [型] : 説明)
- "パラメータで設定されたトピック名" [パラメータで指定されたROSメッセージタイプ] : 入力メッセージ。以下のタイプに対応。

  - [sensor_msgs/msg/PointCloud2] : 3Dセンサ等から出版されるメッセージタイプ。

  - [sensor_msgs/msg/LaserScan] : レーザーファインダ等から出版されるメッセージタイプ。

  - [nav_msgs/msg/OccupancyGrid] : 2D占有マップを表現するメッセージタイプ。


#### 提供するサービス (サービス名 [型] : 説明)
- "~/reset" [std_srvs/srv/Empty] : 保持しているデータのリセットを行う。

#### 利用する tf フレーム (<親フレーム> -> <子フレーム> : 説明)
- <fixed_frame> -> <入力メッセージのフレーム> : 入力メッセージと基準座標間の姿勢変換

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- fixed_frame [string] : 世界座標の基準となる動かないフレームID。(必須パラメータ)
- origin_frame [string] : 出力する2D占有マップのフレームID。(必須パラメータ)
- publish_rate [double] [Hz] : 発行周期 (default : 1.0)
- root : 親マージ器の設定パラメータ。
    - merger [string] : 2D占有マップマージ器のタイプ。(simple, simple_memoryの2タイプ）(必須パラメータ)
    - width [double] [m]: 2D占有マップの幅 (必須パラメータ)
    - height [double] [m]: 2D占有マップの高さ (必須パラメータ)
    - resolution [double] [m/cell] : 2D占有マップの解像度 (必須パラメータ)
    - keep_time [double] [s] : 2D占有マップマージ器タイプがsimple_memoryの場合のみ有効。各グリッド占有値保持時間。0の時は保持し続ける (必須パラメータ)
    - default_data [int] : 2D占有マップマージ器タイプがsimple_memoryの場合のみ有効。-1〜100の範囲内の値となるように設定する必要がある。(必須パラメータ)
- inputs : 入力設定パラメータ。以下のパラメータを1以上の任意の数設定することができる。
    - 入力名 : 入力設定パラメータ名。任意の名前を設定する。
        - enable [bool] : 入力データのON/OFF設定。Falseにすると該当のセンサデータはmapに反映しない。(default : true)
        - topic_name [string] : 購読トピック名。(必須パラメータ)
        - type [string] : 購読トピックのROSメッセージタイプ。(必須パラメータ)
  　    - rate [double] [Hz] : 発行周期 (default : 1.0)
        - merger [string] : 2D占有マップマージ器のタイプ。以下の3タイプを提供。
            - simple : 空間方向のマージを行う。
            - simple_memory : 時間方向のマージを行う。観測した情報をそのまま記憶する。
            - safety_memory : 時間方向のマージを行う。観測した情報が記憶された情報より大きければ記憶する。
        - width [double] [m] : 2D占有マップの幅 (必須パラメータ)
        - height [double] [m] : 2D占有マップの高さ (必須パラメータ)
        - resolution [double] [m/cell] : 2D占有マップの解像度 (必須パラメータ)
        - keep_time [double] [s] : 2D占有マップマージ器タイプがsimple_memoryの場合のみ有効。各グリッド占有値保持時間。0の時は保持し続ける。(必須パラメータ)
        - default_data [int] : 2D占有マップマージ器タイプがsimple_memoryの場合のみ有効。-1〜100の範囲内の値となるように設定する必要がある。(必須パラメータ)
        - point_cloud_filters : 入力メッセージデータのフィルタ。
            - order [int] : フィルタを実行する順番。設定のないフィルタは最後尾に回される。(default : 9999)
            - type [string] : フィルタのタイプ。以下の4タイプを提供。(必須パラメータ)
                - voxel_grid : 空間を設定されたVoxelで区切り、その中に含まれる代表点を用いる。（ダウンサンプリング）
                - trimming : 指定された軸の上下限を設定し、その範囲外の点を除く。
                - transform : 入力データをorigin_frame基準に座標変換を行う。
                - noise : 基準点を中心とした円領域内に含まれるデータの数が設定値以下の場合、ノイズとして扱い除く。
            - leaf_size [double] [m] : voxel_gridタイプの場合のみ有効。ダウンサンプル間隔 (default : 0.05)
            - field_name [string] : trimmingタイプの場合のみ有効。軸(x, y, zのいずれか） (default : "z")
            - min [double] [m] : trimmingタイプの場合のみ有効。使用する点の下限値 (default : 0.15)
            - max [double] [m] : trimmingタイプの場合のみ有効。使用する点の上限値 (default : min * 2)
            - radius [double] [m] : noiseタイプの場合のみ有効。隣接点探索範囲円の半径 (default : 0.0)
            - min_neighbors [int] : noiseタイプの場合のみ有効。ノイズ判断点数。隣接点探索範囲円に含まれる点の数がこれ以下の場合ノイズであると判断される。(default : 0)
        - obstacle_circle : センサ点を中心とした円状の障害物領域を描画するためのパラメータ。現状、凸状のプロファイルを持つシリンダー形状。中心に近い領域は完全占有とした円状の領域となる。その外側を任意の値に設定することができる。
            - forbid_radius [double] [m] : 完全占有とする円の半径 (必須パラメータ)
            - obstacle_radius [double] [m] : 任意の値を設定する円の半径 (default : 0.0)
            - obstacle_occupancy [int] [%] : 任意の値を設定する円の占有値 (default : 0)


How to use
-----------------------

1. ロボット起動

2. ノード起動

.. code-block:: bash

    $ ros2 launch tmc_map_merger map_merger.launch.py

3. 2D占有マップの出版を確認

.. code-block:: bash

    $ ros2 topic hz /dynamic_obstacle_map

4. Rvizで可視化