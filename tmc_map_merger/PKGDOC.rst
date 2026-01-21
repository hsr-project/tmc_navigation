Overview
++++++++


提供機能
--------
複数の入力メッセージから2D占有マップを作成し、それらをマージした2D占有マップを出力する。

- 任意の個数の入力メッセージを許容する。
- 自律移動用途で想定されるROSメッセージタイプをサポートする。
- 入力毎に生成する2D占有マップの大きさ・サンプリング周期等の設定ができる。
- 任意の入力の障害物領域の大きさを実行中に変更できる。


ROS Interface
++++++++++++++


Nodes
-----

- **tmc_map_merger**

  2D占有マップをマージするノード。


Subscribed Topics
^^^^^^^^^^^^^^^^^

- **パラメータで設定されたトピック名** (:ros:msg:`パラメータで指定されたROSメッセージタイプ`)

  入力メッセージ。以下のタイプに対応。

    * (:ros:msg:`sensor_msgs/PointCloud2`)

      3Dセンサ等から出版されるメッセージタイプ。

    * (:ros:msg:`sensor_msgs/LaserScan`)

      レーザーファインダ等から出版されるメッセージタイプ。

    * (:ros:msg:`nav_msgs/OccupancyGrid`)

      2D占有マップを表現するメッセージタイプ。

- **/tf** (:ros:msg:`tf/tfMessage`)

  入力メッセージと基準座標間の姿勢変換を解決するためのTF。

- **/tf_static** (:ros:msg:`tf2_msgs/TFMessage`)

  静的な座標変換を解決するためのTF。（主にロボットの関節など）


Published Topics
^^^^^^^^^^^^^^^^

- **merged_map** (:ros:msg:`nav_msgs/OccupancyGrid`)

  2D占有マップ。


Parameters
^^^^^^^^^^

- **~fixed_frame** (``string``)

  世界座標の基準となる動かないフレームID。

- **~origin_frame** (``string``)

  出力する2D占有マップのフレームID。

- **~publish_rate** (``double``: ``1.0``)

  発行周期[Hz]

- **~root**

  親マージ器の設定パラメータ。

  * **merger** (``string``)

    2D占有マップマージ器のタイプ。(simple, simple_memoryの2タイプ）

  * **width** (``double``)

    2D占有マップの幅[m]

  * **height** (``double``)

    2D占有マップの高さ[m]

  * **resolution** (``double``)

    2D占有マップの解像度[m/cell]

  * **keep_time** (``double``)

    2D占有マップマージ器タイプがsimple_memoryの場合のみ有効。

    各グリッド占有値保持時間[sec]

    0の時は保持し続ける。

  * **default_data** (``int``)

    2D占有マップマージ器タイプがsimple_memoryの場合のみ有効。

    -1〜100の範囲内の値となるように設定する必要がある。

- **~inputs**

  入力設定パラメータ。

  以下のパラメータを1以上の任意の数設定することができる。

  * **入力名** 

    入力設定パラメータ名。

    任意の名前を設定する。

    + **enable** : (``bool``)

      入力データのON/OFF設定。Falseにすると該当のセンサデータはmapに反映しない。

    + **topic_name** : (``string``)

      購読トピック名。

    + **type** : (``string``)

      購読トピックのROSメッセージタイプ。

    + **rate** (``double``: ``1.0``)

      発行周期[Hz]

    + **merger** (``string``)

      2D占有マップマージ器のタイプ。以下の3タイプを提供。

      - simple

        空間方向のマージを行う。

      - simple_memory

        時間方向のマージを行う。観測した情報をそのまま記憶する。

      - safety_memory

        時間方向のマージを行う。観測した情報が記憶された情報より大きければ記憶する。

    + **width** (``double``)

      2D占有マップの幅[m]

    + **height** (``double``)

      2D占有マップの高さ[m]

    + **resolution** (``double``)

      2D占有マップの解像度[m/cell]

    + **keep_time** (``double``)

      2D占有マップマージ器タイプがsimple_memoryの場合のみ有効。

      各グリッド占有値保持時間[sec]

      0の時は保持し続ける。

    + **default_data** (``int``)

      2D占有マップマージ器タイプがsimple_memoryの場合のみ有効。

      -1〜100の範囲内の値となるように設定する必要がある。

    + **point_cloud_filters**

      入力メッセージデータのフィルタ。

      - **order** (``int``)

        フィルタを実行する順番。設定のないフィルタは最後尾に回される。

      - **type** (``string``)

        フィルタのタイプ。以下の4タイプを提供。

          * voxel_grid

            空間を設定されたVoxelで区切り、その中に含まれる代表点を用いる。（ダウンサンプリング）

          * trimming

            指定された軸の上下限を設定し、その範囲外の点を除く。

          * transform

            入力データをorigin_frame基準に座標変換を行う。

          * noise

            基準点を中心とした円領域内に含まれるデータの数が設定値以下の場合、ノイズとして扱い除く。

      - **leaf_size** (``double``)

        voxel_gridタイプの場合のみ有効。ダウンサンプル間隔[m]

      - **field_name** (``string``)

        trimmingタイプの場合のみ有効。軸(x, y, zのいずれか）

      - **min** (``double``)

        trimmingタイプの場合のみ有効。使用する点の下限値[m]

      - **max** (``double``)

        trimmingタイプの場合のみ有効。使用する点の上限値[m]

      - **radius** (``double``)

        noiseタイプの場合のみ有効。隣接点探索範囲円の半径[m]

      - **min_neighbors** (``int``)

        noiseタイプの場合のみ有効。ノイズ判断点数。

        隣接点探索範囲円に含まれる点の数がこれ以下の場合ノイズであると判断される。

    + **obstacle_circle**

      センサ点を中心とした円状の障害物領域を描画するためのパラメータ。

      現状、凸状のプロファイルを持つシリンダー形状。

      中心に近い領域は完全占有とした円状の領域となる。

      その外側を任意の値に設定することができる。

      - **forbid_radius** (``double``)

        完全占有とする円の半径[m]

      - **obstacle_radius** (``double``)

        任意の値を設定する円の半径[m]

      - **obstacle_occupancy** (``int``)

        任意の値を設定する円の占有値[%]


Services
^^^^^^^^

- **~reset** (:ros:srv:`std_srvs/Empty`)

  保持しているデータのリセットを行う。


How to use
++++++++++

1. ロボット起動

2. ノード起動

.. code-block:: bash

    $ roslaunch tmc_map_merger sample.launch

3. 2D占有マップの出版を確認

.. code-block:: bash

    $ rostopic hz /merged_map -w 5

4. Rvizで可視化

5. rqt_reconfigureを用いた障害物領域の変更

   以下のコマンド実行後、tmc_map_merger/inputs/base_scan/obstacle_circleを選択を選択しパラメータ変更を行う。

   障害物領域が変化することを確認する。

.. code-block:: bash

    $ rosrun rqt_reconfigure rqt_reconfigure

6. rqt_reconfigureを用いた入力のON/OFF

   5.のコマンド実行後、tmc_map_merger/inputs/base_scanを選択し入力のON/OFF切り替えを行う。

   障害物領域が変化することを確認する。

