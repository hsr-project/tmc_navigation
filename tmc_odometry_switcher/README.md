tmc_odometry_switcher
===============================================================================


概要
-------------------------------

複数のオドメトリトピックからスイッチングしたオドメトリを出力する。


振る舞い
--------------------------------

- 任意個数のオドメトリトピックを入力とし、選択したオドメトリトピックの移動差分を足し合わせたオドメトリを出力する。
- オドメトリトピック切り替えサービスを呼ぶことで、使用するオドメトリトピックを選択できる。
- 入力オドメトリの種類が変わっても、差分を足し合わせる為切り替え前後で出力オドメトリが飛ぶことは無い


開発者
----------------------------

- 田中　和仁(kazuhito_tanaka@mail.toyota.co.jp)


動作確認済環境
----------------------------

- ROS kinetic + Ubuntu16.04


使用方法
----------------------------

1. 切り替え元となる複数のオドメトリソースノードを立ち上げる。各オドメトリソースノードは odom -> base_footprintを結ぶTFを吐かないように設定しておく。

2. launchファイルを立ち上げる。
   引数で入力オドメトリのリスト({入力オドメトリの種類のキー : 入力オドメトリトピック名}の辞書形式で記載)と、最初に使用するオドメトリの種類を指定する。
   roslaunchファイルでの立ち上げ例：

   ```xml
   <launch>
     <include file="$(find tmc_odometry_switcher)/launch/odometry_switcher.launch">
       <arg name="topic_lists" value="{'source1': 'odom_topic_name1', 'source2': 'odom_topic_name2'}" />
       <arg name="initial_odom" value="source1"/>
     </include>
   </launch>
   ```

3. サービスでオドメトリの種類を切り替える
   "source2"キーで指定されたオドメトリに切り替える時：
   ```bash
   $ rosservice call /odometry_switch "odom_type: {data: 'source2'}"
   ```


### ROSインターフェース

#### 出版するトピック
- switched_odom [geometry_msgs/Odometry] : 出力オドメトリ


#### 発行するtf
- odom -> base_footprint
　フレーム名はパラメータで指定できる


#### 購読するトピック
- xxx_odom [geometry_msgs/Odometry] : 入力オドメトリ
　任意個数のトピックをパラメータで指定できる


#### 提供するサービス
- odometry_switch [tmc_odometry_switcher/OdometrySwitch] : オドメトリ切り替えサービス
　request.odom_typeに選択するオドメトリの種類を指定する
　
#### パラメータ
- ~odom_topics : {入力オドメトリの種類のキー : 入力オドメトリトピック名} の任意個の組み合わせを辞書型で入力
- ~initial_odom (string) : 最初に選択する入力オドメトリの種類のキーを指定
- ~odom_frame (string) : 発行するオドメトリのフレーム名
- ~odom_child_frame (string) : 発行するオドメトリの子フレーム名
