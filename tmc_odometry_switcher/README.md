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



使用方法
----------------------------

1. 切り替え元となる複数のオドメトリソースノードを立ち上げる。各オドメトリソースノードは <odom> -> <base_footprint> を結ぶTFを吐かないように設定しておく。

2. launchファイルを立ち上げる。
   ```bash
   $ ros2 launch tmc_odometry_switcher odometry_switcher.launch.py
   ```

   パラメータで入力オドメトリのリスト({入力オドメトリの種類のキー : 入力オドメトリトピック名}の辞書形式で記載)と、最初に使用するオドメトリの種類を指定する。
   指定例は、launch/odometry_switcher.launch.py, config/odometry_switcher.yamlを参照

3. サービスでオドメトリの種類を切り替える
   "wheel_odom"キーで指定されたオドメトリに切り替える時：
   ```bash
   $ ros2 service call /odometry_switch tmc_navigation_msgs/srv/OdometrySwitch "odom_type: {data: 'wheel_odom'}"
   ```

インターフェース
--------------------

#### 出版するトピック (トピック名 [型] : 説明)
- "switched_odom" [geometry_msgs/msg/Odometry] : 出力オドメトリ

#### 購読するトピック (トピック名 [型] : 説明)
- "xxx_odom" [geometry_msgs/msg/Odometry] : 入力オドメトリ.任意個数のトピックをパラメータで指定できる

#### 提供するサービス (サービス名 [型] : 説明)
- "odometry_switch" [tmc_navigation_msgs/srv/OdometrySwitch] : オドメトリ切り替えサービス.request.odom_typeに選択するオドメトリの種類を指定する

#### 発行する tf (<親フレーム> -> <子フレーム> : 説明)
- <odom> -> <base_footprint> : フレーム名はパラメータで指定できる

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- odom_topics [string : string] : {入力オドメトリの種類のキー : 入力オドメトリトピック名} の任意個の組み合わせを辞書型で入力
- initial_odom [string] : 最初に選択する入力オドメトリの種類のキーを指定 (必須パラメータ)
- odom_frame [string] : 発行するオドメトリのフレーム名 (default : "odom")
- odom_child_frame [string] : 発行するオドメトリの子フレーム名 (default : "base_footprint")
