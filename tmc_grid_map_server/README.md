 開発関係者
================
- 高岡　豊
- 藪下　英典
  
目的
=================
自己位置推定用マップと障害物回避用マップを送信する．
  
前提条件
================
- ２次元グリッド地図を扱う.  


振る舞い
================
地図の画像ファイルと設定ファイルを読み込んで、自己位置推定マップと障害物マップを配信する  
ROS serviceで指定される画像ファイルと設定ファイルを読み込んで、自己位置推定マップと障害物マップを再配信する  
ROS map_serverで発行された地図トピックを，自己位置推定マップと障害物マップとして再配信する  
マップはnav_msgs/OccupancyGrid型とtmc_navigation_msgs/OccupancyGridUint型の２つの形式を発行する  
nav_msgs/OccupancyGrid型のデータは、Free、Unknown、Wallの三値を格納する。パラメータの指定によってはUnknownはFreeに置き換えることも出来る  
tmc_navigation_msgs/OccupancyGridUint型は、Free、Unknown、Wallに加えて、Wallからの距離に応じたポテンシャル値を格納する  
ポテンシャルを張る距離はパラメータで指定できる  
(TODO:tmc_navigation_msgs/OccupancyGridUint型を廃止し、nav_msgs/OccupancyGrid型に統一する。再配信機能も不要になるので削除する)

nav_msgs/OccupancyGrid型とtmc_navigation_msgs/OccupancyGridUint型について
================
二つの型は共にheader,info,dataの情報を持つ。header,infoは同じ仕様だが、dataの仕様が異なる
dataの仕様の差は以下の3点
### 1.型 ###
#### nav_msgs/OccupancyGrid ####
int8の配列型
#### tmc_navigation_msgs/OccupancyGridUint ####
uint8の配列型

### 2.値の意味 ###
#### nav_msgs/OccupancyGrid ####
Occupancy: [0, 100] (Free:0 WALL:100)
Unknown: -1
#### tmc_navigation_msgs/OccupancyGridUint ####
Occupancy: [1, 255] (Free:1 WALL:255)
Unknown: 0

### 3.データの格納順 ###
#### nav_msgs/OccupancyGrid ####
地図の左下から右上の順番で格納
例：3x3の地図上のindex
|-----------|
| 6 | 7 | 8 |
|-----------|
| 3 | 4 | 5 |
|-----------|
| 0 | 1 | 2 |
|-----------|

#### tmc_navigation_msgs/OccupancyGridUint ####
地図の左上から右下の順番で格納。nav_msgs/OccupancyGrid型とは上下逆
例：3x3の地図上のindex
|-----------|
| 0 | 1 | 2 |
|-----------|
| 3 | 4 | 5 |
|-----------|
| 6 | 7 | 8 |
|-----------|

インターフェース
================
### map.yaml フォーマット  ###
ROS map_serverのmap.yamlフォーマットに準拠

(TMC option フォーマット)

自己位置推定マップ、または障害物マップの画像ファイルをそれぞれ個別に指定したいとき

distance_map:

obstacle_map:

のタグを追加して画像ファイルを追加する

### 発行するトピック ###
    - static_distance_ros_map (nav_msgs/OccupancyGrid) : 自己位置推定用のグリッド地図(mapフレーム)．
    - static_obstacle_ros_map (nav_msgs/OccupancyGrid) : 障害物回避用のグリッド地図(mapフレーム)．
    - static_distance_map (tmc_navigation_msgs/OccupancyGridUint) : static_distance_ros_mapをtmc_navigation_msgs/OccupancyGridUint型に変換し、壁からポテンシャル幅分膨張させた地図
    - static_obstacle_map (tmc_navigation_msgs/OccupancyGridUint) : static_obstacle_ros_mapをtmc_navigation_msgs/OccupancyGridUint型に変換し、壁からポテンシャル幅分膨張させた地図

### 購読するトピック ###
    - update_map (nav_msgs/OccupancyGrid) : ROS標準のグリッド地図
      gmappingやhector_mapping等で作成した2Dの専有格子地図

### 提供するサービス ###
    - reload_map (tmc_navigation_msgs/ReloadMap) : 地図の再配信サービス
      Request.new_map_yaml (string) で指定されるyamlファイルを読み込み地図を再配信する
      Response.is_success (bool) で地図の再読み込み成否が返される
      サービス自体は常にTrueを返す

### パラメータ ###
- ~potential_width : 壁から何ｍポテンシャルを張るか(double, default: "3.0"m) tmc_navigation_msgs/OccupancyGridUint型の発行トピックに対してのみ有効
- ~convert_unknown_to_free : UnknownをFreeに置き換えるか(bool, default: "true") nav_msgs/OccupancyGrid型の発行トピックに対してのみ有効

使用例
==============
### mapの読み込み方 ###
    $rosrun tmc_grid_map_server grid_map_server map.yaml

### mapの再読み込み ###
mapの再読み込みサービスを呼ぶと、動作中にマップを切り替えることができる。
サービスのリクエストに、マップのyamlファイルのパスを指定する。
(ファイルパスは、絶対パスもしくはホームディレクトリ(~/)から始まるパスが有効)

例) コマンドラインからnew_mapにマップを切り替えたいとき

    $ rosservice call /reload_map "new_map_yaml: '~/xxx/yyy/new_map.yaml'"


### 使用したいマップの切り替え方 ###
select_map <map_file_name> を使って$HOME/.ros/mapにリンクを張る

例）tmc_potential_mapsの201のマップを使いたいとき

    $select_map 201

### map_serverを用いたmapの動的更新 ###
rosrun map_server map_server <map_file_name>を使って,/update_mapを発行する．

例）new_mapを使いたいとき

    $ rosrun map_server map_server new_map.yaml /map:=/update_map
