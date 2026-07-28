tmc_velocity_acceleration_limiter
===============================================================================

概要
-------------------------------
* 入力された速度に加減速度制限をかけ出力する

開発関係者
-------------------------------
* 田中 和仁


ノードの振る舞い
-------------------------------
* 入力速度の2D成分(x, y, yaw)個別に対して加減速リミットをかけ出力する
* 速度の並進・旋回成分それぞれに対して加速・減速度を個別に指定できる
* 速度入力が途絶した時は、出力速度を0に減速させ速度の発行を停止する
* 加減速度制限機能有効/無効をサービスで切り替えることができ、無効時は入力速度をそのまま出力する

インターフェース
------------------------------
#### 出版するトピック (トピック名 [型] : 説明)
- "acceleration_limited_velocity" [geometry_msgs/msg/Twist] : 加減速制限がかかった速度

#### 購読するトピック (トピック名 [型] : 説明)
- "command_velocity" [geometry_msgs/msg/Twist] : 入力速度

#### 提供するサービス (サービス名 [型] : 説明)
- "~/enable" [std_srvs/srv/Empty] : 加減速制限機能の有効化
- "~/disable" [std_srvs/srv/Empty] : 加減速制限機能の無効化

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- enable_limit_default [bool] : 加減速制限機能のデフォルト有効/無効設定
- linear_acceleration_limit [double] [m/s^2] : 並進加速度リミット (default : 1.0)
- linear_deceleration_limit [double] [m/s^2] : 並進減速度リミット (default : 1.0)
- angular_acceleration_limit [double] [rad/s^2] : 旋回加速度リミット (default : 1.0)
- angular_deceleration_limit [double] [rad/s^2] : 旋回減速度リミット (default : 1.0)
- input_velocity_timeout [double] [s] : 入力速度が途絶されたと判定するタイムアウト (default : 0.2)
