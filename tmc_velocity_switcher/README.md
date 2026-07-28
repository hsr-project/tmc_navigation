tmc_velocity_switcher
==============================

概要
-----------------------------
* 複数の速度入力を管理し、現在有効かつ最も優先度の高い速度を出力する。


開発関係者
----------------
* 田中 和仁
* 松野 喜幸
* 城 崇平

ノードの振る舞い
----------------

velocity_switcher
--------------------
* 入力速度には優先度が設定できる。
* 各入力速度は、X軸方向、Y軸方向、回転軸それぞれを制御対象とするか否かを設定できる。
* 入力速度の最終受信時間をチェックし、タイムアウトしていない入力速度を有効とみなす。
* 各軸ごとに、有効な入力速度かつ制御対象かつ、その中で最も優先度が高い入力速度を出力速度とする。

インターフェース
----------------
#### 出版するトピック (トピック名 [型] : 説明)
- "output_velocity" [geometry_msgs/msg/Twist] : 出力速度

#### 購読するトピック (トピック名 [型] : 説明)
- "Topic名はパラメータ指定" [geometry_msgs/msg/Twist] : 入力速度。任意個数指定可能

#### パラメータ (パラメータ名 [型] [単位] : 説明 (default : XX))
- timeout [double] [s] : 入力速度のタイムアウト時間(default : 1.00)

- switching_period [double] [s] : 入力速度をswitchする際の切り替わり時間 (default : 0.50)

- input_velocities : 入力速度群。任意個の入力速度を指定することができる。
    - 入力名 : 入力速度名。任意の名前を指定する。
        - topic_name [string]: 入力速度のTopic名(必須パラメータ)
        - priority [int]: 入力速度優先度。値の若いほうが優先度が高い。全入力速度間で一意となる値を指定すること (必須パラメータ)
        - as_control_target_x [bool] : この入力速度がX軸を制御対象とするか否か。true:する false:しない (default : true)
        - as_control_target_y [bool] : この入力速度がY軸を制御対象とするか否か。true:する false:しない (default : true)
        - as_control_target_theta [bool] : この入力速度が回転軸を制御対象とするか否か。true:する false:しない (default : true)
