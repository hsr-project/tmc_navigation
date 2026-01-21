概要
=============================
* 複数の速度入力を管理し、現在有効かつ最も優先度の高い速度を出力する。


開発関係者
================
* 田中 和仁
* 松野 喜幸
* 城 崇平

ノードの振る舞い
================

velocity_switcher
--------------------
* 入力速度には優先度が設定できる。
* 各入力速度は、X軸方向、Y軸方向、回転軸それぞれを制御対象とするか否かを設定できる。
* 入力速度の最終受信時間をチェックし、タイムアウトしていない入力速度を有効とみなす。
* 各軸ごとに、有効な入力速度かつ制御対象かつ、その中で最も優先度が高い入力速度を出力速度とする。

インターフェース
================
### Published Topics ###
geometry_msgs::Twist : "output_velocity" : 出力速度

### Subscribed Topics ###
geometry_msgs::Twist : Topic名はパラメータ指定 : 入力速度。任意個数指定可能

### Parameters ###
~timeout : 入力速度のタイムアウト時間(double, default:"1.00"sec)

~switching_period :  入力速度をswitchする際の切り替わり時間(double, default:"0.50"sec)

~input_velocities : 入力速度群。任意個の入力速度を指定することができる。
  + 入力名 : 入力速度名。任意の名前を指定する。
    - topic_name : 入力速度のTopic名(string, 必須パラメータ)
    - priority : 入力速度優先度。値の若いほうが優先度が高い。全入力速度間で一意となる値を指定すること(int, 必須パラメータ)
    - as_control_target_x : この入力速度がX軸を制御対象とするか否か。true:する false:しない(bool, default:"true")
    - as_control_target_y : この入力速度がY軸を制御対象とするか否か。true:する false:しない(bool, default:"true")
    - as_control_target_theta : この入力速度が回転軸を制御対象とするか否か。true:する false:しない(bool, default:"true")
