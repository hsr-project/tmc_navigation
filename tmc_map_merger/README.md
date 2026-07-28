tmc_map_merger
==============

開発関係者
----------
* 西野 環
* 川田 福和
* 城 崇平
* 小野田 覚


インターフェース,使用方法について
------------------------
PKGDOC.rstに記載。


設計方針
--------
Strategyパターンをtemplateで実装
* コンパイル時にインライン化されパフォーマンス向上。

参考ロジック
------------
* Cohen-Sutherland algorithm

  https://en.wikipedia.org/wiki/Cohen-Sutherland_algorithm  
  直線描画時のクリッピングに用いている。

* 円領域の塗りつぶしアルゴリズム

  https://github.com/opencv/opencv/blob/master/modules/imgproc/src/drawing.cpp#L1490
  ここを参考に実装している。


2018.04現在の課題
-----------------
コード内のTODOをリストアップ
* param-test.cppにて、テストを型パラメータ化すること
* map_input-test.cppにて、期待値の比較精度の検討をすること
* map_converter.hppにて、クリップされる領域をobstacle_radiusも考慮して計算すること

