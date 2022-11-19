# Sim City Solver

スマホアプリの [SIMCITY BUILDIT](https://play.google.com/store/apps/details?id=com.ea.game.simcitymobile_row&hl=ja&gl=US&pli=1) を題材にしたジョブショップスケジューリング問題のソルバー

octaveによる実装に不向きであるため、開発停止中。

### 課題

* 「ｎ台の機械がｍ種類の仕事を必ず１度だけ処理する」という条件を満たさない。
* 上記のために、工程処理順序の制約の改良が必要となる。
* 上記のために、（とある仕事のとある機械）＝（とある工程）の関係が成り立たなくなるため、決定変数を機械ではなく工程にする必要がある。
* シムシティでは仕事毎に工程数が異なるため、行列表現に向かず、オブジェクト指向の方が良い。
* 倉庫の概念を導入する必要がある。参考文献より、（倉庫）＝（作業をしない特殊な機械）と定義すればよい。
* （実装済）工程順序が直列ではなくなるため、グラフ理論（隣接行列）を利用した工程処理順序表現が必要となる。


#### 参考文献

* [Numerical Optimizer SIMPLE例題集](https://www.msi.co.jp/nuopt/docs/v23/examples/html/01-00-00.html)
* [ジョブショップスケジューリング問題の数理表現](https://www.jstage.jst.go.jp/article/isciesci/61/1/61_14/_pdf/-char/ja)
* [バッファを考慮に入れたジョブショップスケジューリング](https://www.jstage.jst.go.jp/article/kikaic1979/71/702/71_702_685/_pdf)
* [隣接行列](https://ja.wikipedia.org/wiki/%E9%9A%A3%E6%8E%A5%E8%A1%8C%E5%88%97)
