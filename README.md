# ことばで描くロボット
Physical AI 2026 応用編1 最終課題 / OmniCampus: muro-ssm

日本語の短い指示を分類AIで解釈し、ROS 2 と Gazeboの差動二輪ロボットで実行します。「ロボットを描いて」で連続した走行軌跡が顔になります。一部の線をなぞり直します。

## 環境と起動
動作確認環境: Windows / WSL2・WSLg / Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / Python 3。
ROS 2 Jazzyは[公式手順](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)で導入。ROSのパッケージ取得先設定後、Ubuntuで必要なパッケージを入れます。

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz python3-tk fonts-noto-cjk
```

本リポジトリをダウンロード・展開し、Ubuntu端末で展開先フォルダーへ移動して実行します。

```bash
bash start.sh
```

Gazeboと操作画面が開きます。外部AI API・APIキー・追加のPythonライブラリは不要。Linux側で日本語を入力できない場合は例文メニューを選ぶか、Windowsで入力した文章を貼り付けます。

## 操作
- 「前に進んで」「バックして」: 0.5 m移動。
- 「右を向いて」「左を向いて」: 90度旋回。
- 「360度回転」: 左回り1周。
- 「右斜め前へ」「左斜め前へ」: 45度旋回して0.5 m前進。
- 「正方形を描いて」: 1辺0.5 mの四角。
- 「ロボットの顔を描いて」: 顔の経路を走行し出発位置に戻る。最初の向きには戻さない。
- 停止ボタン / Escape: 実行中の動作を停止。
- 「絵を消す」: 表示された軌跡を消去。

## AIと制御
文字1・2-gramを特徴とする多項ナイーブベイズを124例、11クラス（10動作と拒否）で起動時に学習します。分類スコア0.85未満や未知表現の多い入力は拒否します。否定や複合指示の一部は追加のルールで拒否します。スコアは校正された正解確率ではありません。
AIが任意の絵の経路を生成する機能はありません。顔・四角の経路はあらかじめ定義しています。
Gazeboの車体位置・姿勢をフィードバックし、曲がり角や短い線で減速して線ぶれを抑えます。青線は実際の車体位置から記録した軌跡です。目標線は表示しません。

## 実測結果
2026-10-28、顔の描画を実環境で確認しました。

| 実行 | 実時間 | 走行距離 | 出発位置との差 |
|---|---:|---:|---:|
| 確認1 | 122.6秒 | 4.79 m | 0.9 cm |
| 確認2・動画の動作結果 | 170.6秒 | 4.80 m | 0.9 cm |

編集済み提出動画は2分55秒。時間はPC負荷・シミュレーション進行率で変動します。3分以内は常時保証しません。2回の記録は統計的な精度評価ではありません。

## 検証
ROSを起動せずに分類・制御の回帰テストを実行できます。
```bash
python3 -m unittest test_motion.py
```
実機向けではなく、障害物検知・衝突回避はありません。シミュレーターの真の位置を利用しており、実機で使う場合は自己位置推定が必要です。

## ファイル
- intent.py / training.json: 日本語分類AIと学習例文
- motion.py / face.py / sequence.py: 移動・経路制御
- drawing.py: 実軌跡と距離・時間の記録
- app.py: 操作画面とROS通信
- world.sdf: 自作車体と環境
- bridge.yaml / demo.launch.py / start.sh: 通信と起動
- test_motion.py: 回帰テスト

## 制作・参考
生成AIの支援を受けて実装・修正し、WSL上のGazeboで動作確認しました。
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Gazebo HarmonicとROS](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Gazeboの差動二輪制御](https://gazebosim.org/docs/harmonic/moving_robot/)
