# チバリヨカー — 人追従電動車椅子シミュレーション (ROS 2 Humble + Gazebo Fortress)

最終更新: 2026-08-12（フォルダ整理にあわせて加筆）

人を検出し、追従して走行する電動車椅子のシミュレーションです。
追従方式は **2D LiDAR** / **カメラ色検出** / **カメラYOLO人検出** の3種類を切り替えられます。
Mac上のDocker + ブラウザ(noVNC)で動作します。

このフォルダをClaude Coworkで開き「`README.md` を読んで開発を引き継いでください」と
伝えれば、いつでも再開できます。

> Gazebo Classicはarm64(Apple Silicon)向けパッケージが提供されていないため、
> arm64ネイティブで動く新Gazebo(Fortress)+ ros_gz 構成にしています。

## できること（デモ4種の早見表）

| launch | 追従方式 | 特徴 | 起動コマンド |
|---|---|---|---|
| `demo.launch.py` | **LiDAR**(既定) | `/scan` を人らしい幅でクラスタリング。軽くて安定 | `ros2 launch wheelchair_gazebo demo.launch.py` |
| `demo.launch.py method:=camera` | カメラ色検出 | HSVで特定色を追う。人型メッシュ導入後は色が不定のため**参考実装** | `... demo.launch.py method:=camera` |
| `demo.launch.py method:=yolo` | カメラYOLO | YOLOv8で人検出＋深度で距離。**実機に最も近い** | `... demo.launch.py method:=yolo` |
| `nav_demo.launch.py` | LiDAR + Nav2 | 障害物を回避しながら追従。経路計画つき | `... nav_demo.launch.py` |
| `dance_demo.launch.py` | — | 人の動きを1.2秒遅れで真似る「踊り物真似」デモ | `... dance_demo.launch.py` |

## 構成

```
20270716チバリヨカー/
├── README.md                  # 本ファイル
├── docker-compose.yml         # Docker設定(noVNC付きROS 2環境)
├── docker/Dockerfile
└── ros2_ws/                   # ROS 2ワークスペース(コンテナ内の ~/ros2_ws と共有)
    ├── src/                   # ★ ソースコード本体。編集するのはここだけ
    │   ├── wheelchair_description/  # 車椅子URDF(差動二輪 + 2D LiDAR + RGB-Dカメラ)
    │   ├── wheelchair_gazebo/       # Gazeboワールド・launch(demo/sim/nav/dance)
    │   ├── wheelchair_follower/     # 追従制御ノード + 人移動ノード
    │   └── wheelchair_nav/          # Nav2設定(障害物回避付き追従)
    ├── record.sh              # デスクトップ画面の録画スクリプト
    ├── videos/                # record.sh の録画結果(mp4・8本)
    ├── yolov8n.pt             # YOLOモデル(初回起動時に自動DLされたもの)
    ├── build/  install/  log/ # colcon build の生成物。消しても再ビルドで復活する
    └── (押入れなし)
```

### `src/` の中の主なノード

| ファイル | 役割 |
|---|---|
| `wheelchair_follower/follower_node.py` | LiDAR方式の追従制御 |
| `wheelchair_follower/camera_follower_node.py` | カメラ色検出方式の追従制御 |
| `wheelchair_follower/yolo_follower_node.py` | YOLO方式の追従制御 |
| `wheelchair_follower/nav_follower_node.py` | Nav2へゴールを送る追従制御 |
| `wheelchair_follower/person_mover.py` | 人モデルをウェイポイントに沿って歩かせる |
| `wheelchair_follower/person_dancer_node.py` | 人モデルに振り付けをさせる（`CHOREOGRAPHY` を編集） |
| `wheelchair_follower/dance_mimic_node.py` | 人の動きを遅延再生して真似る |

> `build/` `install/` `log/` はビルドのたびに作り直される一時ファイルです。
> 動きがおかしくなったら、この3つを削除して `colcon build --symlink-install` を
> やり直すと直ることがあります（GitHubには上げない設定にしてあります）。

### 動作の仕組み

1. **wheelchair (車椅子)**: 差動二輪 + 360° 2D LiDAR + 前方RGB-Dカメラ。`/scan`・`/camera/image_raw`・`/camera/depth/image_raw` を出力し `/cmd_vel` で走行
2. **person (人)**: 人型メッシュモデル(Standing person、初回起動時にネットから自動DL。LiDAR用の衝突形状は円柱)。`person_mover` ノードがウェイポイントに沿って移動(歩行速度 0.4 m/s)
3. **ros_gz_bridge**: GazeboとROS 2のトピックを橋渡し(/scan, /camera/*, /cmd_vel, /odom, /clock など。sim.launch.pyが自動起動)
4. 追従ノード(どちらか一方を起動):
   - **follower (LiDAR方式)**: `/scan` をクラスタリング → 人らしい幅(0.05〜0.7m)のクラスタを検出 → 前回位置に近いものを追跡
   - **camera_follower (色検出方式)**: RGB画像をHSV変換し特定色をマスク抽出 → 最大輪郭の重心の画素位置から方位角、深度画像から距離を取得(※人型メッシュ導入後は色が不定のため参考実装。hsv_lower/upperの調整が必要)
   - **yolo_follower (YOLO方式)**: YOLOv8で人を検出 → 最も近い人のバウンディングボックス中心から方位角、深度画像から距離を取得。実機に最も近い構成
5. どちらもP制御で約1.2mの距離を保ち追従。見失うと1秒後に停止

## 起動手順

### 1. コンテナ起動(初回はビルドに数分)

```bash
cd このフォルダ
docker compose up -d --build
```

ブラウザで **http://localhost:6080** を開くとLinuxデスクトップが表示されます。

### 2. ワークスペースのビルド(デスクトップ内のターミナルで)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. シミュレーション実行

全部入り(Gazebo + 車椅子 + 人移動 + 追従):

```bash
ros2 launch wheelchair_gazebo demo.launch.py                  # LiDAR追従(既定)
ros2 launch wheelchair_gazebo demo.launch.py method:=camera   # カメラ色検出追従
ros2 launch wheelchair_gazebo demo.launch.py method:=yolo     # YOLO人検出追従
ros2 launch wheelchair_gazebo nav_demo.launch.py              # Nav2追従(障害物回避付き)
ros2 launch wheelchair_gazebo dance_demo.launch.py           # 踊り物真似(人の動きを真似る)
```

Gazeboが開き、8秒後に人型モデルが歩き出し、車椅子が追従を始めます。

個別に起動する場合:

```bash
# ターミナル1: Gazebo + 車椅子
ros2 launch wheelchair_gazebo sim.launch.py
# ターミナル2: 人の移動
ros2 run wheelchair_follower person_mover
# ターミナル3: 追従ノード(いずれか1つ)
ros2 run wheelchair_follower follower           # LiDAR方式
ros2 run wheelchair_follower camera_follower    # カメラ色検出方式
ros2 run wheelchair_follower yolo_follower      # YOLO方式
```

※ 複数方式を同時に起動しないでください(`/cmd_vel` が競合します)。

## 調整できるパラメータ

follower ノード(`ros2 run wheelchair_follower follower --ros-args -p 名前:=値`):

| パラメータ | 既定値 | 意味 |
|---|---|---|
| target_distance | 1.2 | 保つ距離 [m] |
| stop_distance | 0.8 | これ以下で前進停止 [m] |
| max_linear | 0.7 | 最大速度 [m/s] |
| max_angular | 1.2 | 最大旋回 [rad/s] |
| lost_timeout | 1.0 | 見失い→停止までの秒数 |

camera_follower ノード(上記の制御パラメータに加えて):

| パラメータ | 既定値 | 意味 |
|---|---|---|
| hsv_lower / hsv_upper | [5,100,60] / [25,255,255] | 検出色のHSV範囲(オレンジ) |
| min_area | 300 | 最小検出面積 [px] |
| hfov | 1.36 | カメラ水平FOV [rad] |

yolo_follower ノード(上記の制御パラメータに加えて):

| パラメータ | 既定値 | 意味 |
|---|---|---|
| model | yolov8n.pt | YOLOモデル(初回に自動DL、大きいモデルに変更可) |
| confidence | 0.4 | 検出信頼度しきい値 |
| imgsz | 320 | 推論解像度(大きいほど高精度・低速) |

person_mover ノード: `speed`(歩行速度)、`waypoints`(巡回経路 [x1,y1,x2,y2,...])

## 踊り物真似モード

`dance_demo.launch.py` は、人がその場で踊り、車椅子が少し遅れて同じ動きを真似る「コール&レスポンス」デモです。

### 仕組み

1. **person_dancer ノード**: 人モデルに振り付け(その場スピン → 前後の揺れ → 左右のウィグル → 決めの高速スピン)を `/person/cmd_vel` で出力。振り付けは `(継続時間, 前後速度, 旋回速度)` のリストで定義
2. **dance_mimic ノード**: 人の動き(`/person/odom` のツイスト = 前後速度・旋回速度)を遅延バッファに貯め、`delay`(既定1.2秒)前の動きを自分の `/cmd_vel` として再生

差動二輪の車椅子は横移動できないため、真似るのは前後(linear.x)と旋回(angular.z)の2成分です。振り付けもこの2成分だけで構成しています。

### パラメータ

- person_dancer: `loop`(振り付けの繰り返し、既定true)、`speed_scale`(全体の速さ倍率)
- dance_mimic: `delay`(真似るまでの遅れ秒数)、`linear_scale` / `angular_scale`(動作の倍率)

振り付け自体を変えたい場合は `person_dancer_node.py` の `CHOREOGRAPHY` を編集してください。

## Nav2統合(障害物回避しながら追従)

### 仕組み

`nav_demo.launch.py` は他方式と構成が異なり、車椅子の走行をNav2(ROS 2標準のナビゲーションスタック)に任せます。

1. **nav_follower ノード**: LiDARで人を検出し、TFでodom座標系に変換。「人の手前1.0m」をゴールとしてNav2のNavigateToPoseアクションに送信。人が0.4m以上動いたらゴールを更新(古いゴールは自動置換)
2. **Nav2**: グローバル経路計画(NavFn) + ローカル制御(DWB) + LiDARベースのコストマップで、壁や障害物(worldに茶色の箱を2つ配置済み)を回避しながらゴールへ走行

地図(SLAM)は使わず、odom座標系のローリングウィンドウ・コストマップで動作するシンプル構成です。

### 実行

```bash
# Dockerイメージ再ビルド済みなら不要。既存コンテナに後から入れる場合:
sudo apt-get update && sudo apt-get install -y ros-humble-navigation2

cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch wheelchair_gazebo nav_demo.launch.py
```

起動から約16秒後(Nav2のライフサイクル起動完了後)に追従が始まります。
人と車椅子の間に障害物が入る位置関係になると、迂回して追いかける様子が見られます。

### 可視化・パラメータ

- RViz2でコストマップと経路を確認: `rviz2` を起動し、Fixed Frame を `odom` に、Map(トピック `/global_costmap/costmap`)、Path(`/plan`)、LaserScan(`/scan`)を追加
- nav_follower の主なパラメータ: `standoff`(人の手前で保つ距離、既定1.0m)、`goal_update_dist`(ゴール更新のしきい値、既定0.4m)
- 速度上限・ロボット半径・インフレーション半径などは `wheelchair_nav/config/nav2_params.yaml` で調整

### 制限事項

- 人自身もコストマップ上は障害物になるため、`standoff` をインフレーション半径(0.55m)より十分大きくしておく必要があります
- nav_followerの人検出はLiDARクラスタ方式です。YOLO検出をNav2と組み合わせたい場合は、nav_follower_node.pyの`detect_person`をyolo_follower_node.pyの検出部に差し替えれば実現できます

## YOLOによる人検出・追尾の詳細

### 処理の流れ

1. `/camera/image_raw`(RGB画像)を受信するたびに、YOLOv8で物体検出を実行(COCOの `person` クラスのみ、信頼度しきい値 `confidence` 以上)
2. 検出された各人物のバウンディングボックスについて、`/camera/depth/image_raw`(深度画像)のボックス中央1/3領域の中央値からカメラまでの距離を計算
3. **最も距離が近い人物**を追尾対象に選択
4. ボックス中心の横位置と水平FOVから方位角を計算し、P制御で「距離1.2mを保ちつつ正面に捉える」ように `/cmd_vel` を出力
5. `lost_timeout`(既定1.5秒)の間検出がなければ停止

CPU推論のため検出は数Hz程度です。推論中に届いたフレームはスキップして遅延の蓄積を防いでいます。

### セットアップ

Dockerイメージを再ビルド済みなら追加作業は不要です。既存のコンテナに後から入れる場合:

```bash
pip3 install ultralytics "numpy<2"       # YOLO本体(numpy<2はapt版OpenCVとの衝突回避)
pip3 install "setuptools==58.2.0"        # colcon --symlink-install互換のため
```

初回実行時に以下が自動ダウンロードされます(要ネット接続):

- YOLOv8nモデル `yolov8n.pt`(約6MB) — yolo_follower初回起動時
- 人型メッシュ Standing person(約26MB) — Gazebo初回起動時

### シミュレーション上の工夫

COCO学習済みYOLOは円柱を人として認識しないため、Gazebo内の人はOpen Robotics公式の人型メッシュ(Standing person)で表示しています。LiDAR用の衝突形状は円柱のままなので、LiDAR方式・YOLO方式とも同じ人モデルで動作します。

### 精度・速度の調整

- 検出が遅い → `imgsz` を小さく(例: 224)。または `confidence` を下げると見失いにくくなります
- 精度を上げたい → `model:=yolov8s.pt` など大きいモデルに変更(初回に自動DL)、`imgsz:=640`
- 例: `ros2 run wheelchair_follower yolo_follower --ros-args -p imgsz:=224 -p confidence:=0.3`

### トラブルシューティング

| 症状 | 対処 |
|---|---|
| `import cv2` で `numpy.core.multiarray failed to import` | `pip3 install "numpy<2"` |
| `colcon build --symlink-install` で `option --editable not recognized` | `pip3 install "setuptools==58.2.0"` して build/ install/ を削除後に再ビルド |
| pipが「torch requires setuptools>=77」と警告 | 実害なし(torchの実行にはsetuptools不要)。無視してよい |
| 検出されない | `rqt_image_view` で `/camera/image_raw` に人が写っているか確認。写っていなければカメラの向き・人の位置の問題 |

## 動画の録画

シミュレーション実行中に、コンテナ内の**別ターミナル**で:

```bash
cd ~/ros2_ws
./record.sh 30        # 30秒間録画(秒数は変更可)
```

デスクトップ画面全体がmp4で `ros2_ws/videos/` に保存されます。
このフォルダはMac側と共有されているので、Macの Finder からそのまま開けます。
録画前にGazeboのウィンドウを大きくし、視点を見やすい位置にしておくときれいに撮れます。

現在 `ros2_ws/videos/` には過去の実行を録画したmp4が8本入っています
（ファイル名の数字は `wheelchair_YYYYMMDD_HHMMSS.mp4` の撮影日時）。
容量が大きいためGitHubには上げない設定にしてあります。

その他の方法: GazeboのGUI右上メニューからVideo Recorderプラグインを追加すると3Dビューだけを録画できます。手軽さ重視ならMacの画面収録(Cmd+Shift+5)でブラウザごと録画しても構いません。

## デバッグ

```bash
ros2 topic echo /scan --once                              # LiDARデータ確認
ros2 topic echo /cmd_vel                                  # 制御出力確認
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw   # カメラ映像確認
rviz2                            # 可視化(Fixed Frame: odom, LaserScan/Image追加)
ign topic -l                     # Gazebo側のトピック一覧
ign topic -e -t /model/person/odometry   # 人の位置確認(Gazebo側)
```

## 補足

- 人モデルの衝突形状を円柱にしているのは、Gazeboのアニメーション人型(actor)がLiDARに映らない(衝突形状を持たない)ためです。実機の脚検出(2本脚クラスタ)に近づけたい場合は、円柱を2本の細い円柱に分けると脚ペア検出アルゴリズムの開発に使えます
- 人の移動は、personモデルのVelocityControlプラグイン(速度指令)+OdometryPublisherプラグイン(位置フィードバック)を使った閉ループ制御です
- YOLO方式はCPU推論のため数Hz程度の検出周期になります(推論中のフレームはスキップ)。速度不足を感じたら `imgsz` を小さくするか、LiDAR方式を使ってください
- 初回起動時は人型メッシュ(Fuelから約26MB)とYOLOモデル(約6MB)の自動ダウンロードがあるため、ネット接続が必要です
- Dockerfile変更後は `docker compose build --no-cache && docker compose up -d`、コード変更後はコンテナ内で `colcon build` の再実行が必要です
- 次のステップ候補: 特定人物の追尾(見た目の特徴やIDによるRe-ID)、SLAM(slam_toolbox)導入によるmap座標系でのNav2運用、実機LiDAR(URG/RPLiDAR)・実機カメラへの置き換え

## GitHubへのアップロード

初回は `github_setup.command` を1度だけ実行します。

### 次回以降、変更をアップロードする手順

```bash
cd "/Users/nakahira/Documents/latest/研究/20270716チバリヨカー"
git status              # 何が変わったか、何が上がるかを必ず先に確認
git add -A
git commit -m "変更内容のメモ"
git push
```

### 何が上がって、何が上がらないか

`.gitignore` で次を除外しています。**上がるのは `src/` のソースとDocker設定だけ**です。

| 除外するもの | 理由 |
|---|---|
| `ros2_ws/build/` `install/` `log/` | `colcon build` でいつでも再生成できる一時ファイル（ログ188個を含む） |
| `ros2_ws/videos/` | 録画mp4（約6MB）。成果物だが容量が大きい |
| `ros2_ws/yolov8n.pt` | 初回起動時に自動ダウンロードされる（約6MB） |
| `__pycache__/` `*.pyc` | Pythonの自動生成キャッシュ |

そのためGitHubからクローンした場合は、`docker compose up -d --build` →
コンテナ内で `colcon build --symlink-install` が必要です（YOLOモデルと人型メッシュは
初回起動時に自動でダウンロードされます）。

### 注意点

- **`github_setup.command` を2回目以降に実行しないこと。**
  このスクリプトは `.git` を作り直すため、**これまでのコミット履歴が消えます**。
  初回セットアップ専用です。2回目以降は上の3コマンドを使ってください。

- **`git status` で日本語ファイル名が `\350\252\254...` と表示される場合。**
  文字化けではなく、gitがASCII以外のパスを8進数エスケープで表示する既定設定です。

  ```bash
  git config --global core.quotepath false
  ```

- **`Permission denied (publickey)` が出たら。** remoteがSSH URLになっています。

  ```bash
  gh auth setup-git
  git remote set-url origin https://github.com/ba2vwbus13/chibariyo-car.git
  ```
