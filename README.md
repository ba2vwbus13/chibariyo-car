# チバリヨカー — 人追従電動車椅子シミュレーション (ROS 2 Humble + Gazebo Fortress)

最終更新: 2026-08-20（arm64対応のDocker修正・屋内施設ワールド追加・Re-ID実験パック追加）

人を検出し、追従して走行する電動車椅子のシミュレーションです。
追従方式は **2D LiDAR** / **カメラ色検出** / **カメラYOLO人検出** の3種類を切り替えられます。
Mac上のDocker + ブラウザ(noVNC)で動作します。

このフォルダをClaude Coworkで開き「`HANDOVER.md` と `README.md` を読んで開発を引き継いでください」と
伝えれば、いつでも再開できます。**`HANDOVER.md` に現在の状態・未解決の課題・次にやることをまとめて
あります**(セッションをまたぐときは必ずそちらを先に読んでください)。

> Gazebo Classicはarm64(Apple Silicon)向けパッケージが提供されていないため、
> arm64ネイティブで動く新Gazebo(Fortress)+ ros_gz 構成にしています。

> **2026-08-20 追記(開発機の移行)**: 開発機を別のMac(M4)から現在のiMacへ移したところ、
> `ros-humble-ros-gz` の apt バイナリが arm64 には無いことが分かり、`docker compose build` が
> `E: Unable to locate package ros-humble-ros-gz` で失敗するようになりました。
> 現在の `docker/Dockerfile` は「aptにあれば apt / 無ければ ros_gz(humbleブランチ)をソースビルド」
> という分岐に変更してあります。ソースビルドは10〜20分かかり、**Docker Desktop に 8GB 以上の
> メモリ割り当てが必要**です(不足すると `cc1plus` がOOMで殺されます)。

## できること（デモ4種の早見表）

| launch | 追従方式 | 特徴 | 起動コマンド |
|---|---|---|---|
| `demo.launch.py` | **LiDAR**(既定) | `/scan` を人らしい幅でクラスタリング。軽くて安定 | `ros2 launch wheelchair_gazebo demo.launch.py` |
| `demo.launch.py method:=camera` | カメラ色検出 | HSVで特定色を追う。人型メッシュ導入後は色が不定のため**参考実装** | `... demo.launch.py method:=camera` |
| `demo.launch.py method:=yolo` | カメラYOLO | YOLOv8で人検出＋深度で距離。**実機に最も近い** | `... demo.launch.py method:=yolo` |
| `nav_demo.launch.py` | LiDAR + Nav2 | 障害物を回避しながら追従。経路計画つき | `... nav_demo.launch.py` |
| `dance_demo.launch.py` | — | 人の動きを1.2秒遅れで真似る「踊り物真似」デモ | `... dance_demo.launch.py` |

## ワールド(3種類・`world:=` で切り替え)

| ワールド | 内容 | 起動例 |
|---|---|---|
| `follow_test.world`(既定) | 12m×12mの部屋 + 障害物2個。アルゴリズムの素の挙動を見るのに向く | `ros2 launch wheelchair_gazebo demo.launch.py` |
| `facility.world` | 18m×12mの屋内施設。幅3mの中央廊下 + 4部屋(食堂・談話室・居室・リハビリ室) + 扉4か所(幅1.6m) + 家具20点あまり | `ros2 launch wheelchair_gazebo demo.launch.py world:=facility.world` |
| `crowd_test.world` | follow_test + 通行人 person2。Re-ID評価用(下の「Re-ID評価」の節を参照) | `ros2 launch wheelchair_gazebo crowd_demo.launch.py` |

`sim.launch.py` / `demo.launch.py` / `nav_demo.launch.py` のすべてが `world:=` に対応しています。

人の巡回ルートは `wheelchair_gazebo/config/person_waypoints_<ワールド名>.yaml` に分離してあり、
launchがワールド名から自動で読み込みます(対応するYAMLが無ければノードの既定値)。
`--symlink-install` でビルドしていれば、**YAMLを編集するだけで再ビルドなしに**ルートを変えられます。

車椅子のスポーン位置も引数で変えられます(人ノードの `robot_world_offset` も自動で追従します)。

```bash
ros2 launch wheelchair_gazebo demo.launch.py world:=facility.world spawn_x:=-6.0 spawn_y:=0.0
```

`facility.world` の間取り(x: -9〜9m, y: -6〜6m):

- 中央廊下 `y = -1.5 〜 1.5`(幅3m)が東西に貫通
- 北側の扉: `x = -4.5`(食堂) と `x = 4.5`(談話室) / 南側の扉: `x = -5.5`(居室) と `x = 3.0`(リハビリ室)
- 人の巡回ルートは全区間で最小クリアランス0.41m(人の半径0.15m)を確保

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

屋内施設ワールドで動かす場合は `world:=facility.world` を足します(詳細は「ワールド」の節)。

```bash
ros2 launch wheelchair_gazebo demo.launch.py world:=facility.world
ros2 launch wheelchair_gazebo nav_demo.launch.py world:=facility.world
```

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
| `docker compose build` が `E: Unable to locate package ros-humble-ros-gz` | arm64のaptには ros_gz のバイナリが無い。現在のDockerfileは自動でソースビルドに切り替わる(要10〜20分) |
| ビルド中に `c++: fatal error: Killed signal terminated program cc1plus` | メモリ不足。Docker Desktop → Settings → Resources → Memory を8GB以上に。Dockerfile側は `MAKEFLAGS=-j1` と `--executor sequential` で1並列に制限済み |
| `ros_gz_bridge` が見つからない | ソースビルド版は `/opt/ros_gz_ws/install` に入る。`ros2 pkg prefix ros_gz_bridge` で確認(bashrcで自動sourceしている) |

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

## 既知の問題: 人と車椅子のデッドロック(最優先課題)

`facility.world` で、人が部屋の奥まで入って引き返してくると、**人と車椅子が向かい合って両方とも永久に停止**します。

原因は「近づいたら止まる」ルールが両側にあることです。

| ノード | 該当パラメータ | 挙動 |
|---|---|---|
| `person_mover.py` | `robot_stop_radius` = 0.7 | 車椅子が0.7m以内にいると `Twist()` を送って停止 |
| `follower_node.py` | `stop_distance` = 0.8 | `cmd.linear.x = max(0.0, ...)` のため**後退できない**。0.8m以内では停止するだけ |

互いに相手が動くのを待つため、いったん噛み合うと復帰しません。

**Nav2版(`nav_demo.launch.py`)でも解決しません。** `nav_follower_node.py` が送るゴールは
「人の**手前** standoff m」なので、人が車椅子の方へ向かってくる状況ではゴールが車椅子の現在地付近になり、
車椅子は動かなくてよいという解になります。Nav2の障害物回避は「相手が静止している前提で自分が迂回する」
機能であって「道を譲る」機能ではなく、行き止まりの部屋や幅1.6mの扉では迂回路そのものが存在しません。

## 次にやること: 「譲る(yield)」ロジックの実装

実機でも必ず起きる問題なので、追従アルゴリズム側に明示的な譲り動作を入れます。優先順に3段構え。

### 1. 車椅子に退避動作を入れる(本命・`follower_node.py`)

- **状態機械を追加**: `FOLLOW`(通常追従) / `YIELD`(道を譲る) / `WAIT`(脇で待機)
- **YIELDへの遷移条件**: 人との距離が `yield_distance`(0.9m程度)以下 **かつ** 人が接近中
  (距離の時間微分が負、`d(dist)/dt < -0.05 m/s` を数サイクル連続で観測)
- **YIELD中の動作**: `cmd.linear.x` の下限0を外して後退を許可(`max_reverse` = 0.3 m/s 程度)。
  同時に `/scan` の左右の空き具合を比較し、空いている側へ寄る(角速度を与えながら後退)
- **復帰条件**: 人が離れていく(距離の微分が正)状態が1秒続いたら `FOLLOW` に戻る
- **安全策**: 後退中は背後の `/scan` 最短距離を監視し、0.4m以下なら停止(LiDARは360°なので背後も見える)
- 同じ処理を `camera_follower_node.py` / `yolo_follower_node.py` / `yolo_reid_follower_node.py` にも展開する
  (カメラ方式は背後が見えないので、後退の可否判断は `/scan` を併用する)

### 2. 人側も避けて歩く(`person_mover.py`)

- 現状の「止まる」を「よける」に変更。`robot_stop_radius` 以内に車椅子が入ったら、
  目標方向ベクトルに車椅子からの反発ベクトル(距離の逆数で重み付け)を足して進む
- 一定時間(3秒程度)動けなかったら次のウェイポイントへスキップし、膠着を強制的に解く

### 3. ワールド側で正面衝突の機会を減らす(`facility.world`)

- 各部屋に扉を2つ設けて一方通行のループ動線にする(行き止まりを作らない)
- 発表用の映像では有効だが本質的な解決ではないので、1. と併用する

### 検証の仕方

1. `world:=facility.world` で `demo.launch.py` を起動し、談話室(東側の部屋)に人が入って戻る場面を `./record.sh 90` で録画
2. Before(現状のデッドロック)と After(譲り動作あり)を並べると、発表資料にそのまま使える
3. 定量評価は `metrics_logger_node.py` の指標に「デッドロック発生回数」「1周にかかる時間」を足すと、
   Re-ID実験パックと同じ枠組みで数値比較できる

## 補足

- 人モデルの衝突形状を円柱にしているのは、Gazeboのアニメーション人型(actor)がLiDARに映らない(衝突形状を持たない)ためです。実機の脚検出(2本脚クラスタ)に近づけたい場合は、円柱を2本の細い円柱に分けると脚ペア検出アルゴリズムの開発に使えます
- 人の移動は、personモデルのVelocityControlプラグイン(速度指令)+OdometryPublisherプラグイン(位置フィードバック)を使った閉ループ制御です
- YOLO方式はCPU推論のため数Hz程度の検出周期になります(推論中のフレームはスキップ)。速度不足を感じたら `imgsz` を小さくするか、LiDAR方式を使ってください
- 初回起動時は人型メッシュ(Fuelから約26MB)とYOLOモデル(約6MB)の自動ダウンロードがあるため、ネット接続が必要です
- Dockerfile変更後は `docker compose build --no-cache && docker compose up -d`、コード変更後はコンテナ内で `colcon build` の再実行が必要です
- 次にやることは「譲る(yield)ロジックの実装」の節を参照。その先の候補: SLAM(slam_toolbox)導入によるmap座標系でのNav2運用、実機LiDAR(URG/RPLiDAR)・実機カメラへの置き換え


## Re-ID評価(人混みシナリオ)実験パック — 2026-08-20追加

SCORE!応募書類の「今後の課題」に対応する、**通行人がいる環境でのRe-ID追尾と定量評価**の
実験セットです(別セッションのClaudeが追加。既存コードは setup.py のエントリポイント追加以外
変更していません)。

### 追加されたもの

| ファイル | 役割 |
|---|---|
| `worlds/crowd_test.world` | follow_test + 通行人 person2(胸に赤い帯=服色ちがい) |
| `launch/crowd_demo.launch.py` | 人混みデモ一括起動 + 計測(method:=reid/yolo/lidar) |
| `wheelchair_follower/yolo_reid_follower_node.py` | YOLO+色ヒストグラムRe-ID追従(状態を/follower/statusに配信) |
| `wheelchair_follower/metrics_logger_node.py` | 真値ベースの定量計測(ID維持率・見失い率・再発見時間・距離誤差) |

### 実行手順

```bash
cd ~/ros2_ws
colcon build --symlink-install   # エントリポイント追加のため再ビルド必須
source install/setup.bash
ros2 launch wheelchair_gazebo crowd_demo.launch.py                # Re-ID方式
# 60〜120秒走らせて Ctrl+C → ~/ros2_ws/metrics/summary_reid_*.txt に結果

ros2 launch wheelchair_gazebo crowd_demo.launch.py method:=yolo   # 比較: 最近傍YOLO
ros2 launch wheelchair_gazebo crowd_demo.launch.py method:=lidar  # 比較: LiDAR
```

### 測れる指標(summary_*.txt)

- **ID維持率**: 追従中に正しく本人を追っていた時間割合
- **ID取り違え回数**: 通行人に乗り移った回数(最近傍方式はここが弱いはず)
- **見失い率 / 平均再発見時間**: 横切られた後の復帰性能
- **距離維持誤差**: 本人追従中の |実距離−目標1.2m| の平均±標準偏差

比較のポイント: `method:=yolo`(最も近い人を追う)は通行人が横切ると乗り移りやすく、
`method:=reid` は服の色と位置の連続性で本人に留まる——という差が数値で出れば、
書類・発表の「センサフュージョンの効果」の根拠になります。

### 注意

- `/follower/status` を配信するのはreid方式のみ。yolo/lidar方式の比較測定では
  ID判定ができないため、CSVの距離データと見た目(動画)での比較になります。
  厳密に比較したい場合は各followerに同様のstatus配信を足してください。
- 通行人と本人は物理的に接触することがあります(両方とも高慣性の円柱なので実害は軽微)。
- 色Re-IDは簡易実装(HSVヒストグラム)です。実機では照明変化に弱いため、
  発表では「シミュレーションでの原理検証」と位置づけるのが正直で安全です。

## GitHubへのアップロード

初回は `github_setup.command` を1度だけ実行します。

### 次回以降、変更をアップロードする手順

```bash
cd "/Users/nakahira/Library/CloudStorage/GoogleDrive-ba2vwbus13wind@gmail.com/My Drive/研究/20260716チバリヨカー"
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
