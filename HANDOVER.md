# 引き継ぎマニュアル (2026-08-20 時点)

別のセッション・別の担当者が作業を引き継ぐための申し送りです。
新しいセッションでは **「`HANDOVER.md` と `README.md` を読んで開発を引き継いでください」** と伝えてください。

- プロジェクト全体の説明・使い方 → `README.md`
- いま何が起きていて、次に何をすべきか → **このファイル**

---

## 1. 現在の状態(ひとことで)

- 開発機を **別のMac(M4)から現在のiMacへ移行**した。移行に伴う Docker ビルド失敗は**解決済み**で、
  シミュレーションは動く状態にある
- 屋内施設ワールド `facility.world` を追加し、`world:=` で切り替えられるようにした
- **未解決の最優先課題**: 人と車椅子が向かい合うと双方が停止して復帰しない**デッドロック**
- 並行して、別セッションが **Re-ID評価(人混みシナリオ)実験パック**を追加している(`README.md` の該当節)

## 2. この移行で判明した環境の事実(重要・再発しやすい)

| 事実 | 根拠 | 対処 |
|---|---|---|
| `ros-humble-ros-gz` の apt バイナリは **arm64 に存在しない** | 現iMac(arm64ネイティブ、`ports.ubuntu.com` を参照)で `E: Unable to locate package ros-humble-ros-gz`。同じ apt 行の他のROSパッケージは解決している。個別名 `ros-humble-ros-gz-sim` / 旧名 `ros-humble-ros-ign` も無し(Dockerfileの3段フォールバックが全て素通りしてソースビルドに入った) | `docker/Dockerfile` が humbleブランチの ros_gz を自動でソースビルドする。所要10〜20分 |
| ソースビルド版 ros_gz は `/opt/ros_gz_ws/install` に入る | Dockerfile の `--merge-install` | `/etc/bash.bashrc` で自動 source 済み。確認は `ros2 pkg prefix ros_gz_bridge` |
| `ros_gz_bridge` のコンパイルはメモリを食う | `c++: fatal error: Killed signal terminated program cc1plus`(OOM) | Docker Desktop のメモリ割り当てを **8GB以上**に。Dockerfile側は `MAKEFLAGS=-j1` + `--executor sequential` で1並列に制限済み |
| M4機では同じDockerfileが通っていた | 本人談 | そのビルドは **amd64(Rosetta)で走っていた可能性が高い**。旧機が使えるなら `docker image inspect wheelchair-follow:humble --format '{{.Architecture}}'` で確認できる。未確認 |

> 補足: `set -eux` の `-u` を付けると `source /opt/ros/humble/setup.bash` が
> `AMENT_TRACE_SETUP_FILES: unbound variable` で落ちる。Dockerfile内でROSをsourceするときは `-u` を使わないこと。

## 3. 起動のしかた(最短経路)

```bash
# Mac側
cd "/Users/nakahira/Library/CloudStorage/GoogleDrive-ba2vwbus13wind@gmail.com/My Drive/研究/20260716チバリヨカー"
docker compose up -d          # イメージがあれば --build は不要
# ブラウザで http://localhost:6080

# コンテナ内(noVNCデスクトップのターミナル)
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch wheelchair_gazebo demo.launch.py world:=facility.world     # 屋内施設で追従
ros2 launch wheelchair_gazebo nav_demo.launch.py world:=facility.world # Nav2版
ros2 launch wheelchair_gazebo crowd_demo.launch.py                     # Re-ID評価(別セッション追加分)
./record.sh 90                # 別ターミナルで画面録画(videos/ に保存)
```

## 4. 今回の変更ファイル

| ファイル | 変更 |
|---|---|
| `docker/Dockerfile` | ros_gz を「aptにあれば apt / 無ければソースビルド」に変更。1並列コンパイル |
| `ros2_ws/src/wheelchair_gazebo/worlds/facility.world` | 新規。18m×12m、幅3mの中央廊下 + 4部屋(食堂・談話室・居室・リハビリ室) + 扉4か所(幅1.6m) + 家具 |
| `.../config/person_waypoints_facility.yaml` | 新規。施設ワールド用の巡回ルート |
| `.../config/person_waypoints_follow_test.yaml` | 新規。従来ワールド用(内容は `person_mover.py` の既定値と同じ) |
| `.../launch/sim,demo,nav_demo.launch.py` | `world:=` `spawn_x:=` `spawn_y:=` に対応。ワールド名から巡回ルートYAMLを自動選択 |
| `.../CMakeLists.txt` | `config` をインストール対象に追加 |
| `README.md` / `.gitignore` | 上記の記載、`metrics/` と `build.log` を除外に追加 |

`facility.world` の座標系: x = -9〜9m, y = -6〜6m。廊下は y = -1.5〜1.5。
扉は北側 x = -4.5(食堂)・x = 4.5(談話室)、南側 x = -5.5(居室)・x = 3.0(リハビリ室)。
巡回ルートは全区間で最小クリアランス 0.41m(人の半径0.15m)を座標計算で確認済み。

## 5. 未解決の最優先課題: デッドロック

`facility.world` で人が部屋の奥まで入って引き返してくると、人と車椅子が向かい合って**双方とも永久停止**する。

| ノード | パラメータ | 挙動 |
|---|---|---|
| `person_mover.py` | `robot_stop_radius` = 0.7 | 車椅子が0.7m以内なら `Twist()` を送って停止 |
| `follower_node.py` | `stop_distance` = 0.8 | `cmd.linear.x = max(0.0, ...)` のため**後退できない**。0.8m以内では停止のみ |

互いに相手が動くのを待つため復帰しない。

**Nav2版でも解決しない。** `nav_follower_node.py` のゴールは「人の**手前** standoff m」なので、
人が車椅子に向かってくる状況ではゴールが車椅子の現在地付近になり、車椅子は動かなくてよい解になる。
Nav2の障害物回避は「相手が静止している前提で自分が迂回する」機能であって「道を譲る」機能ではない。
加えて、行き止まりの部屋や幅1.6mの扉では迂回路そのものが存在しない。

## 6. 次にやること: 「譲る(yield)」ロジック

### 6.1 車椅子に退避動作(本命・`follower_node.py`)

状態機械 `FOLLOW` / `YIELD` / `WAIT` を追加する。

```
FOLLOW → YIELD  : 人との距離 < yield_distance(0.9m) かつ 接近中
                  (d(dist)/dt < -0.05 m/s が3サイクル連続)
YIELD           : linear.x の下限0を外して後退を許可(max_reverse = 0.3 m/s)
                  /scan の左右の空き具合を比較し、空いている側へ angular.z を与える
                  背後(±30°)の /scan 最短距離 < 0.4m なら停止 → WAIT
YIELD → FOLLOW  : 人が離れていく(d(dist)/dt > 0)状態が1.0秒継続
WAIT            : 後退できない場合の待機。人が離れたら FOLLOW に戻る
```

追加パラメータ案: `yield_distance`(0.9)、`max_reverse`(0.3)、`yield_release_time`(1.0)、`rear_clearance_min`(0.4)。
同じ処理を `camera_follower_node.py` / `yolo_follower_node.py` / `yolo_reid_follower_node.py` へ展開する
(カメラ方式は背後が見えないので、後退の可否判断は `/scan` を併用)。

### 6.2 人側も避けて歩く(`person_mover.py`)

- 「止まる」を「よける」に変更。`robot_stop_radius` 以内に車椅子が入ったら、
  目標方向ベクトルに車椅子からの反発ベクトル(距離の逆数で重み付け)を加算して進む
- 3秒間ほぼ動けなかったら次のウェイポイントへスキップし、膠着を強制解除する

### 6.3 ワールド側の緩和策(`facility.world`)

- 各部屋に扉を2つ設けて一方通行のループ動線にする(行き止まりを作らない)
- 映像用には有効だが本質的な解決ではないので、6.1 と併用する前提

### 6.4 受け入れ基準

- `world:=facility.world` で10分間連続運転してデッドロック0回
- 人との最短距離が常に 0.35m 以上(接触しない)
- 通常追従時の距離維持誤差が現状(目標1.2m)から悪化しない

## 7. 検証と記録

1. Before(現状のデッドロック)を `./record.sh 90` で録画しておく。談話室(東側の部屋)に人が入って戻る場面で発生する
2. After(譲り動作あり)を同じ画角・同じルートで録画し、並べて比較する
3. 定量評価は `metrics_logger_node.py`(Re-ID実験パック)の枠組みに
   「デッドロック発生回数」「1周の所要時間」「人との最短距離の最小値」を足すと、同じ形式で比較できる

## 8. Re-ID実験パックとの関係

別セッションが `crowd_test.world` / `crowd_demo.launch.py` / `yolo_reid_follower_node.py` /
`metrics_logger_node.py` を追加済み(詳細は `README.md` の「Re-ID評価」節)。
`crowd_demo.launch.py` は本セッションで追加した `sim.launch.py` の `world:=` 引数を使っているので、
両者は同じ土台に乗っている。**片方を書き換えるときは以下の共有点に注意**:

- `sim.launch.py` の launch 引数(`world` / `spawn_x` / `spawn_y`)のインターフェース
- `person_mover` のパラメータ名(`robot_world_offset` / `waypoints` / `robot_stop_radius`)
- `/cmd_vel` は1ノードのみが publish すること(追従ノードを2つ同時に起動しない)

譲るロジック(6章)は Re-ID方式にも必要なので、**Re-ID側の作業と並行して入れる場合は
`follower_node.py` を先に完成させ、そこから他ノードへ移植する**のが手戻りが少ない。

## 9. 注意事項

- `github_setup.command` は**初回専用**。2回目以降に実行すると `.git` を作り直してコミット履歴が消える
- プロジェクトはGoogle Drive上にある。Drive未ダウンロード(クラウドのみ)のファイルは
  外部ツールから読めないことがある。その場合はFinderで一度開いてローカルに落とす
- Dockerfile 変更後は `docker compose build`、コード変更後はコンテナ内で `colcon build --symlink-install`
- `--symlink-install` でビルドしていれば、`config/*.yaml` の巡回ルート変更は**再ビルド不要**
