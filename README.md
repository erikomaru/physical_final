# Single O → Rydges: ROS 2 模倣学習デモ

ROS 2 Humble + Ignition Fortress のUbuntu Docker上で、OSMを基にした6棟の街区を4輪ロボットで走行します。矢印キー1回ごとに約0.6 m移動または約29度旋回する固定ステップ操作でデモを記録し、Pai TrainingのLeRobot ACTで学習します。キー別押下数は端末に表示し、`/model/rover/key_counts` に累積値を記録します。

## 1. MacからDockerを起動

Docker Desktopを起動し、Macのターミナルでプロジェクトのフォルダから実行します。

```bash
bash scripts/start_container.sh
```

初回はDocker Hubからイメージを取得します。Apple Siliconではamd64イメージをDocker Desktopがエミュレーションします。イメージの取得に時間がかかることがあります。起動ログに表示されたURL（通常は [http://localhost:6080](http://localhost:6080)）を開き、VNCパスワード `ubuntu` でデスクトップに接続してください。6080番が使用中ならスクリプトが近くの空きポートを選びます。ポートを指定する場合は `NOVNC_PORT=6081 bash scripts/start_container.sh` のように実行します。Dockerコンテナを止めるときは起動したMacターミナルでCtrl-Cを押します。

プロジェクトは `/workspace/surry_hills`、rosbag・学習データ・モデルはMac側の `runtime_data/` に共有されます。コンテナを再起動してもデータが残ります。既存コンテナとポートが競合する場合は、先にそのコンテナを停止してください。ローカルに `ros2-desktop-ai-robot-book-humble:mine` があればそのイメージを優先します。変更する場合は `ROS2_DOCKER_IMAGE`、データ保存先は `ROVER_DATA_DIR` で指定できます。

## 2. コンテナ内の初回セットアップ

VNCデスクトップで端末を開いて実行します。GitHubから2つの既存リポジトリを取得し、ROSワークスペースをビルドします。

```bash
source /opt/ros/humble/setup.bash
cd /workspace/surry_hills
bash scripts/install_in_docker.sh
source ~/pai_ws/install/setup.bash
```

ビルド済みなら次回以降は最後の `source` だけでよく、再ビルド時はインストーラを再実行します。プロジェクトコードはMac上で編集すると共有フォルダ経由で反映されます。ワールドや車体形状を変えた後は、Gazeboを止めてパッケージを再ビルドし、再起動してください。

今回のOSMワールドへ切り替えるときは、古いGazeboを停止して、コンテナ内で一度次を実行します。

```bash
cd ~/pai_ws
colcon build --symlink-install --packages-select surry_hills_rover
source install/setup.bash
```

## 3. 手動デモを複数回収集

VNCの端末を3つ開きます。各端末でROS環境を読み込みます。

**端末1: GazeboとROS bridge**

```bash
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
ros2 launch surry_hills_rover surry_hills.launch.py
```

**端末2: 1回目の走行を記録**

```bash
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
ros2 bag record --use-sim-time -o /home/ubuntu/dataset/rosbags/surry_osm/episode_001 /model/rover/odometry /model/rover/cmd_vel /model/rover/key_counts
```

**端末3: キーボード操作**

```bash
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
ros2 run surry_hills_rover rover_teleop
```

操作端末の中を一度クリックしてフォーカスを当ててから、矢印キーを1回ずつ押します。上/下は約0.6 m前進・後退、左/右は約29度旋回です。入力は順番待ちになるため、各ステップが終わる前に次のキーを押しても順番に実行します。キーを短く押してください（押し続けるとキーボードの自動リピートも別の押下として数えます）。デモ記録を始める前に `x` を押すと、移動中の入力を消して緑の開始位置・向きへ戻り、押下数も0に戻します。走行デモの途中で `x` を押したbagは、位置が瞬間移動するため学習に使わず、記録し直してください。緑の出発地点から赤い目的地まで運転し、記録端末でCtrl-Cを押して保存します。次のエピソードではGazeboとテレオプを再起動してください。テレオプ終了時に `上, 下, 左, 右` の押下数が表示され、各bagのカウント値は変換時にも表示されます。記録先を `episode_002`、`episode_003` と変えてデモを集めます。旧ワールドのbagは新しい座標・ゴールに合わないため再利用せず、OSM版を新しく記録してください。

## 4. データ変換と学習

記録したbagをLeRobotデータセットに変換し、ACTを学習します。**変換・学習時はGazeboを終了しておく**と、Dockerのメモリを節約できます。変換時には各bagのキー別押下数も表示されます。学習対象は押下ごとに一定時間出力した `cmd_vel` で、カウントtopicはデモの定量評価用としてrosbagに保存します。

```bash
cd ~/pai_training
pixi run python scripts/convert_rover_bags.py --bag-dir /home/ubuntu/dataset/rosbags/surry_osm --output-dir /home/ubuntu/dataset/lerobot_dataset/single_o_to_rydges_osm
pixi run python scripts/train_rover.py
```

変換器は1秒を超える時刻の飛びを別のLeRobot episodeに分割します。Gazebo/ROS時刻が飛んでも、その空白を20 Hzで何十時間分も補間しません。変換が `Killed` で中断された場合、出力先には未完成のフォルダが残ることがあります。消さずに別の `--output-dir` を指定して再変換し、そのパスを学習時の `--dataset-root` に指定してください。学習時の `Repository not found` は、ローカルの `meta/info.json` がないか、`dataset_root` が違う場合に起きることがあります。この手順では公開Hubからデータを取らないため、通常Hub認証は不要です。

再変換先を変えた場合は、設定ファイルを編集しなくても次のように学習先を上書きできます。

```bash
pixi run python scripts/train_rover.py --dataset-root /home/ubuntu/dataset/lerobot_dataset/single_o_to_rydges_osm_retry1 --output-directory /home/ubuntu/checkpoints/train/single_o_to_rydges_osm_retry1_act
```

モデルは `/home/ubuntu/checkpoints/train/single_o_to_rydges_osm_act` に保存されます。設定は `/workspace/surry_hills/configs/` にあります。十分なデモを集めてから学習してください。

## 5. 推論

Gazeboを起動してロボットが初期位置にいる状態で、別端末にて実行します。

```bash
source /opt/ros/humble/setup.bash
source ~/pai_ws/install/setup.bash
cd ~/pai_training
pixi run python scripts/infer_rover.py --model-path /home/ubuntu/checkpoints/train/single_o_to_rydges_osm_act
```

推論ノードがオドメトリから位置・姿勢と目的地までの相対位置を作り、ACTが出した速度を `cmd_vel` に送ります。前進速度はデフォルトで1.5倍し、Gazeboの上限1.2 m/sで制限します。倍率を変える場合は `--speed-scale 2.0` のように指定します。目的地から0.8 m以内で停止します。手動操作と同時に実行しないでください。終了はCtrl-Cです。端末にはゴールまでの距離と速度指令が約1秒ごとに表示されます。

## OSMデータと構成

- `ros2_ws/src/surry_hills_rover/worlds/surry_hills.sdf`: OSM位置を縮尺した平面ワールド、6棟、4輪差動ロボット
- `ros2_ws/src/surry_hills_rover/launch/surry_hills.launch.py`: FortressとROS bridge
- `scripts/`: Docker起動、bag変換、ACT学習・推論
- `runtime_data/`: コンテナと共有するrosbag、データセット、チェックポイント

ワールドの原点をOpenStreetMapのSingle O (node 594438630, -33.8811804, 151.2096810) とし、カフェ地点を含む建物way 556198218を含め、周辺の建物位置・概略フットプリント・階数から6つの直方体を配置しています。緑マーカーと初期位置はカフェ建物のReservoir Street側の入口前に置きました。`x` キーはこの初期位置・向きへ戻します。ロボットには茶色のカップとコーヒー面を載せています。緯度経度を局所平面座標に変換し、全体を0.12倍に縮尺しています。目的地はRydgesの建物西側、Albion Street沿いです。OSM形状を簡略化したGazebo用ワールドで、実際の建物ポリゴンを完全再現するものではありません。

OSMデータ: [Single O node 594438630](https://www.openstreetmap.org/node/594438630)、[Rydges City Central way 540509392](https://www.openstreetmap.org/way/540509392)、[Single Oが入る建物way 556198218](https://www.openstreetmap.org/way/556198218)、建物way IDs 540509391, 540509393, 540509394, 540509395。© [OpenStreetMap contributors](https://www.openstreetmap.org/copyright) (ODbL)。
