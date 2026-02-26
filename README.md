# Unitree ROS2 to Real

このパッケージはUnitreeロボット（Go1など）をROS2から制御するためのインターフェースです。
High-level制御（歩行方向・速度）とLow-level制御（全関節制御）の両方に対応しています。

**🎉 このリポジトリ単体で完結**: メッセージ定義（`ros2_unitree_legged_msgs`）も含まれており、このリポジトリをクローンするだけで全て揃います。

## 機能

- High-level制御: `/cmd_vel`トピックでロボットの移動を制御
- High State購読: ロボットの状態を`/high_state`トピックで取得
- **Low State購読: ロボットの詳細な状態（関節角度、トルクなど）を`/low_state`トピックで取得** ✨
- Odometry配信: `/odom`トピックでオドメトリ情報を配信
- **独自メッセージ定義**: 12種類のメッセージ型が含まれており、追加のリポジトリ不要

## 📋 目次

- [必要な環境](#必要な環境)
- [クイックスタート](#クイックスタート)
- [詳細なセットアップ](#詳細なセットアップ)
- [ネットワーク設定](#ネットワーク設定)
- [使い方](#使い方)
- [動作確認](#動作確認)
- [トラブルシューティング](#トラブルシューティング)

## 必要な環境

- Ubuntu 20.04 / 22.04 / 24.04
- ROS2 (Humble / Jazzy 推奨)
- Git
- CMake 3.8以上
- C++14対応のコンパイラ
- unitree_legged_sdk v3.5.1 以降

## 🚀 クイックスタート

初めての方向けの最短セットアップ手順です。

```bash
# 1. ワークスペース作成（既にある場合はスキップ）
mkdir -p ~/ros2_dog_ws/src
cd ~/ros2_dog_ws/src

# 2. このリポジトリをクローン（既にある場合はスキップ）
git clone <このリポジトリのURL> unitree_ros2_to_real

# 3. メッセージパッケージのシンボリックリンクを作成
# （このステップで ros2_unitree_legged_msgs を colcon が検出できるようにします）
ln -s unitree_ros2_to_real/ros2_unitree_legged_msgs ros2_unitree_legged_msgs

# 4. Unitree SDK をインストール
cd unitree_ros2_to_real
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git

# 5. ビルド
cd ~/ros2_dog_ws
colcon build --symlink-install

# 6. 環境設定を読み込み
source install/setup.bash

# 6. ネットワーク設定（ロボットと接続後）
# インターフェース名を確認
ifconfig
# IPアドレスを設定（<インターフェース名>を実際の名前に置き換え）
sudo ifconfig <インターフェース名> 192.168.123.162 netmask 255.255.255.0

# 7. ノード起動
ros2 run unitree_udp_sender unitree_udp_sender

# 8. 別のターミナルで動作確認
ros2 topic list
ros2 topic echo /low_state
```

## 詳細なセットアップ

### 前提条件の確認

#### ROS2のインストール確認

```bash
# ROS2がインストールされているか確認
ros2 --version

# インストールされていない場合
# Ubuntu 22.04 + ROS2 Humble の例
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop

# 環境設定（.bashrcに追加推奨）
source /opt/ros/humble/setup.bash
```

#### 必要な依存パッケージのインストール

```bash
# ROS2の基本パッケージ
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libboost-all-dev

# rosdepの初期化（初回のみ）
sudo rosdep init
rosdep update
```

## セットアップ

### 1. ワークスペースの作成

```bash
# ワークスペースディレクトリを作成
mkdir -p ~/ros2_dog_ws/src
cd ~/ros2_dog_ws/src
```

### 2. このパッケージの取得

```bash
# このリポジトリをクローン
cd ~/ros2_dog_ws/src
git clone <このリポジトリのURL> unitree_ros2_to_real
```

### 2.5. メッセージパッケージのシンボリックリンク作成

このリポジトリには `ros2_unitree_legged_msgs` が含まれていますが、colconがそれを検出できるよう、シンボリックリンクを作成します：

```bash
cd ~/ros2_dog_ws/src
ln -s unitree_ros2_to_real/ros2_unitree_legged_msgs ros2_unitree_legged_msgs

# パッケージが検出されることを確認
cd ~/ros2_dog_ws
colcon list
# 以下のように2つのパッケージが表示されればOK:
# ros2_unitree_legged_msgs
# unitree_udp_sender
```

**なぜシンボリックリンクが必要？**
- colconはデフォルトで `src` 直下のパッケージを検出します
- `unitree_ros2_to_real/ros2_unitree_legged_msgs` のようにネストされたパッケージは検出されません
- シンボリックリンクを使うことで、実際のファイルは `unitree_ros2_to_real` 内に残したまま、colconに検出させることができます

### 3. Unitree Legged SDK のインストール

```bash
cd ~/ros2_dog_ws/src/unitree_ros2_to_real
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git

# SDKのヘッダーファイルとライブラリが正しく配置されているか確認
ls unitree_legged_sdk/include/unitree_legged_sdk/
ls unitree_legged_sdk/lib/cpp/
```

**注意**: SDKのビルドは不要です。プリビルドされたライブラリが含まれています。

### 4. 依存関係の解決

```bash
cd ~/ros2_dog_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. パッケージのビルド

```bash
cd ~/ros2_dog_ws
colcon build --symlink-install

# ビルドの成功を確認
echo $?  # 0が表示されればビルド成功
```

### 6. 環境設定

```bash
source ~/ros2_dog_ws/install/setup.bash

# .bashrcに追加しておくと便利
echo "source ~/ros2_dog_ws/install/setup.bash" >> ~/.bashrc
```

## リポジトリ構造

このリポジトリは以下の構造になっています：

```
unitree_ros2_to_real/
├── CMakeLists.txt              # メインパッケージのビルド設定
├── package.xml                 # メインパッケージの依存関係
├── README.md                   # このファイル
├── LICENSE
├── include/
│   └── unitree_udp_sender/     # ヘッダーファイル
│       ├── convert.hpp         # メッセージ変換関数
│       └── unitree_udp_sender_component.hpp  # メインコンポーネント
├── src/
│   └── unitree_udp_sender_component.cpp  # 実装ファイル
├── ros2_unitree_legged_msgs/   # メッセージ定義パッケージ（このリポジトリに含まれる）
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/                    # 12種類のメッセージ定義
│       ├── BmsCmd.msg
│       ├── BmsState.msg
│       ├── Cartesian.msg
│       ├── HighCmd.msg
│       ├── HighCmdArray.msg
│       ├── HighState.msg
│       ├── IMU.msg
│       ├── LED.msg
│       ├── LowCmd.msg
│       ├── LowState.msg
│       ├── MotorCmd.msg
│       └── MotorState.msg
└── unitree_legged_sdk/         # Unitree SDK（別途クローン）
    ├── include/
    └── lib/
```

**重要**: `ros2_unitree_legged_msgs` はこのリポジトリに含まれているため、別途クローンする必要はありません。

## ネットワーク設定

### 1. 物理接続

ロボットとPCをイーサネットケーブルで直接接続してください。

### 2. ネットワークインターフェースの確認

```bash
# 接続されているネットワークインターフェースを確認
ifconfig
# または
ip addr show
```

USB-Ethernetアダプタを使用している場合、インターフェース名は通常 `enx` で始まります（例: `enx000ec6612921`）。

### 3. IPアドレスの設定

```bash
# 一時的な設定（再起動すると消える）
sudo ifconfig <インターフェース名> 192.168.123.162 netmask 255.255.255.0

# 例:
# sudo ifconfig enx000ec6612921 192.168.123.162 netmask 255.255.255.0
```

### 4. 永続的なネットワーク設定（オプション）

Ubuntu 22.04以降の場合、Netplanを使用：

```bash
sudo nano /etc/netplan/01-unitree-network.yaml
```

以下の内容を追加（インターフェース名を適宜変更）：

```yaml
network:
  version: 2
  ethernets:
    enx000ec6612921:  # 実際のインターフェース名に変更
      addresses:
        - 192.168.123.162/24
      dhcp4: false
```

設定を適用：

```bash
sudo netplan apply
```

### 5. 接続確認

```bash
# ロボットへのpingを確認
ping 192.168.123.161

# Ctrl+Cで停止
```

pingが通れば接続成功です。

## 使い方

### ノードの起動

#### 基本的な起動

```bash
# 環境設定を読み込み（.bashrcに追加していない場合）
source ~/ros2_dog_ws/install/setup.bash

# ノードを起動
ros2 run unitree_udp_sender unitree_udp_sender
```

#### パラメータを指定して起動

```bash
ros2 run unitree_udp_sender unitree_udp_sender --ros-args \
  -p udp_settings.target_ip_address:=192.168.123.161 \
  -p udp_settings.local_port:=8090 \
  -p udp_settings.target_port:=8082 \
  -p udp_settings.low_local_port:=8007 \
  -p udp_settings.low_target_port:=8007
```

#### ロボットのIPアドレスが異なる場合

```bash
# 例: ロボットのIPが 192.168.123.10 の場合
ros2 run unitree_udp_sender unitree_udp_sender --ros-args \
  -p udp_settings.target_ip_address:=192.168.123.10
```

正常に起動すると、ロボットからの状態情報が受信され始めます。

### ロボットの制御

#### cmd_velで移動制御

別のターミナルを開いて：

```bash
# 環境設定
source ~/ros2_dog_ws/install/setup.bash

# 前進（0.5 m/s）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 回転（0.3 rad/s）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# 停止
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### キーボード操作（teleop_twist_keyboardを使用）

```bash
# teleop_twist_keyboardのインストール（初回のみ）
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard

# キーボードでロボットを操作
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 動作確認

### トピックの確認

```bash
# 利用可能なトピック一覧
ros2 topic list

# 期待される出力:
# /cmd_vel
# /high_state
# /low_state      ← Low State情報（追加機能）
# /odom
# /pose
# /velocity
# ...
```

### Low Stateの確認

```bash
# Low State全体を表示
ros2 topic echo /low_state

# 更新頻度を確認（約500Hzが正常）
ros2 topic hz /low_state

# データ型を確認
ros2 topic info /low_state

# タイムスタンプを確認（Go1のtickベース）
ros2 topic echo /low_state/header

# tickの生値を確認（モーションコントローラーからのms）
ros2 topic echo /low_state/tick

# 特定のデータのみ表示（例: 0番目の関節の角度）
ros2 topic echo /low_state/motor_state[0]/q

# IMU情報のみ表示
ros2 topic echo /low_state/imu

# バッテリー状態のみ表示
ros2 topic echo /low_state/bms
```

#### タイムスタンプの確認

Go1のモーションコントローラーからの`tick`が正しくheaderのタイムスタンプに変換されているか確認：

```bash
# タイムスタンプとtickを同時に表示
ros2 topic echo /low_state --field header.stamp --field tick

# タイムスタンプの差分を確認（毎回約2ms = 500Hz）
ros2 topic echo /low_state/header/stamp
```

### 関節情報の確認

Low Stateから20個の関節の状態を確認できます：

```bash
# 全関節の角度を表示
ros2 topic echo /low_state/motor_state --field q

# 全関節のトルクを表示  
ros2 topic echo /low_state/motor_state --field tau_est

# 全関節の温度を表示
ros2 topic echo /low_state/motor_state --field temperature
```

### 視覚化（オプション）

RViz2でオドメトリを視覚化：

```bash
# RViz2を起動
rviz2

# Fixed Frameを "map" に設定
# Add -> Odometry -> Topic: /odom
```

### データの記録

rosbag2を使ってデータを記録：

```bash
# 全トピックを記録
ros2 bag record -a

# 特定のトピックのみ記録
ros2 bag record /low_state /high_state /odom

# 記録したデータの再生
ros2 bag play <bag_file_name>
```

## トピック一覧

### Subscribed Topics

| トピック | メッセージ型 | 説明 |
|---------|------------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | ロボットの移動速度指令 |
| `/high_cmd` | `ros2_unitree_legged_msgs/HighCmd` | High-levelコマンド |
| `/consecutive_motion` | `ros2_unitree_legged_msgs/HighCmdArray` | 連続動作コマンド |

### Published Topics

| トピック | メッセージ型 | 更新頻度 | 説明 |
|---------|------------|---------|------|
| `/high_state` | `ros2_unitree_legged_msgs/HighState` | 500Hz | ロボットのHigh-level状態 |
| **`/low_state`** | `ros2_unitree_legged_msgs/LowState` | **500Hz** | **ロボットのLow-level状態** ✨ |
| `/odom` | `nav_msgs/Odometry` | 500Hz | オドメトリ情報 |
| `/pose` | `geometry_msgs/PoseStamped` | 500Hz | ロボットの姿勢 |
| `/velocity` | `geometry_msgs/TwistStamped` | 500Hz | ロボットの速度 |

### Services

| サービス | サービス型 | 説明 |
|---------|-----------|------|
| `/emg_switch` | `std_srvs/SetBool` | 緊急停止の有効/無効切り替え |

### Low State メッセージの内容

`/low_state`トピックには以下の詳細な情報が含まれます：

- **header**: 標準的なROS2 Header
  - `stamp`: タイムスタンプ（Go1のモーションコントローラーからの`tick`を基準に計算）
  - `frame_id`: "base_link"
- **motor_state[20]**: 全20個の関節の状態
  - `mode`: 制御モード
  - `q`: 関節角度 [rad]
  - `dq`: 関節角速度 [rad/s]
  - `ddq`: 関節角加速度 [rad/s²]
  - `tau_est`: 推定トルク [N·m]
  - `temperature`: モーター温度 [℃]
- **imu**: IMU（慣性計測装置）情報
  - `quaternion[4]`: 姿勢（クォータニオン）
  - `gyroscope[3]`: 角速度 [rad/s]
  - `accelerometer[3]`: 加速度 [m/s²]
  - `rpy[3]`: ロール・ピッチ・ヨー角 [rad]
  - `temperature`: IMU温度 [℃]
- **foot_force[4]**: 4本の足の接触力推定値
- **foot_force_est[4]**: 4本の足の接触力測定値
- **bms**: バッテリー管理システム情報
  - `version_h`, `version_l`: バージョン
  - `bms_status`: BMS状態
  - `soc`: 充電率 [%]
  - `current`: 電流 [A]
  - `cell_vol[10]`: セル電圧
  - `bq_ntc[2]`, `mcu_ntc[2]`: 温度センサー値
- **wireless_remote[40]**: ワイヤレスリモコンの入力
- **tick**: モーションコントローラーからのタイムスタンプ [ms]（Go1側の時刻）
- その他: `head`, `sn`, `version`, `reserve`, `crc`

#### タイムスタンプについて

Low Stateの`header.stamp`は、Go1のモーションコントローラーから送信される`tick`フィールド（ミリ秒単位）を基準に生成されます：

1. **最初のメッセージ受信時**: `tick`の初期値とその時点のPC時刻を記録
2. **以降のメッセージ**: `tick`の差分を計算し、PC時刻に加算してタイムスタンプを生成

この方法により、Go1側の時間軸を保ちながら、PC時刻との同期を取ることができます。UDP通信の遅延は含まれますが、Go1内部での相対的な時間関係は正確に保持されます。

**重要**: `tick`はGo1の起動時刻を基準とした相対時刻のため、絶対時刻ではありません。初回受信時にPC時刻と同期することで、ROS2のタイムスタンプとして利用できるようにしています。

## パラメータ

| パラメータ | 型 | デフォルト値 | 説明 |
|-----------|----|-----------|----|
| `udp_settings.target_ip_address` | string | `192.168.123.161` | ロボットのIPアドレス |
| `udp_settings.local_port` | int | `8090` | High-level通信のローカルポート |
| `udp_settings.target_port` | int | `8082` | High-level通信のターゲットポート |
| `udp_settings.low_local_port` | int | `8007` | Low-level通信のローカルポート |
| `udp_settings.low_target_port` | int | `8007` | Low-level通信のターゲットポート |

## トラブルシューティング

### ビルドエラー

#### `unitree_legged_sdk が見つからない`

**原因**: SDKが正しい場所にインストールされていない

**解決方法**:
```bash
# SDKの存在確認
ls ~/ros2_dog_ws/src/unitree_ros2_to_real/unitree_legged_sdk

# 存在しない場合、再インストール
cd ~/ros2_dog_ws/src/unitree_ros2_to_real
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git

# 再ビルド
cd ~/ros2_dog_ws
colcon build --symlink-install --packages-select unitree_udp_sender
```

#### `ros2_unitree_legged_msgs が見つからない`

**原因**: メッセージパッケージがビルドされていない

**解決方法**:
```bash
cd ~/ros2_dog_ws
colcon build --packages-select ros2_unitree_legged_msgs
source install/setup.bash
colcon build --packages-select unitree_udp_sender
```

### 実行時エラー

#### ノードが起動しない

**症状**: `ros2 run unitree_udp_sender unitree_udp_sender` でエラー

**確認事項**:
```bash
# 環境設定が読み込まれているか確認
env | grep ROS

# パッケージが正しくインストールされているか確認
ros2 pkg list | grep unitree

# 実行ファイルが存在するか確認
ls ~/ros2_dog_ws/install/unitree_udp_sender/lib/unitree_udp_sender/
```

**解決方法**:
```bash
# 環境設定を再読み込み
source ~/ros2_dog_ws/install/setup.bash

# 再ビルド
cd ~/ros2_dog_ws
colcon build --symlink-install
source install/setup.bash
```

#### ロボットと通信できない

**症状**: トピックにデータが流れてこない、または古いデータのまま

**確認事項**:
1. **ネットワーク接続**:
```bash
# ロボットにpingが通るか確認
ping 192.168.123.161

# ネットワークインターフェースのIPアドレス確認
ifconfig
```

2. **ファイアウォール**:
```bash
# ファイアウォール状態確認
sudo ufw status

# 一時的に無効化（テスト用）
sudo ufw disable

# 必要なポートを開放（本番用）
sudo ufw allow 8007/udp
sudo ufw allow 8082/udp
sudo ufw allow 8090/udp
sudo ufw enable
```

3. **ロボットの状態**:
   - ロボットの電源が入っているか
   - ロボットがスタンバイモードになっているか
   - 緊急停止ボタンが押されていないか

#### データ更新頻度が低い

**症状**: `ros2 topic hz /low_state` で表示される頻度が500Hzより大幅に低い

**原因と解決方法**:
1. **CPU負荷が高い**:
```bash
# CPU使用率確認
top
# 不要なプロセスを終了
```

2. **ネットワーク帯域不足**:
   - USB2.0のEthernetアダプタを使用している場合、USB3.0のものに交換
   - 他のネットワーク通信を停止

3. **リアルタイム性の向上**（上級者向け）:
```bash
# プロセス優先度を上げる
sudo nice -n -20 ros2 run unitree_udp_sender unitree_udp_sender
```

### その他の問題

#### cmd_velで制御できない

**確認事項**:
```bash
# cmd_velトピックが存在するか
ros2 topic list | grep cmd_vel

# Subscriberがいるか確認
ros2 topic info /cmd_vel

# 緊急停止が有効になっていないか確認（無効化）
ros2 service call /emg_switch std_srvs/srv/SetBool "{data: false}"
```

#### 関節が動かない

Low-level制御を行う場合は、ロボットを適切なモードに設定する必要があります。High-level制御（/cmd_vel）では自動的に歩行制御されます。

### デバッグのヒント

```bash
# ログレベルを DEBUG に設定して起動
ros2 run unitree_udp_sender unitree_udp_sender --ros-args --log-level debug

# rqtでトピックをモニタリング
rqt

# ネットワークトラフィックを確認
sudo tcpdump -i <インターフェース名> udp port 8007 or udp port 8082
```

## よくある質問（FAQ）

### Q1: このパッケージはどのUnitreeロボットに対応していますか？

A: Unitree Go1ロボットに対応しています。他のUnitreeロボット（A1、Aliengoなど）でも動作する可能性がありますが、unitree_legged_sdk v3.5.1との互換性を確認してください。

### Q2: ros2_unitree_legged_msgsは別途インストールが必要ですか？

A: **不要です**。`ros2_unitree_legged_msgs` はこのリポジトリに含まれています。シンボリックリンクを作成するだけで使用できます。

### Q3: シミュレータで動作確認できますか？

A: このパッケージは実機との通信専用です。シミュレーション環境での動作確認には、Gazeboとunitree_ros2を使用することをお勧めします。

### Q4: Low StateとHigh Stateの違いは何ですか？

A:
- **High State**: 歩行制御レベルの情報（姿勢、速度、フットステップなど）
- **Low State**: 各関節の詳細情報（角度、トルク、温度など）とセンサー生データ

Low Stateを使うことで、より細かい制御やデータ収集が可能になります。

### Q5: 複数のロボットを同時に制御できますか？

A: はい、可能です。各ロボットに対して異なるIPアドレスとパラメータを設定してノードを複数起動してください：

```bash
# ロボット1
ros2 run unitree_udp_sender unitree_udp_sender --ros-args \
  -r __ns:=/robot1 \
  -p udp_settings.target_ip_address:=192.168.123.161

# ロボット2  
ros2 run unitree_udp_sender unitree_udp_sender --ros-args \
  -r __ns:=/robot2 \
  -p udp_settings.target_ip_address:=192.168.123.162
```

### Q5: ロボットが勝手に動き出すことはありますか？

A: ノードを起動しただけでは動きません。`/cmd_vel`トピックに速度指令を送信するか、High Cmdを送信した時のみロボットが動作します。安全のため、緊急停止ボタンは常に手の届く位置に置いてください。

### Q6: Windows や macOS で使用できますか？

A: このパッケージはLinux専用です。ROS2はWindows/macOSでも動作しますが、unitree_legged_sdkがLinuxのみをサポートしているためです。WSL2（Windows Subsystem for Linux）を使用すれば、Windows上でも動作する可能性があります。
### Q8: Low Stateのタイムスタンプは正確ですか？

A: `header.stamp`は**Go1のモーションコントローラーからの`tick`を基準**に生成されます：

- **利点**: Go1内部での相対的な時間関係は正確に保持されます（500Hzで安定）
- **注意点**: UDP通信の遅延（通常1-5ms程度）が含まれます
- **初期化**: 最初のメッセージ受信時にPC時刻と同期します

絶対時刻の精度が重要な場合は、NTPでPC時刻を同期し、rosbagに記録する際の`header.stamp`を利用してください。Go1内部でのイベント相対時刻（例：関節角度とIMUデータの同期）は高精度です。

### Q9: tickがオーバーフローした場合はどうなりますか？

A: `tick`はuint32_t（約49.7日でオーバーフロー）ですが、本パッケージはオーバーフローを自動的に処理します。連続稼働時も時刻情報は正常に動作します。
## 参考リンク

- [Unitree Robotics 公式サイト](https://www.unitree.com/)
- [unitree_legged_sdk GitHub](https://github.com/unitreerobotics/unitree_legged_sdk)
- [ROS2 Documentation](https://docs.ros.org/)
- [ROS2 Humble インストールガイド](https://docs.ros.org/en/humble/Installation.html)

## コントリビューション

バグ報告や機能追加のリクエストは、GitHubのIssuesでお願いします。プルリクエストも歓迎します。

## 更新履歴

- **v0.3.0** (2026-02-26)
  - **LowStateにstd_msgs/Headerを追加**
  - Go1のモーションコントローラーからの`tick`を利用した時刻同期機能を実装
  - タイムスタンプがGo1側の相対時刻を反映するようになった

- **v0.2.0** (2026-02-26)
  - **リポジトリ構成を単一リポジトリに統合**
  - `ros2_unitree_legged_msgs` をリポジトリ内に含めて依存関係を簡素化
  - シンボリックリンクを使用したビルド方法を導入
  - ドキュメントを大幅に拡充

- **v0.1.0** (2026-02-26)
  - Low State購読機能を追加
  - ビルドシステムを`extension_ament`から標準`ament_cmake`に移行
  - 詳細なドキュメントを作成
  - メッセージパッケージの構造を整理

## ライセンス

TODO: ライセンスを明記してください

## メンテナー

unitree <laikago@unitree.cc>

## 謝辞

このパッケージは、Unitree Roboticsが提供するunitree_legged_sdkをベースに開発されています。

---

**安全に関する注意**: ロボットの動作範囲内に人や障害物がないことを確認してから制御を開始してください。緊急停止ボタンを常に手の届く位置に配置してください。
