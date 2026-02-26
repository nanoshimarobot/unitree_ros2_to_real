# セットアップクイックガイド

このリポジトリを使用するための最短セットアップ手順です。

## 前提条件

- Ubuntu 20.04 / 22.04 / 24.04
- ROS2 (Humble / Jazzy)
- Git

## セットアップ手順

### 1. ワークスペース準備

```bash
mkdir -p ~/ros2_dog_ws/src
cd ~/ros2_dog_ws/src
```

### 2. リポジトリのクローン

```bash
git clone <このリポジトリのURL> unitree_ros2_to_real
cd unitree_ros2_to_real
```

### 3. Unitree SDK のインストール

```bash
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
```

### 4. メッセージパッケージのリンク作成

```bash
cd ~/ros2_dog_ws/src
ln -s unitree_ros2_to_real/ros2_unitree_legged_msgs ros2_unitree_legged_msgs
```

### 5. ビルド

```bash
cd ~/ros2_dog_ws
colcon build --symlink-install
source install/setup.bash
```

### 6. パッケージ確認

```bash
ros2 pkg list | grep unitree
# 以下が表示されればOK:
# ros2_unitree_legged_msgs
# unitree_udp_sender
```

## ネットワーク設定

### PCのIPアドレス設定

```bash
# インターフェース名を確認
ifconfig

# IPアドレスを設定（<インターフェース名>を実際の名前に置き換え）
sudo ifconfig <インターフェース名> 192.168.123.162 netmask 255.255.255.0

# ロボットへの接続確認
ping 192.168.123.161
```

## ノード起動

```bash
# 環境設定（必要に応じて）
source ~/ros2_dog_ws/install/setup.bash

# ノード起動
ros2 run unitree_udp_sender unitree_udp_sender
```

## 動作確認

別のターミナルで：

```bash
source ~/ros2_dog_ws/install/setup.bash

# トピック一覧確認
ros2 topic list

# Low State確認（Low-level状態情報）
ros2 topic echo /low_state

# データ更新頻度確認
ros2 topic hz /low_state
```

## トラブルシューティング

### ビルドエラーが出る場合

```bash
# クリーンビルド
cd ~/ros2_dog_ws
rm -rf build install log
colcon build --symlink-install
```

### パッケージが見つからない場合

```bash
# シンボリックリンクが作成されているか確認
ls -la ~/ros2_dog_ws/src/ros2_unitree_legged_msgs

# パッケージリスト確認
cd ~/ros2_dog_ws
colcon list
# 2つのパッケージが表示されるはず
```

### ロボットと通信できない場合

1. ネットワーク接続を確認: `ping 192.168.123.161`
2. ファイアウォールを無効化: `sudo ufw disable`
3. ロボットの電源が入っているか確認

## 詳細情報

詳細なドキュメントは [README.md](./README.md) を参照してください。

---

**安全に関する注意**: ロボットの動作範囲内に人や障害物がないことを確認してから制御を開始してください。
