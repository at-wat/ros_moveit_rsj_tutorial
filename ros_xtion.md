---
title: Xtion PRO Live からのデータ取得
date: 2018-05-23
---

- Table of contents
{:toc}

# Xtion PRO Live からのデータ取得

Xtion PRO Live からデータを取得し、2次元点群に変換する方法を説明します。

[Xtion PRO Live](https://www.asus.com/jp/3D-Sensor/Xtion_PRO_LIVE/) は深度（物体までの距離）および RGB カラー画像を取得できます。
USB バスパワーで駆動可能なデプスセンサで、非常に扱いやすいのですが現在は廃番となっており入手することはできません。

ROS で利用可能な類似のセンサとして[Orbbec Astra](https://orbbec3d.com/product-astra/)があります。
また Xtion PRO Live の後継機として[Xtion2](https://www.asus.com/jp/3D-Sensor/Xtion-2/)がありますが、こちらはまだ ROS 対応はしていません（2018/5/23時点）。

# 準備

## パッケージのインストール

Xtion PRO Live を利用するためのパッケージと３次元点群から２次元データに変換するためのパッケージをインストールします。
[準備のページ](linux_and_ros_install.html)に同様の手順を書いていますので、すでにインストールされている方はこの手順は不要です。

```shell
$ sudo apt install ros-kinetic-openni2-camera
$ sudo apt install ros-kinetic-openni2-launch
$ sudo apt install ros-kinetic-pointcloud-to-laserscan 
```

## launchファイルの入手

点群を3次元から2次元データに変換するパッケージの launch ファイル等をダウンロード

```shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/KMiyawaki/rsj_pointcloud_to_laserscan.git
```

# 動作確認

## launchファイルの実行
PC の USB ポートに Xtion PRO Live を接続し、次のコマンドを実行します。

```shell
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch rsj_pointcloud_to_laserscan rsj_pointcloud_to_laserscan.launch
```

コンソールに赤字でエラーメッセージが出ていないかどうか確認してください。
もしエラーメッセージが出ていたら、プログラムを __Ctrl+c__{: style="border: 1px solid black" } で終了し、 Xtion PRO Live を USB ポートから一旦抜いて再接続してからもう一度上記を実行してみてください。

## データの表示

別のコマンドターミナルを開き次を実行してください。

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_to_laserscan/config/rviz
$ rosrun rviz rviz -d view_scan.rviz
```

下図のように2次元の点群が表示されれば成功です。

![XtionScan](images/xtion_view_scan.png)

この2次元点群は3次元センサが出力している3次元点群から`pointcloud_to_laserscan`ノードを使って、高さの範囲指定で切り出して生成しているものです。
3次元点群そのものの処理については後に続く実習で説明します。
