# 3次元点群の処理

Xtion PRO Live や YVT-35LX から得られる３次元点群（Point Cloud）に対する基本的な処理を実習します。

## Point Cloud の表示

各センサから得られる点群をRViz によって可視化します。センサごとに実行するコマンドが異なりますので、お手持ちのセンサに応じて次のコマンドを実行してください。

### Xtion PRO Live の場合

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_to_laserscan/launch
$ roslaunch rsj_pointcloud_to_laserscan.launch
```

別のターミナルを開き

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_to_laserscan/config/rviz
$ rosrun rviz rviz -d view_points.rviz
```

図のように Point Cloud が表示されれば成功です。

![XtionPoints](images/xtion_view_points.png)

起動した２つのターミナルを Ctrl+Cで終了してください。

### YVT-35LX の場合

？？？？

## PCL（Point Cloud Library）による３次元点群処理

ロボットでオドメトリのデータを利用したときと同じように３次元センサが出力したPoint Cloudのデータを受け取るプログラムを作成しましょう。Point Cloudの処理を独自に書こうとすると大変な労力が必要です。
２次元画像処理用に[OpenCV](https://opencv.org/)というライブラリがあるように、３次元点群処理には[PCL(Point Cloud Library)](http://pointclouds.org/) があります。PCLを利用するプログラムを作成してみましょう。

端末を開き雛形をダウンロードしてください。
```shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/KMiyawaki/rsj_pointcloud_test.git
```

テキストエディタで rsj_pointcloud_test_node.cpp を開いてください。

```shell
$ cd catkin_ws/src/rsj_pointcloud_test/src
任意のテキストエディタで rsj_pointcloud_test_node.cpp を開く
```

rsj_pointcloud_test_node クラスにある、「PointCloud」用のコールバック関数cb_points を編集します。
```c++
  void cb_points(const PointCloud::ConstPtr &msg)
  {
    ROS_INFO("width: %d, height: %d", msg->width, msg->height);
  }
```

ファイルを保存してエディタを閉じます。

## ビルド＆実行
まず、catkin_wsでcatkin_makeを実行して、追加したコードをビルドします。
```shell
$ cd ~/catkin_ws
$ catkin_make 
```
次にお手持ちの３次元センサを起動します。

### Xtion PRO Live の場合

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_to_laserscan/launch
$ roslaunch rsj_pointcloud_to_laserscan.launch
```

### YVT-35LX の場合

？？？？

新しいターミナルを開き、 rsj_pointcloud_test_node を起動します。

```shell
$ rosrun rsj_pointcloud_test rsj_pointcloud_test_node 
[ INFO] [1523925497.474555840]: Hello ROS World!
[ INFO] [1523925498.184072011]: width: 640, height: 480
```
このように「width: xxx, height: xxx」というメッセージが表示されれば「PointCloud」は受信できています。
width, height とは３次元点群の縦・横の点の数を示しています（２次元画像の解像度に対応しています）。

## 補足 rsj_pointcloud_test_node.cpp について

プログラムの先頭にはROS内でPCLを扱うためのヘッダファイルに関するinclude 文と実習で使うPointCloudの型宣言が記述されています。

```c++
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
typedef pcl::PointXYZ PointT
typedef pcl::PointCloud<PointT> PointCloud;
```

最終行の typedef 宣言では今回の実習で利用する点群の型を生成しています。PCLではpcl::PointCloud<T>というC++のテンプレートで点群を扱うデータ型を表現しています。Tの部分には座標と色情報を持つpcl::PointXYZRGB など様々な点の型を与えることが可能です。今回は色情報のない、位置だけの点pcl::PointXYZ を使います。

なお、#include <visualization_msgs/MarkerArray.h> は点群処理結果を可視化するために必要となる ROS に含まれるヘッダファイルです。

rsj_pointcloud_test_node クラスの冒頭には、sub_odomと同じように「PointCloud」用のサブスクライバクラスを宣言しています。

```c++
class rsj_pointcloud_test_node 
{
private:
  ros::Subscriber sub_points;
```

rsj_pointcloud_test_node のコンストラクタには、「PointCloud」用のサブスクライバ初期化コードが記述されています。

```c++
rsj_pointcloud_test_node()
{
  ros::NodeHandle nh("~");
  sub_points = nh.subscribe("/camera/depth_registered/points", 5, &rsj_robot_test_node::cb_points, this);
```

CMakeLists.txtではPCLをROSで扱えるようにしています。

```c++
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs geometry_msgs sensor_msgs tf pcl_ros)
(注：最後に pcl_ros を追加している)
```

終了したら PCL を使った点群処理に関する実習に進んでください。

[PointCloud に対するフィルタ](ros_3d_points_filters.html)