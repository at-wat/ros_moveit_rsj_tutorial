---
title: PointCloud に対するフィルタ
date: 2018-05-15
---

- Table of contents
{:toc}

３次元点群に対して様々なフィルタを施し、移動ロボットの追跡対象など意味のある情報を抽出します。

# PassThrough フィルタ

`PassThrough`フィルタは得られた点群のうち、一定の範囲内にある点群のみを抽出します。
テキストエディタで`rsj_pointcloud_test_node.cpp`を開いてください。

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_test/src
任意のテキストエディタで rsj_pointcloud_test_node.cpp を開く
```

プログラム冒頭に`include`文を追記してください。

```c++
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  // 追記
#include <visualization_msgs/MarkerArray.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
```

`RsjPointcloudTestNode`クラスの冒頭に、`pcl::PassThrough`フィルタのインスタンスを追加します。
また、フィルタの結果を格納するための`PointCloud`型変数`cloud_passthrough_`、および処理結果を`publish`するためのパブリッシャ`pub_passthrough_`を追加します。

```c++
class RsjPointcloudTestNode
{
private:
略
  PointCloud::Ptr cloud_tranform_;
  // 以下を追記
  pcl::PassThrough<PointT> pass_;
  PointCloud::Ptr cloud_passthrough_;
  ros::Publisher pub_passthrough_;
```

`RsjPointcloudTestNode`クラスのコンストラクタで`PassThrough`フィルタの設定、`cloud_passthrough_`および`pub_passthrough_`を初期化します。

```c++
RsjPointcloudTestNode()
{
  略
  cloud_tranform_.reset(new PointCloud());
  // 以下を追記
  pass_.setFilterFieldName("z");  // Z軸（高さ）の値でフィルタをかける
  pass_.setFilterLimits(0.1, 1.0);  // 0.1 ～ 1.0 m の間にある点群を抽出
  cloud_passthrough_.reset(new PointCloud());
  pub_passthrough_ = pnh_.advertise<PointCloud>("passthrough", 1);
}
```

`cbPoints`関数を次のように変更します。

```c++
void cbPoints(const PointCloud::ConstPtr &msg)
{
  try{
    略
    // ここに cloud_src に対するフィルタ処理を書く
    pass_.setInputCloud(cloud_src);
    pass_.filter(*cloud_passthrough_);
    pub_passthrough_.publish(cloud_passthrough_);
    //  ROS_INFO("width: %u, height: %u", cloud_src->width, cloud_src->height);  // 削除
    ROS_INFO("points (src: %zu, paththrough: %zu)", cloud_src->size(), cloud_passthrough_->size());  // 追記
  }catch (std::exception &e){
    ROS_ERROR("%s", e.what());
  }
}
```

## ビルド＆実行

まず、`catkin_ws`で`catkin_make`を実行して、追加したコードをビルドします。

```shell
$ cd ~/catkin_ws
$ catkin_make 
```

次にお手持ちの３次元センサごとに次のようにノードを起動します。

### Xtion PRO Live の場合

ターミナルでセンサを起動します。

```shell
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch rsj_pointcloud_to_laserscan rsj_pointcloud_to_laserscan.launch
```

新しいターミナルを開き、`rsj_pointcloud_test_node`を起動します。

```shell
$ rosrun  rsj_pointcloud_test rsj_pointcloud_test_node \
_target_frame:=camera_link _topic_name:=/camera/depth_registered/points
[ INFO] [1524040063.315596383]: target_frame='camera_link'
[ INFO] [1524040063.315656650]: topic_name='/camera/depth_registered/points'
[ INFO] [1524040063.320448185]: Hello Point Cloud!
[ INFO] [1524040064.148595331]: points (src: 307200, paththrough: 34350)
```

### YVT-35LX の場合

ターミナルでセンサを起動します。

```shell
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch rsj_pointcloud_to_laserscan rsj_pointcloud_to_laserscan_3durg.launch
```

新しいターミナルを開き、`rsj_pointcloud_test_node`を起動します。

```shell
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun rsj_pointcloud_test rsj_pointcloud_test_node \
_target_frame:= _topic_name:=/hokuyo3d/hokuyo_cloud2
[ INFO] [1528008816.751100536]: points (src: 2674, paththrough: 1019)
```

実行した際に`points (src: xxxx, paththrough: xxx)`というメッセージが表示されれば成功です。
`src`、`paththrough`に続けて表示されている値はセンサから得られた、もとの`PointCloud`における点の個数と`PassThrough`フィルタ実行後の点の個数を示しています。
フィルタ実行後の点の個数がゼロの場合は`pass_.setFilterLimits(0.5, 1.0);`の引数を調節してみてください。

## フィルタ実行結果の可視化

RViz でフィルタ実行後の点群の様子を可視化します。rsj_pointcloud_test_node を起動したまま、新しいターミナルを開き、 RViz を起動します。

### Xtion PRO Live の場合

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_test/config/rviz
$ rviz -d view_filters.rviz
```

### YVT-35LX の場合 の場合

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_test/config/rviz
$ rviz -d view_filters_3durg.rviz
```

なお、 RViz は`roscore`が起動していれば、上記のように`rviz`とタイプするだけでも実行可能です。

図のように Rviz の左方にある`PointCloud2`のチェックボックスのうち、上2つだけにチェックを入れてください。
フィルタ実行前と実行後の点群が重なって表示されています。

![XtionPointsOrigin](images/xtion_view_filter_origin.png)

`PointCloud2`のチェックボックスのうち上から2番めのチェックだけをチェックするとフィルタ実行後の点群だけが表示されます。

![XtionPointsPassThrough](images/xtion_view_passthrough.png)

# VoxelGrid フィルタ

３次元点群の処理には時間がかかることが多いため、低スペックの PC の場合はある程度点を間引いておいた方が都合が良いことがあります。
`VoxelGrid`フィルタは等間隔に点群をダウンサンプリングします。
引き続き`rsj_pointcloud_test_node.cpp`を編集します。
プログラム冒頭に`include`文を追記してください。

```c++
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>  // 追記
#include <visualization_msgs/MarkerArray.h>
typedef pcl::PointXYZ PointT;
```

`RsjPointcloudTestNode`クラスの冒頭に、`pcl::VoxelGrid`フィルタのインスタンスを追加します。
また、フィルタの結果を格納するための`PointCloud`型変数`cloud_passthrough_`、および処理結果を`publish`するためのパブリッシャ`pub_voxel_`を追加します。

```c++
class RsjPointcloudTestNode
{
private:
略
  ros::Publisher pub_passthrough_;
  // 以下を追記
  pcl::VoxelGrid<PointT> voxel_;
  PointCloud::Ptr cloud_voxel;
  ros::Publisher pub_voxel_;
```

`RsjPointcloudTestNode`クラスのコンストラクタで`VoxelGrid`フィルタの設定、`cloud_voxel`および`pub_voxel_`を初期化します。

```c++
RsjPointcloudTestNode()
{
略
  pub_passthrough_ = pnh_.advertise<PointCloud>("passthrough", 1);
  // 以下を追記
  voxel_.setLeafSize(0.025f, 0.025f, 0.025f);  // 0.025 m 間隔でダウンサンプリング
  cloud_voxel.reset(new PointCloud());
  pub_voxel_ = pnh_.advertise<PointCloud>("voxel", 1);
}
```

`cbPoints`関数を次のように変更します。

```c++
void cbPoints(const PointCloud::ConstPtr &msg)
{
  try{
略
    pub_passthrough_.publish(cloud_passthrough_);
    // 以下のように追記・修正
    voxel_.setInputCloud(cloud_passthrough_);
    voxel_.filter(*cloud_voxel);
    pub_voxel_.publish(cloud_voxel);
    ROS_INFO("points (src: %zu, paththrough: %zu, voxelgrid: %zu)", msg->size(), cloud_passthrough_->size(), cloud_voxel->size());
    // 追記・修正箇所ここまで
  }catch (std::exception &e){
    ROS_ERROR("%s", e.what());
  }
}
```

## ビルド＆実行

`PassThrough`フィルタのときと同様にビルドして実行してください。

## フィルタ実行結果の可視化

RViz でフィルタ実行後の点群の様子を可視化します。
`rsj_pointcloud_test_node`を起動したまま、新しいターミナルを開き、 RViz を起動します。

### Xtion PRO Live の場合

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_test/config/rviz
$ rviz -d view_filters.rviz
```

### YVT-35LX の場合 の場合

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_test/config/rviz
$ rviz -d view_filters_3durg.rviz
```

RViz の左にある`PointCloud2`の一番下のチェックだけをONにすると`VoxelGrid`フィルタ実行後の点群だけが表示されます。

![XtionPointsVoxel](images/xtion_view_voxel.png)

`PassThrough`実行後の結果と比較すると点がまばらになっていることが分かると思います。
もし違いがわかりにくい場合は`setLeafSize`関数の引数を

```c++
RsjPointcloudTestNode()
{
略
  voxel_.setLeafSize(0.05f, 0.05f, 0.05f);  // LeafSize 変更
```

のように大きくしてみてください（確認後は元の値に戻しておいてください）。

# クラスタリング

点群のクラスタリング（いくつかの塊に分離すること）により物体認識などをする際の物体領域候補が検出できます。
プログラム冒頭に`include`文を追記してください。

```c++
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>  // 追記
#include <pcl/kdtree/kdtree.h>  // 追記
#include <pcl/segmentation/extract_clusters.h>  // 追記
#include <visualization_msgs/MarkerArray.h>
typedef pcl::PointXYZ PointT;
```

`RsjPointcloudTestNode`クラスの冒頭に、`pcl::search::KdTree`クラスのポインタ、`pcl::EuclideanClusterExtraction`クラスのインスタンス、検出されたクラスタの可視化情報をパブリッシュする `pub_cluster`を追加します。

```c++
class RsjPointcloudTestNode
{
private:
略
  ros::Publisher pub_voxel_;
  // 以下を追記
  pcl::search::KdTree<PointT>::Ptr tree_;
  pcl::EuclideanClusterExtraction<PointT> ec_;
  ros::Publisher pub_clusters_;
```

`RsjPointcloudTestNode`クラスのコンストラクタで`pcl::EuclideanClusterExtraction`の設定、`tree_`、`pub_clusters_`の初期化をします。

### Xtion PRO Live の場合

```c++
RsjPointcloudTestNode()
{
略
  pub_voxel_ = pnh_.advertise<PointCloud>("voxel", 1);
  // 以下を追記
  tree_.reset(new pcl::search::KdTree<PointT>());
  ec_.setClusterTolerance(0.15);
  ec_.setMinClusterSize(100);
  ec_.setMaxClusterSize(5000);
  ec_.setSearchMethod(tree_);
  pub_clusters_ = pnh_.advertise<visualization_msgs::MarkerArray>("clusters", 1);
}
```

### YVT-35LX の場合

```c++
RsjPointcloudTestNode()
{
略
  pub_voxel_ = pnh_.advertise<PointCloud>("voxel", 1);
  // 以下を追記
  tree_.reset(new pcl::search::KdTree<PointT>());
  ec_.setClusterTolerance(0.15);
  ec_.setMinClusterSize(5);
  ec_.setMaxClusterSize(5000);
  ec_.setSearchMethod(tree_);
  pub_clusters_ = pnh_.advertise<visualization_msgs::MarkerArray>("clusters", 1);
}
```

`pcl::EuclideanClusterExtraction`の設定部分のプログラムは次のとおりです。

`ec_.setClusterTolerance(0.15);`
: 15cm以上離れていれば別のクラスタだとみなす

`ec_.setMinClusterSize(100); ec_.setMaxClusterSize(5000);`
: クラスタを構成する点の数は最低でも100個、最高で5000個

`ec_.setSearchMethod(tree_);`
: ある点とクラスタを形成可能な点の探索方法としてKD木を使用する。

`cbPoints`関数を次のように変更します。

```c++
void cbPoints(const PointCloud::ConstPtr &msg)
{
  try
  {
      略
    pub_voxel_.publish(cloud_voxel);
    // 以下のように追記・修正
    std::vector<pcl::PointIndices> cluster_indices;
    tree_->setInputCloud(cloud_voxel);
    ec_.setInputCloud(cloud_voxel);
    ec_.extract(cluster_indices);
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(), it_end = cluster_indices.end(); it != it_end; ++it, ++marker_id)
    {
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*cloud_voxel, *it, min_pt, max_pt);
      Eigen::Vector4f cluster_size = max_pt - min_pt;
      if (cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0)
      {
        marker_array.markers.push_back(make_marker(frame_id, "cluster", marker_id, min_pt, max_pt, 0.0f, 1.0f, 0.0f, 0.2f));
      }
    }
    if (marker_array.markers.empty() == false)
    {
      pub_clusters_.publish(marker_array);
    }
    ROS_INFO("points (src: %zu, paththrough: %zu, voxelgrid: %zu, cluster: %zu)", msg->size(), cloud_passthrough_->size(), cloud_voxel->size(), cluster_indices.size());
    // 追記・修正箇所ここまで
  }
  catch (std::exception &e)
  {
    ROS_ERROR("%s", e.what());
  }
}
```

## ビルド＆実行

`VoxelGrid`フィルタのときと同様にビルドして実行してください。

## フィルタ実行結果の可視化

RViz でフィルタ実行後の点群の様子を可視化します。
`rsj_pointcloud_test_node`を起動したまま新しいターミナルを開き、 RViz を起動します。

### Xtion PRO Live の場合

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_test/config/rviz
$ rviz -d view_filters.rviz
```

### YVT-35LX の場合 の場合

```shell
$ cd ~/catkin_ws/src/rsj_pointcloud_test/config/rviz
$ rviz -d view_filters_3durg.rviz
```

RViz の左にある`PointCloud2`の一番下のチェックだけを ON にすると`VoxelGrid`フィルタ実行後の点群だけが表示されます。
さらにクラスタリング結果が半透明の緑の BOX で表示されているのが分かります。
これはプログラム中でクラスタリング結果を RViz が可視化可能な型である `visualization_msgs::MarkerArray`に変換してパブリッシュしているからです。

![XtionClusters](images/xtion_view_clusters.png)

# 特定の条件に合致するクラスタを検出する

検出したクラスタのうち、一定の大きさをもつものだけを抽出するようにしましょう。
最終的にはゴミ箱や人間の足など、特定の大きさなど何らかの条件を満たすクラスタに向かって走行するように制御します。

`cbPoints`関数を次のように変更します。
```c++
void cbPoints(const PointCloud::ConstPtr &msg)
{
  try
  {
      略
    pub_voxel_.publish(cloud_voxel);
    std::vector<pcl::PointIndices> cluster_indices;
    tree_->setInputCloud(cloud_voxel);
    ec_.setInputCloud(cloud_voxel);
    ec_.extract(cluster_indices);
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    /*  */
    size_t ok = 0;  // 追記
    /*  */
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(), it_end = cluster_indices.end(); it != it_end; ++it, ++marker_id)
    {
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*cloud_voxel, *it, min_pt, max_pt);
      Eigen::Vector4f cluster_size = max_pt - min_pt;
      if (cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0)
      {
        // 以下を追記・修正
        bool is_ok = true;
        if (cluster_size.x() < 0.05 || cluster_size.x() > 0.4)
        {
          is_ok = false;
        }
        else if (cluster_size.y() < 0.05 || cluster_size.y() > 0.6)
        {
          is_ok = false;
        }
        else if (cluster_size.z() < 0.05 || cluster_size.z() > 0.5)
        {
          is_ok = false;
        }
        visualization_msgs::Marker marker = make_marker(frame_id, "cluster", marker_id, min_pt, max_pt, 0.0f, 1.0f, 0.0f, 0.2f);
        if (is_ok)
        {
          marker.ns = "ok_cluster";
          marker.color.r = 1.0f;
          marker.color.g = 0.0f;
          marker.color.b = 0.0f;
          marker.color.a = 0.5f;
          ok++;
        }
        marker_array.markers.push_back(marker);
        // 追記・修正箇所ここまで
      }
    }
    if (marker_array.markers.empty() == false)
    {
      pub_clusters_.publish(marker_array);
    }
    /*** 修正 ***/
    ROS_INFO("points (src: %zu, paththrough: %zu, voxelgrid: %zu, cluster: %zu, ok_cluster: %zu)", msg->size(), cloud_passthrough_->size(), cloud_voxel->size(), cluster_indices.size(), ok);
    /*** 修正 ***/
  }
  catch (std::exception &e)
  {
    ROS_ERROR("%s", e.what());
  }
}
```

## ビルド＆実行

クラスタリングのときと同様にビルドして実行してください。

## フィルタ実行結果の可視化

クラスタリングのときと同様に RViz で可視化してください。ある一定の大きさのクラスタだけを赤く表示しているのが分かります。

![XtionClusters](images/xtion_view_specific_clusters.png)

# 最も近いクラスタを検出する

前項で抽出したクラスタのうち、センサに最も近いクラスタを選択するようにしましょう。

`cbPoints`関数を次のように変更します。
```c++
void cbPoints(const PointCloud::ConstPtr &msg)
{
  try
  {
      略
    pub_voxel_.publish(cloud_voxel);
    std::vector<pcl::PointIndices> cluster_indices;
    tree_->setInputCloud(cloud_voxel);
    ec_.setInputCloud(cloud_voxel);
    ec_.extract(cluster_indices);
    visualization_msgs::MarkerArray marker_array;
    /*  */
    int target_index = -1;  // 追記
    /*  */
    int marker_id = 0;
    size_t ok = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(), it_end = cluster_indices.end(); it != it_end; ++it, ++marker_id)
    {
          略
        if (is_ok)
        {
          marker.ns = "ok_cluster";
          marker.color.r = 1.0f;
          marker.color.g = 0.0f;
          marker.color.b = 0.0f;
          marker.color.a = 0.5f;
          ok++;
          // 以下のように追記
          if(target_index < 0){
            target_index = marker_array.markers.size();
          }else{
            float d1 = ::hypot(marker_array.markers[target_index].pose.position.x, marker_array.markers[target_index].pose.position.y);
            float d2 = ::hypot(marker.pose.position.x, marker.pose.position.y);
            if(d2 < d1){
              target_index = marker_array.markers.size();
            }
          }
          // 追記箇所ここまで
        }
        marker_array.markers.push_back(marker);
      }
    }
    if (marker_array.markers.empty() == false)
    {
      // 以下のように追記
      if(target_index >= 0){
        marker_array.markers[target_index].ns = "target_cluster";
        marker_array.markers[target_index].color.r = 1.0f;
        marker_array.markers[target_index].color.g = 0.0f;
        marker_array.markers[target_index].color.b = 1.0f;
        marker_array.markers[target_index].color.a = 0.5f;
      }
      // 追記箇所ここまで
      pub_clusters_.publish(marker_array);
    }
    ROS_INFO("points (src: %zu, paththrough: %zu, voxelgrid: %zu, cluster: %zu, ok_cluster: %zu)", msg->size(), cloud_passthrough_->size(), cloud_voxel->size(), cluster_indices.size(), ok);
  }
  catch (std::exception &e)
  {
    ROS_ERROR("%s", e.what());
  }
}
```

## ビルド＆実行

前項と同様にビルドして実行してください。

## フィルタ実行結果の可視化

前項と同様に RViz で可視化してください。ある一定の大きさのクラスタだけを赤く表示し、その中でセンサに最も近いクラスタを紫で表示しているのが分かります。

![XtionClusters](images/xtion_view_target_cluster.png)
