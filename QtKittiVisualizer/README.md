QtKittiVisualizer
=================

Description
-----------

The **QtKittiVisualizer** allows viewing the 3D point clouds and bounding boxes of the [raw KITTI data sets](http://www.cvlibs.net/datasets/kitti/raw_data.php).  It is a tool based on *Qt*, the *PCL* and the *VTK*.

It includes the *C++* part of the [raw data development kit](http://kitti.is.tue.mpg.de/kitti/devkit_raw_data.zip) provided on the [official KITTI website](http://www.cvlibs.net/datasets/kitti/).


Dependencies
-----------
It is a tool based on *Qt*, the *PCL* and the *VTK*.
Please refer http://blog.sina.com.cn/s/blog_1496fa80e0102ybq5.html to install.


Data Structure
-----------
```
./
    kitti/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data
    QtKittiVisualizer/
```

Use
-----------
```
cmake .
make
./qt-kitti-visualizer --help  or
./qt-kitti-visualizer --dataset 0 (Data number should exist in dataset.)
```

License
-------

The new parts of the software are licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).
