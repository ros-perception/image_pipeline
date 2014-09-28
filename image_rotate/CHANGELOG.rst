1.12.10 (2014-09-28)
--------------------

1.12.9 (2014-09-21)
-------------------

1.12.8 (2014-08-19)
-------------------

1.12.6 (2014-07-27)
-------------------

1.12.4 (2014-04-28)
-------------------

1.12.3 (2014-04-12)
-------------------

1.12.2 (2014-04-08)
-------------------
* use NODELET_** macros instead of ROS_** macros
* use getNodeHandle rather than getPrivateNodeHandle
* add executable to load image_rotate/image_rotate nodelet.
  add xml file to export nodelet definition.
  Conflicts:
  image_rotate/package.xml
* make image_rotate nodelet class
  Conflicts:
  image_rotate/CMakeLists.txt
  image_rotate/package.xml
  image_rotate/src/nodelet/image_rotate_nodelet.cpp
* move image_rotate.cpp to nodelet directory according to the directory convenstion of image_pipeline
* Contributors: Ryohei Ueda

1.12.1 (2014-04-06)
-------------------
* replace tf usage by tf2 usage
