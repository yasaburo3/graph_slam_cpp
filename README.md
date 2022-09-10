matplotlib参考：
https://github.com/lava/matplotlib-cpp
https://mangoroom.cn/cpp/call-matplotlib-on-cpp.html
https://blog.csdn.net/qq_31253399/article/details/115342900
如果不用cmake编译：g++ -o graph_slam ./*.cpp ./matplotlibcpp.h -I /usr/include/python2.7 -l python2.7

使用三种实现方式完成图优化：（1）从Eigen开始手写GN（2）g2o（3）Ceres
原始数据：
![raw](https://user-images.githubusercontent.com/42105276/189471093-25e4cf19-9afb-43ec-8718-7c7b2361050d.png)
优化结果：（手写GN，其他结果见result文件夹）
![4](https://user-images.githubusercontent.com/42105276/189471115-ac256acd-afb0-47b8-b4ad-e1d214f5f13e.png)
