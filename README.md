matplotlib参考：
https://github.com/lava/matplotlib-cpp
https://mangoroom.cn/cpp/call-matplotlib-on-cpp.html
https://blog.csdn.net/qq_31253399/article/details/115342900
如果不用cmake编译：g++ -o graph_slam ./*.cpp ./matplotlibcpp.h -I /usr/include/python2.7 -l python2.7

使用三种实现方式完成图优化：（1）从Eigen开始手写GN（2）g2o（3）Ceres

优化结果：（手写GN，其他结果见result文件夹）
![GN](https://user-images.githubusercontent.com/42105276/189851829-5c4a6039-c3c8-4996-a11c-370436948042.png)

![对比](https://user-images.githubusercontent.com/42105276/189851839-a8bd2fe9-350e-4a64-ab96-de29372f39c5.png)
