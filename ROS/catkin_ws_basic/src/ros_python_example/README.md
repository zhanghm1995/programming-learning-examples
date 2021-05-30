## ros_python

书写Python版的ROS包时，可以与C++版的ROS包格式保持一致，即创建CMakeLists和package.xml文件来组织整个Python-ROS包，这样就能仍然使用`rosrun`的命令来启动Python脚本。

其实也可以直接写Python脚本，`import` ROS相关的包，直接调用ROS的一些功能，在运行的时候就直接按照正常的Python脚本运行方式进行运行即可。

## TODO
- [ ] 1. 消息发布与订阅
- [ ] 2. 如何使用自定义消息
- [ ] 3. 如何使用自己的Python环境

