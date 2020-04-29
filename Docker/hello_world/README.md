## 介绍
参考链接：
https://linuxhint.com/create_docker_image_from_scratch/

构建镜像：
```shell
g++ -o hello -static hello.cc
docker build --tag hello .
```
查看镜像：
```shell
docker images
```
运行：
```shell
docker run hello
```
**注：** 编译过程必须加入-static，否则在运行时会出现错误：
```shell
docker run hello
```
原因：
由于