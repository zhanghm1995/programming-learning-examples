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
**注：** 编译过程必须加入-static构建静态可执行文件，否则在运行时会出现错误：
```shell
standard_init_linux.go:211: exec user process caused "no such file or directory"
```
原因：
由于该应用从scratch基础镜像构建而来，而scratch镜像中不包含任何程序运行需要的动态链接库，可以从ubuntu镜像作为base image来构建，就没问题