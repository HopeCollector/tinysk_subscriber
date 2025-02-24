
# 🐍Tiny Snake Subscriber

## 项目简介

本项目为微型生命探测机器人项目的子项目，目的在于驱动传感器并将其通过网络打包发送会远程地面站。项目设计的机器人外观为 蛇型机器人，蛇头位置配备摄像头、激光雷达、IMU三种传感器设备。

通信的总体架构为发布者-订阅者架构，机器人为发布者，采集数据后进行发布，地面站是订阅者，负责接收消息。由于项目整体限制传输带宽，因此本项目除 IMU 之外的所有数据都进行了压缩处理。点云进行了一定比例的降采样，视频进行使用硬件加速的 jpeg 编码。

机器人使用的开发板型号为 **RaspberryPi Zero 2W**，使用基于 aarch64(armv8) Debian12(bookwarm) 的树莓派定制操作系统

本项目为**接收端**，发送端参见 https://github.com/HopeCollector/tinysk_publisher

## 环境配置

使用 pip 安装依赖：

```sh
pip install -r requirements.txt
```

## 运行

使用以下指令运行项目：

```sh
python3 main.py
```

## 通信协议

通信协议使用 capnp 序列化工具，通信协议的详细内容在 `messages` 文件夹中
