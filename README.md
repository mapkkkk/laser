# visual_commue

## 项目简介：
树莓派程序，作为MCU上位机进行任务主控和视觉处理

## 项目说明：
* ProtocolMCU - 完成与飞控MCU的串口通信，实现树莓派的完全控制
  * Serial.py - 底层串口通信机制
  * Base.py - 基于串口的数据交换
  * Protocol.py - 基于基本数据交换建立基本功能实现
  * Application.py - 高级功能实现
* RadarDrive - 激光雷达驱动
* Vision - 机器视觉
## 使用方法：
manager作为主进程运行，部署在树莓派上
若是本地模式则此电脑便没有用了
远程模式需在此电脑上启动远程脚本

## TODO

* remote：远程控制系统
* mission1：植保无人机
* vision_onnx：部署问题
* vision_cv：代码积累

## 致谢：
本项目基于ElluIFX / ANO_LX_FC项目重构，没有学长帮助，真不知何时才能完成
原项目地址：

[ElluIFX / ANO_LX_FC](https://github.com/ElluIFX/ANO_LX_FC)

大家快去fork一下!

