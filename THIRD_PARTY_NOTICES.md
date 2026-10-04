# 第三方来源与授权

本工程 QMI8658A 驱动的寄存器定义、初始化、同步读取和单位换算参考并移植自 **lewisxhe/SensorLib**，固定提交：

`477fc682e9ed30ced39774f3b7e93a1504968884`

- 仓库：[https://github.com/lewisxhe/SensorLib](https://github.com/lewisxhe/SensorLib)
- 驱动实现：[src/sensor/imu/qmi8658/SensorQMI8658.cpp](https://github.com/lewisxhe/SensorLib/blob/477fc682e9ed30ced39774f3b7e93a1504968884/src/sensor/imu/qmi8658/SensorQMI8658.cpp)
- 驱动接口：[src/sensor/imu/qmi8658/SensorQMI8658.hpp](https://github.com/lewisxhe/SensorLib/blob/477fc682e9ed30ced39774f3b7e93a1504968884/src/sensor/imu/qmi8658/SensorQMI8658.hpp)
- 寄存器定义：[src/sensor/imu/qmi8658/SensorQMI8658_Reg.hpp](https://github.com/lewisxhe/SensorLib/blob/477fc682e9ed30ced39774f3b7e93a1504968884/src/sensor/imu/qmi8658/SensorQMI8658_Reg.hpp)
- 仓库授权：[LICENSE](https://github.com/lewisxhe/SensorLib/blob/477fc682e9ed30ced39774f3b7e93a1504968884/LICENSE)

上述 QMI8658 源文件声明 `Copyright (c) 2026 lewis he`；仓库 LICENSE 声明 `Copyright (c) 2022 lewis he`。本工程保留两项版权声明与 MIT 授权文本。移植改动包括使用 ESP-IDF 5.5.5 新 I²C 主机接口，以 C 实现当前板子需要的初始化和六轴读取，并增加错误检查。

```text
MIT License

Copyright (c) 2022 lewis he
Copyright (c) 2026 lewis he

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
