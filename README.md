# ESP32-C2 / QMI8658A 六轴串口演示

基于主控原理图，将 QMI8658A 的三轴加速度和三轴角速度通过 UART0 输出；VS Code 串口监视器每隔 300 ms 收到一行新数据。驱动参考 [lewisxhe/SensorLib](https://github.com/lewisxhe/SensorLib)，固定参考版本 `477fc682e9ed30ced39774f3b7e93a1504968884`，移植为 ESP-IDF 5.5.5 的原生 C 驱动。

## 硬件与默认配置

| 信号 | ESP32-C2 引脚 | 配置 |
| --- | --- | --- |
| QMI8658A SDA | GPIO0 | I²C，400 kHz |
| QMI8658A SCL | GPIO1 | I²C，400 kHz |
| QMI8658A INT1 | MTMS / GPIO4 | 输入，轮询读取，不安装中断服务 |
| MCU_EN | MTCK / GPIO6 | 启动时拉高，保持板上 3.3 V 电源 |
| UART0 TX | GPIO20 | 115200 baud |
| UART0 RX | GPIO19 | 115200 baud |

SDA、SCL 均按实板已具有外部 3.3 V 上拉配置。QMI8658A 的 CS 上拉选择 I²C，SA0 为高，因此先探测 7 位地址 `0x6A`；失败时再尝试 `0x6B`，仅在 `WHO_AM_I == 0x05` 时接受该设备。

加速度量程为 ±4 g，陀螺仪量程为 ±512 °/s，六轴输出数据率为 112.1 Hz。采用同步采样，每次连续读取 12 字节完整六轴数据。输出保留传感器原始 X/Y/Z 方向；`Acc[g]` 单位为 g，`Gyro[dps]` 单位为 °/s。300 ms 是串口显示周期，传感器采样周期由其 ODR 决定。

## 连接烧录转接板

使用《烧录芯片原理图.pdf》中的专用 CP2102N 转接板：电脑连接转接板，转接板连接主板 USB-C。主板 Type-C 的 D+/D− 用于 UART，普通 USB 线直接连接主板与电脑不会生成串口设备。

- 串口必须交叉连接：转接板 TXD → 主控 GPIO19 / U0RXD，转接板 RXD ← 主控 GPIO20 / U0TXD，并共地。
- 转接板图纸将 D− 标为 `flash_RX`（转接板 TXD），D+ 标为 `flash_TX`（转接板 RXD）。两份图纸的同名网络不能单独证明实板已正确交叉连接。
- 转接板同时提供 CHIP_EN、GPIO9 和供电，实现自动进入下载模式；外部供电使板上串口开关选择外部连接。

选用设备管理器中实际对应烧录转接板的 COM 端口。系统中出现的其他 USB 串口设备不代表已经连接本板。

## 编译、烧录与监视

在项目目录的 PowerShell 终端中执行。环境脚本需要用开头的点号进行 **dot-source**，使 ESP-IDF 环境变量保留在当前终端：

```powershell
Set-Location E:\tail_light\esp32c2_two
. .\tools\Activate-Idf.ps1
idf.py build
idf.py -p COMx -b 115200 flash monitor --no-reset
```

将 `COMx` 替换为本板实际端口。脚本使用现有安装 `D:/Espressif/frameworks/esp-idf-v5.5.5`。项目默认目标为 `esp32c2`。退出 IDF Monitor 按 `Ctrl+]`；单独打开监视器可执行 `idf.py -p COMx monitor --no-reset`。

VS Code 中按 `Ctrl+Shift+P`，执行 **Tasks: Run Task**，选择 **ESP-IDF: Monitor QMI8658A demo**，串口默认 `COM11`，即可查看已烧录程序的输出。`Ctrl+Shift+B` 可调用本工程的编译任务。串口监视波特率为 115200。

工程已设置 `idf.monitorNoReset: true`，监视任务也使用 `--no-reset`，避免连接时主动复位。主板依靠 GPIO6 / MCU_EN 保持供电；复位可能使该信号失效，程序尚未重新拉高 GPIO6 就已掉电。若只显示 `ESP-ROM` 或随后没有数据，保持转接板连接、松开 BOOT / 复位键，按住**主板电源键**约 2 秒后松开，再使用上述不复位的监视任务。烧录仍会执行必要的复位，烧录后也可能需要按主板电源键重新开机。`--no-reset` 省略连接后的复位脉冲，但监视器打开串口时仍会设置 DTR / RTS。

也可使用 ESP-IDF 扩展的 Build、Flash、Monitor，目标为 `esp32c2`。若扩展提示找不到 ESP-IDF 安装，在 VS Code **用户设置**中核对：

```json
{
  "idf.currentSetup": "D:/Espressif/frameworks/esp-idf-v5.5.5",
  "idf.eimIdfJsonPath": "D:/Espressif/esp_idf.json"
}
```

`esp_idf.json` 应为该安装的有效安装清单；无需仅为本工程重新安装 SDK。

启动后先显示设备识别及配置结果，随后每 300 ms 输出一行，例如：

```text
t=1200ms Acc[g]=(0.012,-0.023,1.004) Gyro[dps]=(0.140,-0.080,0.030)
```

数据使用换行追加显示。通信或采样失败时查看串口错误信息；失败读取不应作为有效六轴数据使用。

若烧录提示无串口响应，先核对所选 COM 口、供电和下载模式，再核对转接板 TXD 到主控 RX、转接板 RXD 到主控 TX 的实板交叉连接。成功编译不能证明这些硬件连接已通过验证。

## 验证

无需硬件的驱动测试：

```powershell
python tests/run_host_tests.py
```

上板后核对以下现象：

1. 正确识别 QMI8658A，串口连续输出六轴数据，`t` 的正常相邻差值为约 300 ms。
2. 板子静止时加速度向量模长约为 1 g，角速度接近 0 °/s；安装方向决定哪个轴主要承受重力。
3. 绕不同轴转动时，相应角速度轴随转动变化；翻转板子后重力投影方向随之改变。
4. 保持供电时停止或复位程序，再启动仍可识别传感器；I²C 断开时显示错误而不输出无效样本。

本次已完成 ESP-IDF 5.5.5 编译、29 项主机驱动测试，以及 COM11 上的实际烧录和数据采集。静止采集 30 行、转动采集 200 行，相邻设备时间戳间隔均为 300 ms；转动时六轴数值随之变化。静止加速度模长约 0.971 g。

实板原始陀螺仪 X 轴在静止段约为 −12.9 °/s，存在明显零偏，因此“静止角速度接近零”这一项尚未通过。当前驱动输出传感器原始物理量；应用需要更准确角速度时，应进行静止零偏校准。

完整记录见 `artifacts/VALIDATION.md`。本次采集已经关闭串口，VS Code 可直接使用 COM11 打开 Monitor。

也可自行记录并检查 30 行真实数据：

```powershell
python tools/capture_imu.py --port COM11 --count 30 --output artifacts/hardware/serial_check.log
```

采集脚本与 VS Code Monitor 不能同时占用同一串口。

## 开源来源

固定版本的 [QMI8658 驱动](https://github.com/lewisxhe/SensorLib/blob/477fc682e9ed30ced39774f3b7e93a1504968884/src/sensor/imu/qmi8658/SensorQMI8658.cpp) 与[寄存器定义](https://github.com/lewisxhe/SensorLib/blob/477fc682e9ed30ced39774f3b7e93a1504968884/src/sensor/imu/qmi8658/SensorQMI8658_Reg.hpp)提供移植参考。本工程使用 ESP-IDF 新 I²C 主机接口，不要求安装整个 SensorLib 或 Arduino。授权文本和来源见 `THIRD_PARTY_NOTICES.md`。
