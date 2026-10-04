# QMI8658A ESP-IDF component

This small C port adapts the QMI8658 register configuration, CTRL9 command
handshake and AHB clock-gating logic from
[lewisxhe/SensorLib](https://github.com/lewisxhe/SensorLib/tree/477fc682e9ed30ced39774f3b7e93a1504968884),
commit `477fc682e9ed30ced39774f3b7e93a1504968884` (version 0.5.0).
Upstream files: `src/sensor/imu/qmi8658/SensorQMI8658.cpp` and
`SensorQMI8658_Reg.hpp`. The original MIT license is preserved in `LICENSE`.

Uses ESP-IDF 5.5 synchronous `i2c_master` APIs. The caller owns the bus and
serializes access to the sensor handle. Addresses 0x6A and 0x6B are checked
against WHO_AM_I 0x05. SA0 high selects 0x6A on QMI8658A.

Default configuration: 400 kHz I2C, +/-4 g, +/-512 degrees/s, ODR code 6
(about 112.1 Hz in six-axis mode), FIFO bypass, SyncSample enabled and AHB
clock gating disabled. Interrupt outputs remain high impedance; QMI8658A
normal data-ready signals are routed to INT2, not INT1.

Reading STATUSINT locks an available sample. If locking is still in progress,
the driver waits 12 microseconds, then reads all six signed little-endian
values in one 12-byte burst ending at GZ_H, which releases the lock. On a
failed transfer it attempts to release the lock and returns the original
error. Outputs are committed only after a complete successful read.

Compared with the upstream code, this port propagates all I2C errors,
rejects CTRL9 completion timeouts before acknowledgement, and uses finite
wall-clock deadlines. Reset readiness and sensor-enable configuration are
verified. For register behavior see the QST
[QMI8658A datasheet Rev A](https://files.waveshare.com/wiki/common/QMI8658A_Datasheet_Rev_A.pdf),
sections 5.5, 5.10, 6, 13 and 16.3.
