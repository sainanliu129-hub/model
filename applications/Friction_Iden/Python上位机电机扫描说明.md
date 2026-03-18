# 电机扫描（串口转 CAN）

用串口转 CAN 桥与电机通信，做电流扫描、速度扫描、加速度扫描并保存 CSV，供本目录 MATLAB 摩擦/惯量辨识使用。

## 依赖

```bash
pip install -r requirements.txt
```

## 文件说明

| 文件 | 说明 |
|------|------|
| `serial_can_bridge.py` | 串口转 CAN 协议（rapoo 帧格式），被各扫描脚本依赖 |
| `scan_current_serial.py` | 电流扫描：逐级加电流，检测电机开始转动（静摩擦） |
| `scan_speed_levels.py` | 速度扫描：按固定转速等级正反转，记录反馈（库伦/粘滞摩擦） |
| `scan_accel_serial.py` | 加速度扫描：不同加速度等级 (rpm/s)，每档 10s 速度斜坡（转动惯量 I_a） |

## 核心流程

**电流扫描 `scan_current_serial.py`**

1. 输入：目标转速、最大电流。
2. 生成电流档位：0～0.6 A 步长 0.1，0.61～最大 步长 0.01。
3. 正向：从 0 起逐档加电流，固定目标转速；每档保持 7 s，周期发速度指令并收反馈写 CSV；若**连续 4 s 转速 > 1 rpm** 认为已转动，停止正向。
4. 反向：同逻辑，目标转速取负。
5. 输出：CSV（时间戳、方向、目标转速、电流指令、实际转速、电流、温度）。

**速度扫描 `scan_speed_levels.py`**

1. 输入：最大转速、电流阈值。
2. 按表取 ≤ 最大转速的等级，得到正转档位列表。
3. 顺序执行：0 → 正转各档 → 0 → 反转各档 → 0；每档保持 10 s，周期发速度指令（目标转速 + 电流阈值），收反馈写 CSV、每档打印一次。
4. 输出：CSV（时间戳、方向、目标转速、实际转速、电流、温度）。

**加速度扫描 `scan_accel_serial.py`**

1. 输入：最大转速（斜坡上限）、电流阈值。
2. 加速度等级（默认 1, 2, 5, 10, 20, 50 rpm/s）；协议无加速度指令，用**目标速度 = 加速度 × 时间**的斜坡模拟。
3. 正向：按等级依次跑，每档 10 s 内目标速度从 0 线性增至 min(加速度×10, 最大转速)，周期发指令并写 CSV。
4. 反向：同逻辑，目标速度为负。
5. 输出：CSV（时间戳、方向、accel_rpm_s、目标转速、实际转速、电流、温度）。

## 使用前

1. 把串口转 CAN 桥接到电脑，记下 COM 口（如 COM5）。
2. 在脚本里改 `SERIAL_PORT`（各脚本中均有），波特率默认 115200。

## 运行

- 电流扫描：`python scan_current_serial.py`，按提示输入最大电流 (A)。
- 速度扫描：`python scan_speed_levels.py`，按提示输入最大转速 (rpm) 和电流阈值 (A)。
- 加速度扫描：`python scan_accel_serial.py`，按提示输入最大转速 (rpm) 和电流阈值 (A)；每档加速度 10 s。

结果 CSV 会保存在当前目录，文件名带时间戳。

---

## 与 MATLAB 摩擦/惯量辨识的对应

辨识时**输入 3 个文件**，分别得到**静摩擦 τ_s、库伦/粘滞 τ_c 与 b、转动惯量 I_a**：

| 顺序 | Python 脚本 | CSV 列（示例表头） | 列数 | MATLAB 得到 |
|------|-------------|--------------------|------|-------------|
| 1 | **scan_current_serial.py** | timestamp_s, direction, target_speed_rpm, command_current_A, actual_speed_rpm, phase_current_A, temp_degC | 7 | **τ_s**（静摩擦） |
| 2 | **scan_speed_levels.py** | timestamp_s, direction, target_speed_rpm, actual_speed_rpm, phase_current_A, temp_degC | 6 | **τ_c、b**（库伦、粘滞） |
| 3 | **scan_accel_serial.py** | timestamp_s, direction, accel_rpm_s, target_speed_rpm, actual_speed_rpm, phase_current_A, temp_degC | 7 | **I_a**（转动惯量，需先用 2 得到 τ_c、b） |

**使用**：把三个 CSV 放到 `data/friction_iden/`，在 MATLAB 中改 `run_full_iden_three_files.m` 里的 `file_static`、`file_velocity`、`file_accel`、`K_tau`，运行一次即可得到 τ_s、τ_c、b、I_a。

MATLAB 读取时自动跳过表头与 `direction` 等文本列，只取：时间、实际转速(rpm)、电流(phase_current_A)。
