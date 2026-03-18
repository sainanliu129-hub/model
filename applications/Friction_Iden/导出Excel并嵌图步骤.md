# 导出「静摩擦 / 动摩擦 / 转动惯量」Excel 并嵌图 — 具体步骤

适用于 **Linux + LibreOffice Calc**（无需 Excel）。

---

## 一、一次性环境准备

### 1. 安装 Python 3（若未安装）

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install python3 python3-pip

# 检查
python3 --version
```

### 2. 安装嵌图依赖

```bash
pip3 install openpyxl Pillow
```

或使用当前用户安装（不需 sudo）：

```bash
pip3 install --user openpyxl Pillow
```

### 3. 确认 MATLAB 能调用系统 Python

在终端执行：

```bash
python3 -c "from openpyxl import load_workbook; from openpyxl.drawing.image import Image; print('ok')"
```

若输出 `ok` 说明环境就绪。

---

## 二、每次辨识并导出 Excel

**只需在 MATLAB 里运行主脚本**：脚本跑完后会**自动调用**同目录下的 `embed_figures_xlsx.py` 把图嵌进 xlsx，**不用单独执行 .py 文件**。只要本机已装好 Python 和 openpyxl/Pillow（见“一、一次性环境准备”），嵌图会在 MATLAB 里一气完成。

### 1. 在 MATLAB 中配置数据与参数

打开并编辑 `run_full_iden_three_files.m`：

- 设置 **数据目录** 与 **三个 CSV 路径**（电流扫描、速度扫描、加速度扫描），例如：

  ```matlab
  dd = get_data_dir('test_data/泉智博_100-20-10 26_3_6');
  file_static   = fullfile(dd, 'scan_torque_serial_mit_20260309_192539.csv');
  file_velocity = fullfile(dd, 'scan_result_mit_20260306_154134.csv');
  file_accel    = fullfile(dd, 'scan_accel_serial_mit_20260306_161602.csv');
  ```

- 设置 **K_tau**、**FrictionModel**（如 `'tanh_viscous'`）、**VarianceSpeedPct** 等（按需）。

### 2. 运行主脚本

在 MATLAB 命令窗口执行：

```matlab
cd('applications/Friction_Iden')   % 或你的实际路径
run_full_iden_three_files
```

或在该目录下直接点“运行”运行 `run_full_iden_three_files.m`。

### 3. 查看结果

- **单台电机 Excel**：`build/<电机名>_静摩擦_动摩擦_转动惯量.xlsx`
- **全部电机汇总表**：`build/全部电机参数汇总.xlsx`（每次辨识后自动更新，每台电机只保留最新一行参数）
- **附图 PNG**：同目录下 `build/` 中的 `静摩擦辨识图_xxx.png`、`速度拟合图_xxx.png` 等。

若嵌图成功，命令行会提示：**附图已嵌入各表（Python openpyxl）**。

### 4. 用 LibreOffice Calc 打开

```bash
libreoffice --calc "applications/Friction_Iden/build/<电机名>_静摩擦_动摩擦_转动惯量.xlsx"
```

或在文件管理器中双击该 xlsx，用 Calc 打开即可看到三张表。

**说明**：用 openpyxl 嵌入的图片在 **LibreOffice Calc 中可能不显示**（已知兼容问题），但同一 xlsx 在 **Microsoft Excel** 或 **Google Sheets** 中打开通常能正常看到图。若只用 Calc，可直接查看 `build/` 目录下对应的 PNG 图（静摩擦辨识图、速度拟合图等）。

---

## 三、仅重新嵌图（已有 xlsx 和 PNG）

若已生成 xlsx 和 build 下的 PNG，只想重新嵌图，可在终端执行：

```bash
cd applications/Friction_Iden
python3 embed_figures_xlsx.py "build/<电机名>_静摩擦_动摩擦_转动惯量.xlsx" "build" "<电机名>"
```

将 `<电机名>` 换成实际名称（与 xlsx/PNG 文件名中的一致）。

---

## 四、常见问题

| 现象 | 处理 |
|------|------|
| 提示「附图未嵌入」 | 执行 `pip3 install openpyxl Pillow` 后重跑 |
| `python3` 找不到 | 安装 Python 3 或将命令改为 `python`（若系统默认是 Python 3） |
| MATLAB 中 `system('python3 ...')` 用的不是已装 openpyxl 的 Python | 在终端用 `which python3` 确认路径，或在 MATLAB 里用 `system(['/usr/bin/python3 ', ...])` 指定完整路径 |
| 不想嵌图，只要数据表 | 在 `run_full_iden_three_files.m` 里给 `export_motor_spec_to_excel` 增加参数 `'EmbedFigures', false` |
| Calc 打开 xlsx 只看到路径/没有图 | 嵌图在 Calc 中可能不显示，可用 **Excel** 或 **Google Sheets** 打开同一文件看图，或直接看 `build/` 下 PNG |
| 需要单独运行 .py 吗？ | **不需要**。只运行 MATLAB 主脚本即可，脚本结束前会自动调用 `embed_figures_xlsx.py` 嵌图 |

---

## 五、全部电机参数汇总表 — 列说明

`build/全部电机参数汇总.xlsx` 中「全部电机参数汇总」表各列含义如下：

| 列名 | 含义 | 单位/说明 |
|------|------|-----------|
| **电机型号** | 电机标识（通常为数据目录名） | — |
| **辨识日期** | 该电机参数被辨识的时间 | yyyy-mm-dd HH:MM |
| **K_tau_Nm/A** | 力矩系数：电流与力矩比例 | N·m/A |
| **摩擦模型** | 所用摩擦模型 | 如 tanh_viscous、coulomb_viscous、stribeck_viscous |
| **tau_s_Nm** | 静摩擦扭矩（电流扫描静止段得到） | N·m |
| **mu_s_Nm** | tanh 模型：等效库伦幅值（τ_f 中 tanh 项系数） | N·m |
| **v_act_rads** | tanh 模型：特征速度，tanh 过渡快慢 | rad/s |
| **mu_d_Nms_rad** | tanh 模型：粘滞系数（τ_f 中线性项系数） | N·m·s/rad |
| **tau_c_pos_Nm** | 库伦摩擦（正速度方向）；coulomb 模型用；tanh 时多为空或与 μ_s 一致 | N·m |
| **tau_c_neg_Nm** | 库伦摩擦（负速度方向） | N·m |
| **b_Nms_rad** | 粘滞系数；coulomb/stribeck 模型用；tanh 时多为空或与 μ_d 一致 | N·m·s/rad |
| **I_a_kgm2** | 转动惯量（加速度扫描拟合得到） | kg·m² |
| **I_a_R2** | 惯量拟合优度 R²（越接近 1 越好） | 无量纲 |
| **R2_friction** | 摩擦拟合优度 R² | 无量纲 |
| **RMSE_Nm** | 摩擦拟合均方根误差 | N·m |
| **nUsed** | 摩擦辨识所用有效速度档点数 | — |
| **备注** | 额外说明（如 stribeck 的 τ_0 等） | — |
