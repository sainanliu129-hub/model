# E1 人形机器人模型与算法（独立仓库）

本仓库仅包含与 **E1 人形机器人** 相关的代码，与 Legacy 机械臂仓库互不依赖。

## 目录结构

```
model_e1/
├── addpaths.m              % 一键添加所有模块到 MATLAB 路径
├── applications/           % 各功能模块的调用示例，每模块一个文件夹
│   ├── Body_GravityPara_Iden/   % 机身重力参数辨识
│   │   ├── data/               % 所需数据文件
│   │   ├── build/              % 生成的文件（轨迹、结果等）
│   │   └── *.m                 % 入口脚本与示例
│   ├── Friction_Iden/          % 关节摩擦辨识（E1 腿）
│   │   ├── data/
│   │   ├── build/
│   │   └── *.m
│   └── Trajectory/             % 腿部轨迹生成与读取示例
│       ├── data/
│       ├── build/
│       └── *.m
├── plan/                  % 轨迹生成原理与实现（E1 腿 12 关节）
├── dynamics/              % E1 动力学（RigidBodyTree）
├── robot_model/           % E1 机器人模型（get_e1_*）
├── utility_function/      % 通用工具（路径、E1 关节名、读 CSV 等）
├── model/                 % 配置文件（如 TOML）
├── noetix_description/    % E1 描述（urdf/ 下为 E1.urdf 等）
├── data/                  % 项目级数据目录（可选）
└── build/                 % 项目级生成目录（可选）
```

## 使用说明

1. 在 MATLAB 中 `cd` 到本仓库根目录，执行 `addpaths`。
2. 各应用入口在 `applications/<模块名>/` 下，如：
   - 重力参数辨识：`applications/Body_GravityPara_Iden/run_generate_e1_test_trajectories.m`
   - 摩擦辨识：`applications/Friction_Iden/run_full_iden_three_files.m`
   - 轨迹示例：`applications/Trajectory/` 下示例脚本。
3. 数据放在各模块的 `data/` 或根目录 `data/`，生成结果写入各模块的 `build/` 或根目录 `build/`。

## 依赖

- MATLAB（含 Robotics System Toolbox，用于 RigidBodyTree）。
- 本仓库自包含，不依赖 model_legacy 或原 model_matlab_backup。
