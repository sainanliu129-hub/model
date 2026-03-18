# Body_GravityPara_Iden（机身重力参数辨识）

- **principle/**：原理层函数（回归矩阵、辨识、正/逆动力学、预处理等），仅供调用、不直接运行。
- **test/**：可运行脚本（入口、对比、示例、验收），直接打开并运行即可；数据与结果使用本目录下 `data/`、`build/`。
- **data/**：所需数据（如跑步 CSV、多关节采集 CSV）。
- **build/**：生成的轨迹 CSV、MAT 等。
- **docs/**：方案与流程文档（辨识方案、预处理流程、梳理说明等）。

常用入口（在 `test/` 下）：`run_min_param_id_from_csv`、`run_full_dynamics_validation`、`run_debug_pi_phys_id_fd_plot`、`run_torque_comparison`、`example_run_min_param_id`、`test_ReMatrix_E1_limb_URDF`。

摩擦与转子惯量补偿：辨识前可先做 `Friction_Iden`，在 `run_min_param_id_from_csv(..., opts)` 中设置 `opts.friction_params` 和 `opts.I_a`，使用 τ_rb = τ_meas − τ_friction − τ_rotor。原理与动力学见仓库根目录 `dynamics/`、`robot_model/`。共用工具在 **utility_function/**，脚本会通过 `ensure_body_gravity_para_iden_path()` 自动加路径。
