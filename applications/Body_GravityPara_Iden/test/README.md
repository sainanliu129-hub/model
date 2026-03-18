# test（可运行脚本）

本目录为**直接点击即可运行**的脚本：辨识入口、对比、示例、验收等。数据与结果路径均相对于上级目录（Body_GravityPara_Iden）的 `data/`、`build/`。

常用入口：
- `run_min_param_id_from_csv` — 从 CSV 做最小参数辨识
- `run_full_dynamics_validation` — 一次跑全（含 ID/FD 对比图）；或用 `run_debug_pi_phys_id_fd_plot` 单独调 π_fd 并画 ID/FD
- `run_identify_pi_fd_only` — 只跑 π_fd 优化并保存到 build（不画图）
- `run_plot_id_fd_only` — 只读已保存参数做 ID/FD 对比（不跑优化，适合反复改窗/画图）
- `run_light_dynamics_validation` — 轻量版全流程：可选“轨迹A辨识/轨迹B对比”，按 cfg.models 选模型
- `run_torque_comparison` — 多版本力矩对比
- `run_check_full_param_consistency_cad_only` — **CAD 全参链路自检（不依赖 mat）**，阶段 A 必跑
- `run_check_full_param_consistency` — 全参链路自检（用 mat 轨迹，阶段 D）
- `run_diagnose_pi_Y_columns` — 自检未过时立刻跑，排查 π 与 Y 列（无 mat 时用随机轨迹）
- `run_check_regressor_column_norms` — 打印 Y 列范数及 (link, 参数)
- `run_check_pi_order_vs_urdf` — 打印 π_cad 与 para_order 对照
- `example_run_min_param_id` — 示例一键辨识
- `test_ReMatrix_E1_limb_URDF` — 回归矩阵验收
