# 已运行检查与证据范围

日期：2026-10-13。

| 检查 | 结果 | 证据 |
|---|---|---|
| Python 功能检查 | 24 项通过 | `python -m unittest discover -s tests -v` |
| 算法独立核对 | 20 张 3×3 图、三策略与 Bellman-Ford 状态图松弛结果一致 | `test_exact_state_graph_oracle` |
| 合成图实验 | 200 张地图、600 个策略查询 | `reports/summary.json`、`per_case.csv`、`cases.json.gz` |
| C 实际编译 | GCC 16.2.0，警告视为错误 | `reports/c-build.log` |
| C/Python 逐段核对 | 600/600 一致，含不可达 | `reports/c-verification.json` |
| C 停车检查 | 旧版本、障碍、识别歧义、位置不明、丢线、锁定后显式恢复 | `scripts/verify_c.py` 中实际调用 |
| C 主机可执行文件 | 退出码 0 | `reports/c-build.log` |
| JavaScript/Python 核对 | 600/600 一致 | `reports/js-verification.json` |
| CLI 与地图 C 导出 | 实际运行，生成路径与地图头文件 | `junctionpilot.cli` / `export_firmware.py` |
| 浏览器仿真 | 已在 Edge 检查策略切换、阻塞重规划、不可达停车、路段中停止、位置确认 | 浏览器界面核对 |
| 压缩记录完整性 | 解压后的字节与原记录一致 | 本地 gzip 解压比较；两种跨语言检查直接读取压缩记录 |

GitHub Actions 工作流会重新执行 Python 检查、合成图实验、实际 C 编译和 JavaScript 核对；远程运行状态需在发布后另行核对。

未进行 MCU 交叉编译/烧录、实际传感器接入、循迹 PID、相机识别、赛道计时或碰撞测试。固定数组大小是本机 ABI 的实测值，不是整车固件全部 RAM 占用。
