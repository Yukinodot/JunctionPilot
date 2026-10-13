# JunctionPilot

**面向电赛小车练习的路径决策项目：把车头方向、转弯耗时和道路状态纳入规划。**

项目围绕送药、寻径和避障类电赛小车的导航需求展开。它包含三个结果一致的规划实现：Python 用于建模和实验，C 用于后续嵌入式移植，JavaScript 用于离线地图仿真。

核心问题是：小车从某个方向进入路口后，应当走哪条路？较短的路线可能有更多转弯；较快的路线也可能经过更难识别或容易失误的路段。

本仓库是原创编写的算法练习项目，**没有参赛获奖经历，也没有完成实车制作**。航向状态搜索和加权路径规划已有成熟方法，这里的特色是成本建模、执行约束和跨语言验证，不声称学术首创。

## 项目内容

- **最短距离 / 预计最快 / 风险加权**三种策略，在同一地图和通行条件下比较。
- 用 `(路口, 车头方向)` 搜索状态，分别考虑左转、右转、掉头及稳定时间。
- 道路阻塞更新和地图版本检查；在确认位置后重规划。
- 多帧识别确认门槛与停车事件逻辑。图像识别模型本身尚未实现。
- 无动态内存分配的 C 规划器与持续停车锁定逻辑。
- 本地可操作的地图面板：设置道路阻塞、修改转弯参数、逐路段执行和返航。

## 快速运行

Python 3.10+，核心无第三方运行依赖。

```bash
python -m pip install -e .
python -m unittest discover -s tests -v
python -m junctionpilot.cli maps/turn_tradeoff.json --policy fastest
```

输出路径、每段动作和 `outputs/route.json`。以下示例保持同一地图，分别改变目标或道路状态：

```bash
python -m junctionpilot.cli maps/turn_tradeoff.json --policy distance
python -m junctionpilot.cli maps/turn_tradeoff.json --policy robust
python -m junctionpilot.cli maps/turn_tradeoff.json --block S U
python -m junctionpilot.cli maps/turn_tradeoff.json --calibration configs/calibration.example.json
```

地图不可达时返回退出码 2，输出 `route: null`，上层控制应保持停车。

直接用浏览器打开 [demo/index.html](demo/index.html) 即可操作仿真面板。文件不需要网络，也不发送小车数据。地图为自建拓扑练习图，绘图坐标不表示实际尺寸；道路长度、宽度使用单独属性。

## 一个可以讲清楚的例子

默认标定参数均为仿真设定。自建 `turn_tradeoff` 地图上：

| 策略 | 路线 | 距离 | 转弯 | 名义时间 |
|---|---|---:|---:|---:|
| 最短距离 | S → A → B → C → D → G | 1000 mm | 4 | 6.735 s |
| 预计最快 | S → U → V → G | 1150 mm | 3 | 6.284 s |
| 风险加权 | S → A → B → C → D → G | 1000 mm | 4 | 6.735 s |

第二条路线多走 150mm，但在此模型中少转一次弯，名义时间减少 451ms。第三种策略考虑合成风险分数后选择另一条路。**这些是计算结果，不是实车计时。**

## 实验与验证

已在本机运行：

- 24 项 Python 检查，覆盖方向成本、不可达、宽度约束、返航、识别确认和中途阻塞。
- 200 张合成 5×5 地图，共 600 个策略查询。
- GCC 编译实际 C 文件，使用 `-Wall -Wextra -Werror`，600/600 个查询的路径、成本和状态与 Python 一致。
- JavaScript 的 600/600 个查询与 Python 一致。
- C 规划器工作区为 **2304 字节**；图结构 2568 字节，路线结果结构 6164 字节，均为本机 C ABI 的 `sizeof` 实测值。

200 张图中有 68 张可达，其余因随机缺路、阻塞或宽度不足而不可达。在可达案例中，预计最快策略相对距离策略平均减少 **4.02% 的模型内名义时间**，24 张图有严格减少。风险加权策略在 26 张可达图上减少风险分数之和。它们体现给定成本模型中的取舍，不能证明真实赛道加速或故障率下降。

详见 [仿真报告](reports/RESULTS.md)、[逐查询 CSV](reports/per_case.csv)、[完整地图与路径记录（gzip）](reports/cases.json.gz)、[C 编译记录](reports/c-build.log)。可运行 `python scripts/unpack_cases.py` 恢复原始 JSON，全部验证脚本直接读取压缩记录。

重新运行全部验证：

```bash
python scripts/benchmark.py
python scripts/verify_c.py --cc gcc
node scripts/verify_js.js
python scripts/build_demo.py
```

Windows 若 GCC 未进入 PATH，可以使用 `--cc C:/msys64/ucrt64/bin/gcc.exe`。Node.js 只用于检查浏览器规划器，Python 核心不依赖它。

## C 与硬件接入

```bash
gcc -std=c11 -Wall -Wextra -Werror firmware/demo.c firmware/jp_planner.c firmware/jp_guard.c -o build/demo
python scripts/export_firmware.py maps/turn_tradeoff.json --output outputs/map_generated.h
```

先创建 `build` 目录，Windows 输出名可用 `build/demo.exe`。`firmware` 没有厂商 HAL 依赖；需要在你的开发板工程中接入 PWM、电机、传感器、定时器和路口确认。**主机上 C 编译成功不代表已在 MCU 上烧录运行。**

- [硬件接口、连线与标定步骤](docs/HARDWARE.md)
- [执行状态与阻塞处理](docs/EXECUTION.md)
- [成本模型与算法](docs/METHOD.md)
- [电赛选题来源与范围](docs/TOPIC_REVIEW.md)
- [项目讲解与后续实验](docs/PROJECT_STORY.md)

## 电赛来源

题目主题参考了 [2021 年 F 题智能送药小车](https://www.nuedc-training.com.cn/index/news/details/new_id/259) 和 [2025 年 K 题自动避障小车](https://res.nuedc-training.com.cn/topic/2025/topic_130.html)，后者为高职高专组。赛题链接与查证范围保存在选题文档中。

本仓库未按某届题目完整复刻赛道，也未验证该届全部器件、车体、限时、任务和评分要求。参加具体赛事时应依据对应组别的正式赛题调整实现。

## 结构

```text
src/junctionpilot/   地图、规划、识别门槛、事件执行与命令行
firmware/           可移植 C 规划器、停车锁定、主机示例
maps/               两张自建练习图
configs/            标定示例（未实测）
demo/               离线路径仿真面板
scripts/            实验、跨语言验证、C 地图导出
reports/            实际运行记录与合成案例
docs/               选题、方法、硬件接入与讲解
```

代码由 AI 协助编写，使用 MIT 许可。赛题与厂商资料保留各自权利，仓库仅引用来源链接。
