# Drake Atlas

**理解 Drake 的建模、控制、物理机制与源码实现。**

[English](README.en.md) · [系列首页](https://github.com/huangkiki/sim-atlas) · [入门导读](docs/guide.md) · [完整课程路线](docs/curriculum.md) · [源码地图](docs/source-map.md) · [版本](docs/versions.md) · [开发任务](docs/roadmap.md) · [六仓总看板](https://github.com/users/huangkiki/projects/2)

这是 **[Sim Atlas · 仿真图谱](https://github.com/huangkiki/sim-atlas)** 的独立社区学习仓库，重点覆盖 Systems/Diagram/Context、MultibodyPlant、SceneGraph、SAP、控制与优化。

提供两条完整路线：**A 应用路线**从对象与建模走向控制、机器人、传感器、学习接口与数据；**B 原理与源码路线**解释动力学、接触模型、求解器、积分、观测及扩展。已交付首篇导读、E1「模型、坐标、状态与时间」、E2「驱动、机器人与任务接口」及 E3「接触、求解器与力观测」专题，完整课程继续开发。

## 从这里开始

1. 阅读[导读](docs/guide.md)，建立对象与调用关系。
2. 阅读[E1 模型与坐标](docs/modeling-state-time.md)及[状态与时间](docs/state-time.md)，对照[原创 API 例子](examples/README.md)。
3. 继续[E2 驱动与机器人](docs/control-robotics.md)和[任务接口](docs/task-interfaces.md)，区分控制参考、驱动力和物理观测。
4. 阅读[E3 接触模型](docs/contact-models.md)、[求解与时间](docs/contact-solvers.md)及[力观测](docs/contact-observation.md)，把材料、solver 和采样分开理解。
5. 跟随[源码地图](docs/source-map.md)，在固定提交中核对原生字段、配置和执行路径。
6. 按[课程路线](docs/curriculum.md)选择应用或原理专题；需要环境时看[安装说明](docs/installation.md)。

当前先完成引擎知识体系与源码课程。最小 API 片段服务于理解，运行状态逐项注明；本轮没有新增仿真实验、训练、基准或独立评分器。后续实验复用 [DexLab](https://github.com/huangkiki/Dexlab) 的版本、配置和工况记录。

## 已交付专题

| 专题 | 读完能够解释 | 验证边界 |
|---|---|---|
| [E1：模型、坐标、状态与时间](docs/modeling-state-time.md) | Parser/Finalize、空间惯量、q/v、Context 所有权与重置、连续/离散时间、输出采样 | [源码与语法已核对](docs/evidence/e1-validation.md)，未运行引擎 |
| [E2：驱动、机器人与任务接口](docs/control-robotics.md) | 驱动映射、PD/限额、FK/Jacobian/IK、原生事件状态机 | [源码与静态检查](docs/evidence/e2-validation.md)，未运行控制或优化 |
| [E3：接触、求解器与力观测](docs/contact-models.md) | point/hydro 材料组合、三种近似、SAP 终止、连续/离散积分、force/wrench/impulse | [源码与静态检查](docs/evidence/e3-validation.md)，未运行接触或求解 |

E4–E7 仍按[路线](docs/roadmap.md)开发，不以目录存在替代课程完成。

## 维护与来源

每章保留原生 API、版本化来源、易错点和阅读练习。各 Atlas 仓库独立，不需要安装其他 Atlas 或 DexLab。教程进度和 DexLab 实验证据覆盖分别记录，不据此给引擎排名。

[官方源码基线](https://github.com/RobotLocomotion/drake/tree/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d) · [贡献](CONTRIBUTING.md) · [来源与许可](THIRD_PARTY.md)
