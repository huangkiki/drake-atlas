# 原生 API 阅读例子

[model_state.py](model_state.py) 使用 Drake 1.57.0 的原生 Python API，展示 Parser → Finalize → Diagram → Context、浮动基座世界位姿、空间速度、数组副本、根 Context 克隆和默认状态重置。

**交付状态：源码核对与 Python 语法检查；未执行。** 不含 Simulator、AdvanceTo、独立实验或评分器。即使以后运行并成功打印，也只证明该环境下的 API/状态操作，不证明物理、接触或性能正确。内嵌 URDF 是本课程原创均匀盒，没有外部资产。

先阅读[模型与坐标](../docs/modeling-state-time.md)和[状态与时间](../docs/state-time.md)。预测浮动基座的 nq/nv、快照与原 Context 的位置差异，以及 SetDefaultContext 后的时间，再对照固定源码解释。当前阶段不要求运行。

可在不安装/导入 Drake 的情况下检查语法：

```bash
python -c "import ast; from pathlib import Path; ast.parse(Path('examples/model_state.py').read_text())"
```

源码/语法检查不覆盖 Parser 实际接受资产、Python ABI、几何后端与运行期约束；这些限制记录在[验收文件](../docs/evidence/e1-validation.md)。

## E2：控制、机器人与任务

| 文件 | 说明 | 未执行边界 |
|---|---|---|
| [arm_kinematics.py](arm_kinematics.py) | 原创两杆 URDF，显式 actuator/焊接、TCP FK/Jacobian、独立 Context 中的 IK 问题 | 未构建模型或求解；无碰撞几何/避障声明 |
| [sampled_pd.py](sampled_pd.py) | 复用该原创资产，原生 PID → Saturation → ZeroOrderHold → actuation | 未运行闭环；参数未经调参或稳定性验证 |
| [task_sequencer.py](task_sequencer.py) | LeafSystem 周期事件与 Context 状态，接近/闭合/保持/释放/异常 | 未执行事件；force/slip 输入是假定合同，未实现传感 |

这些文件不是统一仿真封装。两个 arm 示例共享一个具体教学模型的构建函数；任务例子独立说明 Systems 的状态/事件，不伪造机械臂轨迹或接触源。所有代码均只做完整 AST、源码 API 和资产静态检查，阈值/增益/周期只是教学数值。见[控制与机器人](../docs/control-robotics.md)、[任务接口](../docs/task-interfaces.md)、[E2 验证](../docs/evidence/e2-validation.md)。
