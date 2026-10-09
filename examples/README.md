# E1 阅读例子

[model_state.py](model_state.py) 使用 Drake 1.57.0 的原生 Python API，展示 Parser → Finalize → Diagram → Context、浮动基座世界位姿、空间速度、数组副本、根 Context 克隆和默认状态重置。

**交付状态：源码核对与 Python 语法检查；未执行。** 不含 Simulator、AdvanceTo、独立实验或评分器。即使以后运行并成功打印，也只证明该环境下的 API/状态操作，不证明物理、接触或性能正确。内嵌 URDF 是本课程原创均匀盒，没有外部资产。

先阅读[模型与坐标](../docs/modeling-state-time.md)和[状态与时间](../docs/state-time.md)。预测浮动基座的 nq/nv、快照与原 Context 的位置差异，以及 SetDefaultContext 后的时间，再对照固定源码解释。当前阶段不要求运行。

可在不安装/导入 Drake 的情况下检查语法：

```bash
python -c "import ast; from pathlib import Path; ast.parse(Path('examples/model_state.py').read_text())"
```

源码/语法检查不覆盖 Parser 实际接受资产、Python ABI、几何后端与运行期约束；这些限制记录在[验收文件](../docs/evidence/e1-validation.md)。
