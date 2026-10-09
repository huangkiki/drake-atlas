# drake-atlas 固定版本源码入口

阅读基线：`1e1466ba466e7ce8fa9fcca4e086ce1383e5427d`。以下入口已核对官方 Git 树与文件内容身份；不是全仓审查或运行验收记录。

本阶段先理解引擎架构、建模、步进、控制、接触/求解、传感器/渲染、性能与扩展。独立实验、基准、训练和评分暂不开展，后续复用 DexLab。

| 源码文件 | 阅读目的 |
|---|---|
| [multibody/plant/multibody_plant.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/multibody_plant.h) | 核对对象职责、数据布局、参数与版本约定 |
| [multibody/plant/multibody_plant_config.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/multibody_plant_config.h) | 核对对象职责、数据布局、参数与版本约定 |
| [multibody/plant/multibody_plant.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/multibody_plant.cc) | 核对对象职责、数据布局、参数与版本约定 |
| [multibody/plant/compliant_contact_manager.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/compliant_contact_manager.cc) | 区分接触几何、接触律与数值近似 |
| [multibody/plant/contact_model_doxygen.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/contact_model_doxygen.h) | 区分接触几何、接触律与数值近似 |
| [multibody/contact_solvers/sap/sap_solver.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_solver.h) | 识别算法、输入状态、配置与限制 |
| [systems/framework/context.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/framework/context.h) | 核对对象职责、数据布局、参数与版本约定 |
| [systems/framework/diagram.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/framework/diagram.h) | 核对对象职责、数据布局、参数与版本约定 |
| [systems/analysis/simulator.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/analysis/simulator.h) | 追踪构建、步进、数据更新与生命周期 |
| [geometry/scene_graph.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/geometry/scene_graph.h) | 追踪构建、步进、数据更新与生命周期 |
| [multibody/parsing/parser.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/parsing/parser.h) | 核对对象职责、数据布局、参数与版本约定 |
| [geometry/meshcat_visualizer.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/geometry/meshcat_visualizer.h) | 区分传感数据、可视化与物理状态 |

## E1：模型、状态与时间的追踪入口

专题：[模型与坐标](modeling-state-time.md) → [状态与时间](state-time.md)。下表补充本次实际引用的固定文件；对应行号在专题就近标注，Python 签名与源码核对范围见[验收记录](evidence/e1-validation.md)。

| 文件 | E1 阅读重点 |
|---|---|
| [bindings/pydrake/math/math_py_monolith.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/math/math_py_monolith.cc) | RigidTransform 平移构造与数学操作 |
| [bindings/pydrake/multibody/math_py.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/multibody/math_py.cc) | SpatialVelocity 的 w/v 参数 |
| [bindings/pydrake/multibody/parsing_py.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/multibody/parsing_py.cc) | Parser 原生 Python 重载与严格模式 |
| [bindings/pydrake/multibody/plant_py.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/multibody/plant_py.cc) | 构造器显式 time_step、getter 引用/副本、free-body 参数 |
| [bindings/pydrake/systems/framework_py_semantics.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/systems/framework_py_semantics.cc) | Context.Clone Python 绑定 |
| [bindings/pydrake/systems/framework_py_systems.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/systems/framework_py_systems.cc) | System.SetDefaultContext 与子 Context Python 入口 |
| [common/value.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/common/value.h) | abstract value 的复制/Clone 语义与外部资源边界 |
| [math/rigid_transform.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/math/rigid_transform.h) | X_AB、点/方向变换、RPY 与四元数入口 |
| [multibody/math/spatial_force.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/math/spatial_force.h) | 力矩/力排列与参考点 |
| [multibody/math/spatial_velocity.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/math/spatial_velocity.h) | 角/线速度排列、measured/expressed frame、参考点 Shift |
| [multibody/parsing/package_map.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/parsing/package_map.h) | 本地包映射、资源 URL 与远程资源边界 |
| [multibody/tree/joint.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/tree/joint.h) | position_start 与 velocity_start 分别位于 q 和 v |
| [multibody/tree/multibody_tree.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/tree/multibody_tree.cc) | free-body setter 实际转交 mobilizer 的路径 |
| [multibody/tree/quaternion_floating_joint.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/tree/quaternion_floating_joint.h) | q/v 布局、parent/child frame 与速度表达 |
| [multibody/tree/spatial_inertia.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/tree/spatial_inertia.h) | 中心惯量、UnitInertia、平行轴、有效性 |
| [systems/analysis/integrator_base.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/analysis/integrator_base.h) | 连续误差控制、fixed-step 与事件边界 |
| [systems/framework/context_base.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/framework/context_base.h) | System 与 Context 树的一一对应、Clone 根条件 |
| [systems/framework/diagram_builder.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/framework/diagram_builder.h) | Build 的单次使用与系统所有权 |
| [systems/framework/system.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/framework/system.cc) | SetDefaultContext 实际重置的字段 |
| [systems/framework/system.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/framework/system.h) | 子 Context 获取、默认状态及输入复制 API |
