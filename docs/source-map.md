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
