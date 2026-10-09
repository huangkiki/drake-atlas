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

## E2：驱动、机器人与任务接口

专题：[驱动与机器人](control-robotics.md) → [任务接口](task-interfaces.md)。原生输入到输出/事件的引用范围见正文及[E2 验证](evidence/e2-validation.md)，本次新增以下固定文件。

| 文件 | E2 阅读目的 |
|---|---|
| [bindings/pydrake/multibody/inverse_kinematics_py.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/multibody/inverse_kinematics_py.cc) | Python IK Context 重载与位置约束参数 |
| [bindings/pydrake/systems/controllers_py.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/systems/controllers_py.cc) | PID 的 Python 参数和端口名 |
| [bindings/pydrake/systems/framework_py_values.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/systems/framework_py_values.cc) | BasicVector 副本与数值访问 |
| [bindings/pydrake/systems/primitives_py.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/systems/primitives_py.cc) | Saturation 与 ZeroOrderHold Python 参数 |
| [multibody/inverse_kinematics/add_multibody_plant_constraints.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/inverse_kinematics/add_multibody_plant_constraints.h) | 关节/四元数/闭链等约束及硬软距离区别 |
| [multibody/inverse_kinematics/differential_inverse_kinematics.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/inverse_kinematics/differential_inverse_kinematics.h) | 经典微分 IK 的 N(q)、速度/加速度限制和 stuck |
| [multibody/inverse_kinematics/differential_inverse_kinematics_system.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/inverse_kinematics/differential_inverse_kinematics_system.h) | Systems 微分 IK 的端口与 v=qdot 限制 |
| [multibody/inverse_kinematics/inverse_kinematics.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/inverse_kinematics/inverse_kinematics.cc) | IK 模型约束装配与关闭 joint-limit 的实际范围 |
| [multibody/inverse_kinematics/inverse_kinematics.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/inverse_kinematics/inverse_kinematics.h) | IK 约束、Context 所有权/修改与碰撞前置条件 |
| [multibody/parsing/detail_urdf_parser.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/parsing/detail_urdf_parser.cc) | URDF transmission 与 actuator 的创建 |
| [multibody/plant/externally_applied_spatial_force.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/externally_applied_spatial_force.h) | 作用点在 B 表达、空间力在 world 表达 |
| [multibody/plant/sap_driver.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/sap_driver.cc) | PD + 前馈联合限额、SAP 净驱动读回 |
| [multibody/tree/joint_actuator.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/tree/joint_actuator.cc) | 增益有限性、非负约束与关闭 controller |
| [multibody/tree/joint_actuator.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/tree/joint_actuator.h) | actuator 输入位置、限额、增益与公开适用模式 |
| [solvers/mathematical_program.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/solvers/mathematical_program.h) | IK 二次代价、初值接口 |
| [solvers/mathematical_program_result.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/solvers/mathematical_program_result.h) | 优化结果成功状态、解与约束诊断 |
| [systems/controllers/inverse_dynamics.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/controllers/inverse_dynamics.h) | inverse dynamics 力项和重力补偿 |
| [systems/controllers/inverse_dynamics_controller.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/controllers/inverse_dynamics_controller.h) | 加速度反馈、完全驱动假设和模型参数 Context |
| [systems/controllers/pid_controller.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/controllers/pid_controller.h) | PID 状态、维度、反馈与投影 |
| [systems/framework/input_port.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/framework/input_port.h) | Eval、FixValue 的覆盖与复制 |
| [systems/framework/leaf_system.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/framework/leaf_system.h) | 离散 State、周期事件和输出依赖声明 |
| [systems/primitives/saturation.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/primitives/saturation.h) | 逐元素限额与 direct feedthrough |
| [systems/primitives/zero_order_hold.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/primitives/zero_order_hold.h) | 保持状态、首次采样及初始化 |
| [bindings/pydrake/trajectories_py.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/trajectories_py.cc) | 三次轨迹 Python 重载与参数 |
| [common/trajectories/piecewise_polynomial.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/common/trajectories/piecewise_polynomial.h) | 三次路径插值、样本列和端点导数 |
| [systems/primitives/trajectory_source.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/primitives/trajectory_source.cc) | Context 时间求值、堆叠顺序及区间外导数 |
| [systems/primitives/trajectory_source.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/systems/primitives/trajectory_source.h) | 轨迹输出阶数与 System 所有权 |

## E3：接触、求解与力观测

阅读链：[材料与几何](contact-models.md) → [求解与时间](contact-solvers.md) → [力与采样](contact-observation.md)。已存在的 Plant/SapDriver/SAP 参数源码继续复用；新增入口如下。

| 固定源文件 | 本轮核对目的 |
|---|---|
| [bindings/pydrake/geometry/geometry_py_scene_graph.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/geometry/geometry_py_scene_graph.cc) | Python surface/normal/inspector 字段 |
| [geometry/collision_filter_manager.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/geometry/collision_filter_manager.h) | 候选对、固有排除与模型/Context manager 生命周期 |
| [geometry/proximity_properties.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/geometry/proximity_properties.h) | 材料属性与 rigid/compliant hydroelastic 构造参数 |
| [geometry/query_object.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/geometry/query_object.h) | hydro 支持对、网格、fallback 与 scalar 边界 |
| [geometry/query_results/contact_surface.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/geometry/query_results/contact_surface.h) | surface 身份、world 网格与弹性压力场 |
| [geometry/query_results/penetration_as_point_pair.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/geometry/query_results/penetration_as_point_pair.h) | 见证点、穿透与 B→A 法向 |
| [multibody/contact_solvers/contact_solver_results.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/contact_solver_results.h) | 内部力/速度/广义量，核对 ft 遗留注释 |
| [multibody/contact_solvers/sap/contact_problem_graph.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/contact_problem_graph.h) | clique、cluster 和参与图的范围 |
| [multibody/contact_solvers/sap/sap_contact_problem.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_contact_problem.h) | 线性化动量、A、v* 与 clique |
| [multibody/contact_solvers/sap/sap_friction_cone_constraint.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_friction_cone_constraint.cc) | R、near-rigid 下界及摩擦锥投影 |
| [multibody/contact_solvers/sap/sap_friction_cone_constraint.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_friction_cone_constraint.h) | 线性柔顺、relaxation time 与正则化 |
| [multibody/contact_solvers/sap/sap_hunt_crossley_constraint.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_hunt_crossley_constraint.cc) | Similar/Lagged 的 n0、z、cost 与 impulse |
| [multibody/contact_solvers/sap/sap_hunt_crossley_constraint.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_hunt_crossley_constraint.h) | Hunt–Crossley 近似与参数量纲 |
| [multibody/contact_solvers/sap/sap_model.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_model.cc) | cost/gradient/Hessian、负平方根缩放 |
| [multibody/contact_solvers/sap/sap_solver.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/contact_solvers/sap/sap_solver.cc) | Newton/线搜索、实际终止与 AutoDiff 隐函数路径 |
| [multibody/plant/contact_properties.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/contact_properties.cc) | 刚度/耗散/relaxation time/动态摩擦的实际组合 |
| [multibody/plant/contact_results.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/contact_results.h) | point/hydro/deformable 结果集合 |
| [multibody/plant/coulomb_friction.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/coulomb_friction.h) | 静态与动态摩擦的调和平均 |
| [multibody/plant/discrete_update_manager.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/discrete_update_manager.cc) | 接触装配、面心求积、力读回与 q 更新 |
| [multibody/plant/hydroelastic_contact_info.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/hydroelastic_contact_info.h) | A 侧 centroid wrench 与 surface 所有权 |
| [multibody/plant/hydroelastic_traction_calculator.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/hydroelastic_traction_calculator.cc) | 连续 hydro traction、atan 摩擦与面积积分 |
| [multibody/plant/point_pair_contact_info.h](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/multibody/plant/point_pair_contact_info.h) | B 侧力、点、分离与 slip speed |
| [bindings/pydrake/geometry/geometry_py_common.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/geometry/geometry_py_common.cc) | Python 材料/geometry property 接口 |
| [bindings/pydrake/geometry/geometry_py_hydro.cc](https://github.com/RobotLocomotion/drake/blob/1e1466ba466e7ce8fa9fcca4e086ce1383e5427d/bindings/pydrake/geometry/geometry_py_hydro.cc) | Python hydroelastic 属性重载 |
