# fusion2URDF (Root-Joint Fixed 修复增强版)

[![tests](https://github.com/BohnChen/fusion2URDF-root-joint-fixed/actions/workflows/test.yml/badge.svg)](https://github.com/BohnChen/fusion2URDF-root-joint-fixed/actions/workflows/test.yml)
[![license: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20%2F%20Humble-22314e.svg)](https://docs.ros.org/)
[![Fusion 360](https://img.shields.io/badge/Fusion%20360-script-orange.svg)](https://www.autodesk.com/products/fusion-360/)

**基于 [Adriaeik/fusion2URDF](https://github.com/Adriaeik/fusion2URDF) (v3.1.0) 的增强修复版。**

本仓库专注于解决机械臂、轮式底盘及复杂多连杆装配体在导出 URDF/ROS 2 包时，出现的**根组件关节原点米级虚假偏移、网格解构散架、关节帧错位（Double-Lift Bug）**问题，并保留原版完整的 ROS 2 Xacro、自动碰撞网格拟合、闭环机构支持与 ros2_control 配置。

---

## 核心修复说明 (Root-Owned Joint Double-Lift Fix)

### 问题复现
在 Fusion 360 中设计机器人时，通常有两种建模习惯：
1. **子装配体定义关节**：关节在子装配体内定义，坐标相对于子组件局部坐标系。
2. **顶层装配（根组件）定义关节**：所有构件作为 Occurrence 放入根设计中，关节直接在根组件下定义（常见于机械臂各轴串联、Dummy 系列机械臂装配体）。

在上游原版代码中，Phase 1 提取的根组件下关节 `geometryOrOrigin` 实际上**已经是世界/根坐标**。但后续构建树结构时，代码默认将该坐标视作子组件或父组件 Occurrence 的局部坐标，又通过其位姿矩阵做了一次变换（**Double-Lift**），导致：
- **关节坐标发生 0.5m ~ 1.1m 以上的虚假平移**；
- 模型网格在 RViz 中严重错位或“解体散开”；
- 离线生成的 URDF 旋转轴与实际几何旋转中心无法对齐。

### 修复方案
在 `core/robot_model.py` 的 `_compute_joint_global_origin` 逻辑中加入自适应保守判定：
```python
# 0b) Root-owned joints: Fusion reports geometryOrOrigin* in the root
# frame, which IS the world frame. Lifting through the child/parent
# occurrence would double-apply its pose. One/Two agreement is the discriminator:
if (
    edge.defining_component == snapshot.design_name_clean
    and fj.geometry_or_origin_one_cm is not None
    and fj.geometry_or_origin_two_cm is not None
    and max(
        abs(a - b)
        for a, b in zip(fj.geometry_or_origin_one_cm, fj.geometry_or_origin_two_cm)
    ) < 1e-6
):
    return fj.origin_global_m
```
- **保守安全**：仅当“关节定义在根组件”且“两侧几何原点完全吻合”时，直接采用世界原点，消除重复变换。
- **向下兼容**：对于其他嵌套子装配体或常规装配场景，不触发该条件，完全遵循原版逻辑。
- **实测验证**：已在 DummyArm 6-DoF 机械臂及官方测试套件上实测通过，关节轴残差 < 1.6mm，部件无拉扯错位。

---

## 功能特性

- **一键导出 ROS 2 标准包**：自动生成 `urdf/`（模块化 Xacro 与扁平化 URDF）、`meshes/`（Visual 与 Collision）、`launch/`、`config/`（ros2_controllers.yaml）。
- **多种碰撞体策略支持**：
  - 默认根据几何体生成紧凑 Primitive（长方体、圆柱、球体）。
  - 支持前缀 `!acc_*`（精确视觉网格碰撞）、`!cxh_*`（凸包碰撞）、`!pri_*`（基元碰撞）。
- **真实惯性矩阵导出**：直接从 Fusion 360 物理材质与重心计算完整惯量张量。
- **闭环连杆（Closed-Loop Kinematics）支持**：平行四边形、四连杆夹爪自动检测，主运动链生成 URDF 树，闭环副写入 `robot_data.yaml` 供 Isaac Sim / MuJoCo 重建。
- **坐标系重定位工具**：自带 `tools/reframe.py`，允许在导出后通过 CSV 配置调整关节轴向（Z 轴向前/向外）而无需重跑 Fusion。

---

## 快速上手与安装

### 1. 安装到 Fusion 360

#### macOS:
打开终端，将此仓库克隆或软链接至 Fusion 360 的 Scripts 目录：
```bash
# 方式一：直接软链接（推荐，开发同步方便）
git clone https://github.com/BohnChen/fusion2URDF-root-joint-fixed.git ~/fusion2URDF
ln -s ~/fusion2URDF "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/fusion2URDF"
```

#### Windows:
将克隆的文件夹复制或通过符号链接（`mklink /D`）放置在：
`%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\fusion2URDF`

### 2. 在 Fusion 360 中运行
1. 打开你要导出的机器人装配体模型。
2. 确保装配体中的每个活动构件都已创建关节（Joint）或固定（Rigid Group）。
3. 按下快捷键 `Shift + S` 打开 **实用程序 (Utilities) -> 脚本和附加模块 (Scripts and Add-Ins)**。
4. 在 **我的脚本 (My Scripts)** 列表中找到 **fusion2URDF**，点击 **运行 (Run)**。
5. 弹窗选择导出的文件夹，稍作等待（导出过程中会有趣味冷知识提示弹窗）。

### 3. 在 ROS 2 中可视化

导出完成后，在终端编译并启动可视化：

```bash
# 进入你的 ROS 2 工作空间并复制包
cd ~/ros2_ws/src
cp -r <导出路径>/<robot_name>_description .

# 编译并加载环境
cd ~/ros2_ws
colcon build --packages-select <robot_name>_description
source install/setup.bash

# 启动 RViz 预览
ros2 launch <robot_name>_description display.launch.py
```

---

## 进阶技巧：Fusion 内命名标记 (`!`-tags)

无需退出 Fusion，通过直接在组件名、实体名或关节名添加前缀即可灵活控制导出行为：

| 前缀 / 标记 | 应用对象 | 功能说明 |
|---|---|---|
| `!acc_*` | 组件 / 刚性组 | 强制采用高精度原视觉网格作为碰撞体（适合夹爪指尖接触面） |
| `!cxh_*` | 组件 / 刚性组 | 强制生成凸包（Convex Hull）碰撞网格 |
| `!pri_*` | 组件 / 刚性组 | 强制自动拟合几何基元（盒子/圆柱）作为碰撞体 |
| `!` (实体前缀) | 实体 (Body) | 将特定实体标记为“仅视觉展示”（如螺丝、电线、天线），不计入碰撞轮廓 |
| `!frame_*` | 虚拟组件 | 纯坐标系标记（无质量、网格），常用于 `!frame_footprint` 或工具末端中心 TCP |
| `!passive_*` | 关节 (Joint) | 标记为被动关节，`ros2_control` 将不为其生成执行控制器接口 |
| `!closing_*` | 关节 (Joint) | 明确标记该关节为四连杆/平行闭环副中的闭环约束 |

---

## 运行测试

如需验证或二次开发，可在本地运行单元与流水线测试：

```bash
cd tests
python -m unittest discover -s . -p "test_*.py"
# 或者使用根目录的测试运行脚本
python run_tests.py
```

---

## 致谢与上游项目

- 上游主项目：[Adriaeik/fusion2URDF](https://github.com/Adriaeik/fusion2URDF)（感谢 @Adriaeik 构建了现代 ROS 2 / Xacro 的现代化导出器框架）
- 经典项目：[syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf)（ROS 1 时代的先驱）
