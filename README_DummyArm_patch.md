# fusion2URDF 修复版（DummyArm 项目用）

本文件夹 = **fusion2URDF v3.1.0**（上游 [Adriaeik/fusion2URDF](https://github.com/Adriaeik/fusion2URDF)，
commit `2044690`）+ **一个 17 行补丁**（`core/robot_model.py`）。

## 修的是什么

Fusion 里「关节属于根组件」时，`geometryOrOrigin*` 已经是世界坐标；
上游代码又把它按组件位姿做了一次 lift（double-lift），导致：

- 关节 origin 出现 0.5~1.1 m 的虚假平移（本例 `_1` 曾为 `-0.565 -0.071 0.0365`）
- 网格/帧距米级错位、部件散开

补丁在 `_compute_joint_global_origin` 里加了判定（`Root-owned joints` 分支）：
定义组件 = 根组件、且 geometryOrOrigin 的 one/two 两侧数值一致（1e-6）时，直接采用世界坐标，不再 lift。
**对其他模型是保守的**：判定不成立时走原逻辑，行为不变。

## 在 Fusion 里安装（Windows/Mac）

1. Fusion 360 → **实用工具 → 脚本和附加模块**（`Shift+S`）
2. **脚本**标签页 → **我的脚本** 旁边的绿色 **+**
3. 选中本文件夹（`fusion2URDF-patched`）
4. 列表里选 `fusion2URDF` → **运行**；按提示选输出目录

导出结果应**直接可用**（无需再做离线 reframe）。若仍有问题，保留输出里的 `debug/`（snapshot + 报告）并反馈。

## 备注

- 原始补丁文件：`../fusion2URDF-root-joint-fix.patch`（可对全新 clone 用 `git apply` 重打）
- 本机已验证：打补丁后的同一代码路径（离线 reframe 重生成）产出的 URDF，
  关节帧与 CAD 对齐、轴残差 <1.6mm
- 未验证：Fusion 内完整重导（需要你在 Fusion 里跑一次）
