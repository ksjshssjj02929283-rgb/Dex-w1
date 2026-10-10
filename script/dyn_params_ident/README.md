# 使用方法

## traj2 真机前馈模拟

左右臂可分别点击“辨识 / 验证”的“选择文件”，从任意目录选择采集数据，文件名不限。文件格式为 15 或 22 列空白分隔数值：时间、7 关节位置、7 电流估算力矩、可选的 7 传感器力矩。无需对应的参考轨迹文件。切换辨识文件需要重新辨识；切换验证文件保留当前参数。原“选择本地数据目录”仍可批量恢复 `left/right/rbtlog_traj1.dat` 和 `rbtlog_traj2.dat`。

Qt 完成辨识后，点击“**四种动力学工况 RMSE 对比**”一次输出四种结果：

- 完整动力学，未缩放摩擦；
- `ddq=0` 前馈，未缩放摩擦；
- `ddq=0` 前馈，加真机摩擦缩放；
- 完整动力学，只加真机摩擦缩放。

控制台会按左右臂、各关节输出四种工况的 RMSE，结果不写入参数配置。

默认勾选“使用 YAML 摩擦比例”，每次对比读取 `~/dexe_robot_server_data/robot_config.yaml` 当前版本下的左右臂逐关节比例。取消勾选可临时调节 J1–J5 的 `fv`、J6–J7 的 `fv` 和全部关节的 `fc`（范围 0–10，左右臂共用），仅用于离线试算。无需重新辨识；速度死区和 `fo` 偏置保持原规则。控制台输出本次生效比例和来源。

## 真机摩擦比例配置

在 `robot_config.yaml` 的 `w1_v02` / `w1_v03` 块中，紧接 `dyn_base_params_secondary`：

```yaml
  fv_ratio_primary: [0.6, 0.6, 0.6, 0.6, 0.6, 0.5, 0.5]
  fv_ratio_secondary: [0.6, 0.6, 0.6, 0.6, 0.6, 0.5, 0.5]
  fc_ratio_primary: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
  fc_ratio_secondary: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
```

`primary` 为左臂，`secondary` 为右臂，每行按 J1–J7 排列，必须包含 7 个有限非负数。倍率直接乘原辨识摩擦系数，0 关闭该项，1 为原系数。修改后重启服务生效。

新版本服务每次启动会自动补齐缺失字段，保留已有调节值；默认倍率保持原有补偿效果。库仑摩擦的 0.2 已移到 YAML，生成代码仅保留速度死区和正负号，不会再额外乘 0.2。升级时需重新编译安装 `robot_server_lib` 和 `w1_server`，以更新程序及默认配置。

Qt 从 YAML 读取比例，并校验本地 `inverse_dynamics_symbol.cpp` 和 `regression_func.cpp` 的映射与死区。默认规则为：

| 参数 | C++ 下标（从 0 开始） | 缩放 |
| --- | --- | --- |
| 黏性摩擦 fv | 3, 13, 24, 35, 46, 57, 68 | 前 5 个关节 0.6，后 2 个 0.5 |
| 库仑摩擦 fc | 4, 14, 25, 36, 47, 58, 69 | 0.2；速度绝对值 ≤ 0.000872664626 rad/s 时为零 |

Qt 会校验参数名称与生成代码的下标。AF-ARF 对比在原滤波域进行，库仑摩擦死区先应用，再滤波、裁剪。验证保留原轨迹的惯性、重力等项，仅改变摩擦，不等同于真机 `ddq=0` 的控制器回放。

按钮直接使用当前参数计算：

```text
tau_ff_real = g(q) + c(q, dq) + r_v*fv*dq + r_c*fc*sign_deadband(dq) + fo
```

其中 `ddq=0` 两种工况不重新辨识、不构造新的回归矩阵、不计算惯性矩阵或 `M*ddq`；完整动力学两种工况使用已缓存的完整回归预测。图中同时绘制四种预测曲线，AF-ARF 辨识结果同样可用。惯性参数仍用于科氏/离心力计算，不能把惯性参数本身置零。

对比参考是验证文件的电流力矩。实测力矩可能含加速惯性和外力，所以此处 RMSE 只表征前馈与实测力矩的差异，不能用来评判完整动力学拟合。Qt 使用处理后的位置/速度及手臂模型，不包含工具载荷补偿和全身姿态变化。

## 安装依赖
```bash
# 安装sympybotics
python -m pip install --upgrade pip
git clone https://github.com/cdsousa/SymPyBotics.git
cd SymPyBotics
python3 setup.py install --user
cd .. && rm -rf SymPyBotics
# 安装pylmi_sdp
pip install -i https://mirrors.aliyun.com/pypi/simple/ pylmi_sdp
# 安装dsdp
sudo apt install -y dsdp
```

## 可能遇到的问题

### `lmi_sdp`与`SymPy`版本不兼容问题

如果遇到如下报错：

```
ImportError: cannot import name 'MatrixError' from 'sympy.matrices.matrices' (/home/xc/.local/lib/python3.10/site-packages/sympy/matrices/matrices.py)
```

请手动修改 `lmi_sdp` 包中的相关文件，将

```python
from sympy.matrices.matrices import MatrixError
```

改为

```python
from sympy.matrices.common import MatrixError
```

需要修改的文件包括：

- `.../python3.x/site-packages/lmi_sdp/lm.py`
- `.../python3.x/site-packages/lmi_sdp/lmi.py`
