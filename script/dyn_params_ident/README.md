# 使用方法

## Qt 摩擦模型选择

“计算参数与写入”区域可选择经典库仑、平滑库仑或 Stribeck，传统回归及 AF-ARF 都使用所选模型。支持编辑平滑库仑的 tanh 平滑速度、Stribeck 共用的特征/平滑速度和指数。控件按当前模型启用，计算/采集期间禁止修改；参数改变后清除双臂旧辨识结果，需要重新计算。

选择 Stribeck 后，直接使用原有的“tanh 平滑速度”控件，与“Stribeck 特征速度”双向同步，不新增独立速度框；后端及 YAML 仅使用一个 `stribeck_velocity` 参数。切回平滑库仑时，“tanh 平滑速度”恢复该模型独立的 `friction_velocity_scale`，Stribeck 调节不会覆盖它。经典库仑模式禁用速度控件。

Qt 默认读取 `~/dexe_robot_server_data/robot_config.yaml`，而不是其他目录中同名文件。在当前版本段（`w1_v02` / `w1_v03`）可手动设置：

```yaml
  friction_model: smooth
  friction_velocity_scale: 0.1
  stribeck_velocity: 0.1
  stribeck_alpha: 2.0
```

- `classic`：`Fc*sign(v) + Bv*v + Fo`。
- `smooth`（默认）：`Fc*tanh(v/friction_velocity_scale) + Bv*v + Fo`。
- `stribeck`：`[Fc+(Fs-Fc)*exp(-(|v|/stribeck_velocity)^stribeck_alpha)]*tanh(v/stribeck_velocity) + Bv*v`。

经典/平滑模型的 `Fo` 是恒定偏置，并非真正的静止粘滞摩擦。Stribeck 复用原 `fo` 参数位置表示 `Fs`，不再包含独立偏置；`sign(0)=0`。

Stribeck 始终使用 tanh，指数衰减和零速附近方向过渡共用 `stribeck_velocity`，默认 `0.1`。`friction_velocity_scale` 只用于平滑库仑模型，默认也为 `0.1`。速度单位均为 rad/s，必须是有限正数，不支持用 0 切换 sign。旧 YAML 中的 `stribeck_smoothing_velocity` 不再生效，Qt 写入当前版本参数时会移除该字段。旧版 sign Stribeck 参数需要重新辨识双臂，不能直接复用。

点击写入同时保存所选模型、速度、指数及对应动力学参数，保留其他版本段与注释。模型配置由双臂共用；配置改变时必须辨识并写入双臂，单臂写入仅允许摩擦设置不变。服务端在 Stribeck 模式中对 `Fc` 和 `Fs` 同时应用 `fc_ratio`。

本次服务端代码升级需要重新编译安装 `robot_server_lib` 及依赖它的服务程序；之后仅修改 YAML 模型/速度/指数无需 codegen 或编译，但必须使用匹配的新辨识参数并重启服务。修改 YAML 后重启 Qt 以加载设置。

## traj2 真机前馈模拟

左右臂可分别点击“辨识 / 验证”的“选择文件”，从任意目录选择采集数据，文件名不限。文件格式为 15 或 22 列空白分隔数值：时间、7 关节位置、7 电流估算力矩、可选的 7 传感器力矩。无需对应的参考轨迹文件。切换辨识文件需要重新辨识；切换验证文件保留当前参数。原“选择本地数据目录”仍可批量恢复 `left/right/rbtlog_traj1.dat` 和 `rbtlog_traj2.dat`。

Qt 完成辨识后，点击“**四种动力学工况 RMSE 对比**”一次输出四种结果：

- 完整动力学，未缩放摩擦；
- `ddq=0` 前馈，未缩放摩擦；
- `ddq=0` 前馈，加真机摩擦缩放；
- 完整动力学，只加真机摩擦缩放。

控制台会按左右臂、各关节输出四种工况的 RMSE，结果不写入参数配置。

默认勾选“使用 YAML 摩擦比例”，每次对比读取 `~/dexe_robot_server_data/robot_config.yaml` 当前版本下的左右臂逐关节比例。取消勾选可临时调节 J1–J5 的 `fv`、J6–J7 的 `fv` 和全部关节的 `fc`（范围 0–10，左右臂共用），仅用于离线试算。无需重新辨识；控制台输出本次生效比例和来源。

传统回归辨识使用滤波后的 `q/dq/ddq` 构造回归矩阵，拟合原始电流力矩（不滤波）。验证的 R² 和“四种动力学工况 RMSE”同样统一以原始电流力矩为目标。`q` 及由原始 `q` 差分得到的 `dq/ddq` 分别执行三阶 Butterworth 零相位滤波；状态与原始力矩统一裁剪前 3 秒和后 1 秒。代码使用归一化截止参数 `fc*2*pi*dt`，其中 `fc` 为最高谐波频率的 10 倍；换算单次 Butterworth 的设计截止频率为 `pi*fc`，默认约 5.65 Hz（不是 1.8 Hz）。传感器力矩仅用于显示，不参与辨识或主评价指标。

AF-ARF 仅在 `traj1` 辨识时使用无加速度回归及其配套 `alpha` / `beta` 力矩滤波，这部分算法不变。AF-ARF 与传统回归现在共用辨识集的固定 `dt=0.01 s` 时间网格、原始 `q` 的 Butterworth 滤波、以及“原始 `q` 差分后再滤波”的 `dq` 处理；AF-ARF 仍不计算 `ddq`。`traj2` 验证也共用同一状态预处理、采样点、裁剪和原始电流力矩目标。

两种算法都不重采样、不插值、不删除重复时间戳；计算时间按原始采样点序号乘固定 `dt` 构造，记录时间戳只保留供追溯。该口径假设数据以 100 Hz 均匀采集；实际丢帧或采样抖动不会自动校正，比较时需注意其影响。

两种算法都用辨识所得参数计算 `tau_pred = M(q)*ddq + c(q,dq) + g(q) + tau_f`（`c` 为科氏/离心力矩，等价于 `C*dq`，质量/惯性矩阵为同一个 `M`）。实现使用完整瞬时回归 `W(q,dq,ddq)*beta`，与逐项动力学计算等价，包含模型中的驱动惯量；不对预测力矩额外低通。完整与 `ddq=0` 四种工况均对比原始电流力矩。

公平比较时保持辨识文件、验证文件、机器人/臂侧、摩擦模型、状态滤波参数和摩擦比例一致，分别选择传统算法与 AF-ARF 重新辨识，记录同一验证集的完整未缩放 RMSE / R²。较低 RMSE、较高 R² 表示这个验证集上预测更好，并不预先保证 AF-ARF 一定优于传统算法。`ddq=0` 工况作为省略惯性项的辅助对比，不作为完整模型精度的主排名指标。

## 真机摩擦比例配置

在 `robot_config.yaml` 的 `w1_v02` / `w1_v03` 块中，紧接 `dyn_base_params_secondary`：

```yaml
  fv_ratio_primary: [0.6, 0.6, 0.6, 0.6, 0.6, 0.5, 0.5]
  fv_ratio_secondary: [0.6, 0.6, 0.6, 0.6, 0.6, 0.5, 0.5]
  fc_ratio_primary: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
  fc_ratio_secondary: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
```

`primary` 为左臂，`secondary` 为右臂，每行按 J1–J7 排列，必须包含 7 个有限非负数。倍率直接乘原辨识摩擦系数，0 关闭该项，1 为原系数。修改后重启服务生效。

新版本服务每次启动会自动补齐缺失字段，保留已有调节值；默认倍率保持原有补偿效果。库仑摩擦的 0.2 已移到 YAML，生成代码使用 `tanh(dq / 0.1)`，不会再额外乘 0.2。升级时需重新编译安装 `robot_server_lib` 和 `w1_server`，以更新程序及默认配置。

Qt 从 YAML 读取比例，并校验本地 `inverse_dynamics_symbol.cpp` 和 `regression_func.cpp` 的映射与死区。默认规则为：

| 参数 | C++ 下标（从 0 开始） | 缩放 |
| --- | --- | --- |
| 黏性摩擦 fv | 3, 13, 24, 35, 46, 57, 68 | 前 5 个关节 0.6，后 2 个 0.5 |
| 库仑摩擦 fc | 4, 14, 25, 36, 47, 58, 69 | 0.2；使用 `tanh(dq / 0.1)` 平滑 |

Qt 会校验参数名称与生成代码的下标。两种辨识算法的验证都在瞬时力矩域进行，摩擦基函数使用相同滤波后的速度，目标为裁剪后的原始电流力矩。完整验证保留原轨迹的惯性、重力等项，仅改变摩擦比例，不等同于真机 `ddq=0` 的控制器回放。

按钮直接使用当前参数计算：

```text
tau_ff_real = g(q) + c(q, dq) + r_v*fv*dq + r_c*fc*tanh(dq/0.1) + fo
```

其中 `ddq=0` 两种工况不重新辨识、不构造新的回归矩阵、不计算惯性矩阵或 `M*ddq`；完整动力学两种工况使用已缓存的完整回归预测。图中同时绘制四种预测曲线，AF-ARF 辨识结果同样可用。惯性参数仍用于科氏/离心力计算，不能把惯性参数本身置零。

四种工况在两种算法下的对比参考均为验证集原始电流力矩。实测力矩可能含加速惯性和外力，所以 `ddq=0` RMSE 只表征省略惯性项的前馈与实测力矩差异，不能当作完整动力学拟合误差。Qt 使用处理后的位置/速度及手臂模型，不包含工具载荷补偿和全身姿态变化。

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
