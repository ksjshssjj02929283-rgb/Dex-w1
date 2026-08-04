# navigation_client 交付文档

> **适用包版本**：`0.7.1`
> **代码基线**：`glf/modify_navigation@7e07427` + 当前工作树
> **文档集版本**：`1.3.0`
> **更新日期**：2026-08-04

本目录是 `navigation_client` 随代码版本维护的交付文档唯一事实源（SSOT）。项目 55
`交付文档/` 下的同名文档是项目管理镜像，用于评审和过程归档；发生差异时，以本目录和
当前源码为准，并在同一轮变更中回写项目镜像。

## 文档索引

| 文档 | 读者 | 内容 |
|---|---|---|
| [design.md](design.md) | 模块维护者 | 边界、架构、feature、ActionTaskClient、执行器、自测设计 |
| [api_and_usage.md](api_and_usage.md) | W1Manager / application / 直接调用方 | 公开 API、ROS 接口、状态、示例、部署与自测入口 |
| [version_manifest.json](version_manifest.json) | 发布与检查脚本 | 包版本、代码基线、文档集版本、公开契约指纹 |
| [check_docs.py](check_docs.py) | 维护者 / CI | 校验包版本、公开方法、ROS 接口配置、自测用例与文档一致性 |

模块根目录的 [../README.md](../README.md) 是快速入口，不替代本交付文档集。

## 版本绑定规则

1. `pyproject.toml` 的 `project.version` 是包版本 SSOT；文档不得自行定义另一套产品版本。
2. 下列任一项变化时，必须在同一代码变更中更新本目录：
   - `ChassisClient` 公开方法、参数、返回值或状态字符串；
   - `interfaces.py` 中 REAL/SIM topic、action、service 或参数节点；
   - `RC-A` 自动用例编号、范围、判据或正式执行入口；
   - PC2 部署方式、Python 模块入口或打包方式；
   - 行为边界、单位、坐标系、线程/Executor 约束。
3. 对外兼容契约变化时，先更新 `pyproject.toml` 版本，再更新两份正文和
   `version_manifest.json`；只改内部实现且对上述契约无影响时，可不提升文档集版本。
4. `source_revision` 记录正文核对所基于的功能提交。文档随后单独提交不会使该字段失效；
   功能契约再次变化时才更新它。
5. 正式发布前必须执行一致性检查；检查失败不得宣称文档已跟随版本更新。

## 一致性检查

从仓库根目录执行：

```bash
python3 src/dexe_robot_client/navigation_client/docs/check_docs.py
```

脚本只解析源码 AST 和 TOML/JSON，不导入 ROS2 包，也不会启动节点或修改文件。它检查：

- 包版本是否与 `version_manifest.json`、三份 Markdown 头部一致；
- `ChassisClient` 公开方法是否全部出现在调用侧文档；
- REAL/SIM 接口配置和 `RC-A` 用例目录是否仍与已审阅指纹一致；
- 交付文档是否被 `setuptools` package-data 纳入 wheel。

若契约确实已变更，先更新正文，再运行：

```bash
python3 src/dexe_robot_client/navigation_client/docs/check_docs.py --print-fingerprint
```

人工审阅输出后，把新指纹写入 `version_manifest.json`，最后重新运行默认检查。禁止仅刷新
指纹而不更新正文。

## 当前版本矩阵

| 包版本 | 功能基线 | 文档集 | 自动目录 | 状态 |
|---|---|---|---|---|
| `0.7.1` | `7e07427` + 当前 RC-A-11/最终表格变更 | `1.3.0` | `RC-A-01～RC-A-16`，每项默认 15 轮 | 当前工作树基线 |

说明：当前自动目录不含 `NavigateThroughPoses`；production 的路网导航公开接口仍因
`W1Manager` 消费而保留。自动范围删除不等于生产接口删除。15 轮属于调用稳定性门槛，
新增 `RC-A-15/16` 分离预期拒绝路径。`RC-A-11` 每轮包含四点、六点两次合法设置，因此
全量 15 轮共调用合法 footprint 30 次；未提交前不得把
`source_revision=7e07427` 单独表述为已经包含当前工作树变更。
runner 结束时还会在终端输出逐用例和 `TOTAL` 汇总表；该表是现场快速审阅入口，
`summary.json` 与 `summary.md` 仍是可追溯机器制品。

## 发布前检查清单

- [ ] `pyproject.toml` 版本与交付文档头部一致
- [ ] 两份正文已按当前源码和 W1Manager 调用面复核
- [ ] `check_docs.py` 返回 `PASS`
- [ ] wheel/源码包中包含 `navigation_client/README.md` 与 `navigation_client/docs/`
- [ ] 目标机执行结论与开发机检查分开记录；未执行项保持 `pending-execution`
