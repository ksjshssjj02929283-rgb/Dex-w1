# Dex-w1 Git 历史归档

此仓库保存 `w1` 工作区内 7 个 Git 仓库在归档时本地可用的分支、版本标签、历史引用和子模块检出版本，所有版本均保留；为通过 GitHub 密钥保护，已移除历史中的 GitLab Deploy Token，受影响的提交和标签哈希已更新。

归档时间：2026-11-06T16:50:00.578529+08:00（Asia/Shanghai）。

## 分支和标签

各来源仓库的相对路径作为分支和标签前缀。例如：

- `dexe_robot_server/dev_w1`：开发目录当前已提交的版本。
- `upstream_repos/dexe_robot_server/dev_w1`：上游归档中的同名分支。
- `upstream_repos/dexe_robot_client/main_w1`：客户端归档分支。
- `upstream_repos/dexe_robot_server/V0.5.1`：带仓库前缀的版本标签。

本地已有的远端跟踪分支也保存为可检出的分支；与同名本地分支指向相同对象时合并为一个目标分支。子模块 detached HEAD 保存为 `__worktree_HEAD__` 分支，其他历史引用保存于 `__archive_refs__/` 下。另将仅由 reflog 保留的一条历史提交保存于 `__archive_refs__/reflog/` 下。

| 来源仓库 | 归档分支 | 版本标签 |
| --- | ---: | ---: |
| `upstream_repos/dependencies/dexe_interfaces` | 30 | 14 |
| `upstream_repos/dependencies/magic_enum` | 1 | 24 |
| `upstream_repos/dexe_robot_server` | 47 | 15 |
| `upstream_repos/dexe_robot_server/dexe_interfaces` | 29 | 14 |
| `upstream_repos/dexe_robot_client` | 32 | 22 |
| `dexe_robot_server` | 38 | 11 |
| `dexe_robot_server/dexe_interfaces` | 17 | 10 |

## 使用

```bash
git clone https://github.com/ksjshssjj02929283-rgb/Dex-w1.git
cd Dex-w1
git switch dexe_robot_server/dev_w1
```

切换到来源分支后，仓库根目录展示该来源仓库的文件。`main` 分支提供索引与完整的 [manifest.json](manifest.json) 引用映射。

历史中的 `.gitmodules` 和 Git 子模块引用保持原样，可能仍指向原来的服务地址。两个子模块的完整已提交历史另有各自前缀分支保存；离线恢复时可从本仓库取回对应提交。

## 归档范围

归档仅包含已有 Git 对象与引用，没有从原始上游服务器获取新提交。未提交修改、未跟踪文件、Git 管理之外的工作区目录和构建产物不在本次 Git 历史归档中。开发目录 `dexe_robot_server` 的 26 条已跟踪文件变更保留在本地。

## 历史凭据清理

三份历史 `.gitmodules` 文件含同一 GitLab Deploy Token。上传副本已移除 URL 中的认证信息，未裁剪任何提交、分支或版本标签。本地原仓库保持原样。

此次清理更新了 735 个提交和 91 个分支/标签引用。`manifest.json` 保留原始 `object` 并增加 `published_object`；[history-map.json](history-map.json) 提供完整的原始到发布哈希映射。
