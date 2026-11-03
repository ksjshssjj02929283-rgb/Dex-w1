# W1 崩溃记录与离线堆栈调查

## 运行行为

重新编译、安装 `robot_server_lib` 和 `w1_server` 后，仍使用 `run_w1.sh` 启动。
程序在创建 ROS 节点前启用崩溃记录，路径为：

```text
~/log/crash/w1_crash_<秒时间戳>_<PID>_<纳秒>.log
```

每次启动产生一个独立文件，权限为 0600。文件包含启动版本/编译时间、实际可执行文件路径、
内核版本、core 限额、`kernel.core_pattern`、启动及节点初始化后的内存映射和已注册线程的 TID。
目前没有自动清理策略，应随设备日志维护一起归档/清理。

`SIGSEGV`、`SIGBUS`、`SIGABRT`、`SIGILL`、`SIGFPE` 触发时，handler 直接向已打开的文件
写一条 `CRASH` 记录，包括 PID/TID、信号及 `si_code`、有效时的故障地址，以及原始 PC/SP/FP
（ARM64 还记录 LR）。它不依赖 NanoLog、ROS、字符串分配或在线符号解析。
记录后恢复默认动作，向当前线程重新递送同一信号，保留信号退出状态及生成 core 的机会。
不以 `_exit()` 作为正常退出路径，不在 handler 内刷新业务日志。

主线程保持名字 `w1_server`，避免破坏 `run_w1.sh` 中的 `pgrep -x w1_server`。
主要业务线程命名为 `w1_torque`、`w1_planner`、`w1_publish`，日志线程为 `w1_log`；
为它们分别安装带保护页的 128 KiB 备用信号栈，在线程提升实时优先级前完成分配/预触页。
初始化未调用时，这些注册接口不做操作，保持 sim_server 原有行为。

ROS executor/DDS 等第三方创建的线程同样适用进程级信号处理，但未为它们注入备用信号栈。
因此普通内存访问故障可以记录，第三方线程栈溢出时则不能保证 handler 能运行。
`SIGINT`、`SIGTERM` 的现有 ROS 行为不变；`SIGKILL`、整机掉电、RCU stall 不会生成此类记录。

## Core 保存的前提

初始化仅调整本进程：将 soft core limit 提升到现有 hard limit，调用 `PR_SET_DUMPABLE(1)`，
处理启动脚本 `setcap` 导致不可转储的情况。后者也影响同用户调试/读取进程内存的权限。
不修改全局 sysctl、系统 core 收集器、systemd 服务或其他进程。

日志中出现 `CRASH` 不等于 core 一定保存成功。磁盘空间、权限、hard limit、收集器规则都可能影响保存。
`write()` 不保证断电持久化，存储或内核严重卡死时也可能无法完成。

在目标设备检查：

```bash
cat /proc/sys/kernel/core_pattern
cat /proc/$(pgrep -x w1_server)/limits
```

- 若 pattern 是普通文件路径：检查对应目录、权限和限额。相对路径相对于崩溃进程的工作目录。
- 若 pattern 指向 systemd-coredump：使用 `coredumpctl list` / `coredumpctl info` / `coredumpctl dump`。
- 若 pattern 指向 Apport：按该机器 Apport 的收集规则找 core；不能直接假设 coredumpctl 可用。

core 可能包含运行时数据，应限制读取权限。不要在运行中的机器人上验证致命信号。

## 还原堆栈

CMake 保留默认 Release 优化，并为本项目编译代码增加 `-g -fno-omit-frame-pointer`；
允许外部指定其他 build type。发布时保留故障版本的可执行文件、动态库和调试符号，不能用后续重新编译的版本替代。
优化可能使部分局部变量不可见，外部无调试符号的库也不一定能显示源码行。

```bash
bash w1_server/tools/analyze_core.sh \
  /path/to/matching/w1_server /path/to/core > backtrace.txt 2>&1
```

脚本仅做离线分析，输出所有线程的 `bt full`、映射和当前线程寄存器；不附加运行中的机器人。
限制 GDB 单个符号处理工作线程、1 GiB 地址空间和 120 秒 CPU 时间；不自动加载 core 关联脚本。
大型真实 core 可能超过此上限，届时应转移到有足够内存的调试机分析。

先用 `CRASH` 行里的 TID 找对应线程，再对照 PC 和 core 调用栈。`CRASH` 是现场地址记录，
不是已解析的完整 backtrace；完整栈依赖 core 与匹配的二进制/符号。

## 独立测试（不启动 ROS/机器人）

```bash
bash w1_server/tools/validate_crash_diagnostics.sh
```

仅编译 `crash_probe.cpp` 和 `crash_diagnostics.cpp`，串行运行，不执行 build.sh/run_w1.sh。
编译链每个进程地址空间上限 512 MiB；每个测试子进程上限 256 MiB、CPU 5 秒、墙钟 15 秒。
重复测试将 core hard limit 设为 0，避免批量生成 core；不修改全局 core 配置。

覆盖 13 种场景：主线程段错误、工作线程段错误、未注册线程段错误、工作线程栈溢出、
持有应用锁时崩溃、abort、未捕获异常、BUS/ILL/FPE，以及不应拦截的 TERM/INT/KILL。
断言退出信号不变、崩溃记录数量、PC 非零、工作线程 TID、文件权限以及已注册线程实际使用备用栈。
持锁测试模拟的是应用锁不可重入，不是人为冻结真实 NanoLog 线程；测试程序完全不初始化 NanoLog，
验证了崩溃通道不依赖其运行。

## 本次验证范围与资源限制

- x86_64 本机：13 个信号场景通过。
- 使用独立 probe，经 GDB 递送原始 SIGSEGV、执行 handler 后保存 core；
  离线读取可还原 `CrashLeaf -> CrashMiddle`、源码行号、主线程 join 栈，记录的 PC 与故障 PC 相符。
  这是 GDB 生成的 core，**不是目标系统自动 core 收集的验收**。
- ARM64 寄存器提取分支已实现，但未在 Jetson 实机运行验证。
- 初次无独立内存限额的构建遇到桌面 OOM，随后 1.5 GiB 地址空间限制下的语法检查也未完成。
  2026-11-02 改用下面的独立资源限制后，`robot_server_lib`、`sim_server`、`w1_server`
  三个包完整编译/链接通过，附带的 `feedback` 可执行文件也通过，总用时 16 分 21 秒。
- W1 产物已静态确认包含 `Initialize`、`RegisterCurrentThread`、`HandleFatalSignal`，
  以及 `.debug_info`、`.debug_line`、符号表。未启动机器人或覆盖现有安装目录。

成功构建采用与 `build.sh` 一致的 Release 配置、关闭力控仿真和编译缓存，另作如下资源设置：

```text
colcon --executor sequential
MAKEFLAGS=-j1
CMAKE_BUILD_PARALLEL_LEVEL=1
COLCON_PARALLEL_WORKERS=1
CMAKE_CXX_FLAGS=--param=ggc-min-expand=10 --param=ggc-min-heapsize=32768 -ftrack-macro-expansion=0
```

GCC 参数降低编译时垃圾回收阈值并关闭宏诊断展开历史，保持 `-O3` 和 `-g`。
构建运行在独立 systemd 用户服务内，`MemoryMax=4G`、`MemorySwapMax=0`、`OOMPolicy=stop`、
`OOMScoreAdjust=500`、`TasksMax=32`、`Nice=10`、`RuntimeMaxSec=1800`。
先前仅设 3 GiB 上限时，单个 `pinocchio_api.cpp` 编译进程触及限额，只有构建服务被终止；
成功的 4 GiB 构建峰值达到限额并通过回收继续，服务内 `oom_kill=0`。

此次临时产物和记录：

```text
/tmp/w1-crash-colcon-install/w1_server/lib/w1_server/w1_server
/tmp/w1-crash-serial-log/latest_build/
/tmp/w1-crash-build-serial.sh
W1 ELF Build ID: 13c58868f2a5ba3fb2b574d51e6d315262d3b7bc
```

这验证的是当前 x86_64 环境的编译/链接，Jetson 自动保存 core 和 ARM64 实机行为仍需单独验收。

后续整项目构建应在资源受控的独立环境完成，至少采用包级/编译级单并发和专属 cgroup 内存上限；
仅设置 `-j1` 或 `nice` 不能提供内存隔离。不要为复测而关闭 OOM 防护或重复无上限构建。
