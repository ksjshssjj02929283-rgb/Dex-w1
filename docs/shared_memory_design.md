# dexe_robot_server 共享内存设计

共享内存定义位于 [`w1_server/utils/shared_memory_def.h`](../w1_server/utils/shared_memory_def.h)。它是 dexe_robot_server 与 dexe_ethercat 之间的实时数据平面，ROS 接口、规划状态机和硬件进程不直接互相调用。

## 1. 设计理念

共享内存设计优先保证实时控制的确定性：

1. **控制与反馈分离**：硬件反馈和 Server 控制分别使用独立共享内存，两个方向互不争用，也能单独判断哪一侧失效。
2. **最新值优先**：读端调用 `PeekLatest()` 直接读取最新完整帧，不要求逐帧消费。1 kHz 闭环关心当前状态和当前指令，旧帧即使未读取也允许被跳过。
3. **固定内存、无动态分配**：共享区使用固定容量、固定大小的 POD 数据结构，运行时不申请内存，不依赖序列化，适合高频线程。
4. **无进程间互斥锁**：读写协调只依赖原子索引和 CAS。写端不会阻塞等待读端；只有即将覆盖正在读取的槽位时，本次写入才返回 `false`。
5. **完整帧发布**：写端先在目标槽位中写完数据、计数和时间戳，最后再原子发布最新索引，避免读端看到只写了一半的数据。
6. **主动判定过期**：每帧带有单调时钟时间戳。即使共享内存仍然存在，只要数据超过有效期，也会被视为无效，避免把硬件进程停止前的旧数据继续用于控制。

因此，这套结构应理解为**跨进程 Latest Buffer**，而不是保存完整历史记录的普通消息队列。容量 8 主要用于隔离读写时序和吸收短时调度抖动，不代表读端需要依次处理 8 帧。

## 2. 双向通道与数据布局

真机使用两段 System V 共享内存：

| 共享内存 | Key | 单帧类型 | 写端 → 读端 | 主要内容 |
|---|---:|---|---|---|
| `FeedbackSharedMemory` | `0x1005` | `RobotFeedback` | dexe_ethercat → Torque Thread / ChassisNode | 机器人状态字、EtherCAT AL 状态、最多 32 个关节的位置/速度/电机力矩/传感器力矩/温度、底盘反馈和 IMU |
| `ControlSharedMemory` | `0x2005` | `RobotControl` | Torque Thread → dexe_ethercat | 机器人控制字、最多 32 个关节的位置/速度/`joint_torque_feedforward`/力矩限制、差速或全向底盘指令 |

每段共享内存的布局为：

```text
ShmStruct
├── ring_buffer[8]
│   └── QueueElement
│       ├── data       一帧 RobotFeedback 或 RobotControl
│       ├── cnt        帧序号
│       ├── ts         CLOCK_MONOTONIC 写入时间
│       └── valid      槽位是否已写入
└── latest_and_reader  原子保存 latest 与 reader 两个 32 位索引
```

反馈帧中的关节顺序由硬件决定。Torque Thread 读取后通过关节名映射为 Server 内部固定的 20 自由度顺序；控制帧写入前再由 `SortControlByJointName()` 排回硬件顺序。名称映射将控制算法与 EtherCAT 从站排列解耦，但也意味着启动时必须验证关节名是否完整且唯一。

## 3. 读写一致性

写入 `AppendNewElement()` 的流程：

1. 原子读取 `latest_and_reader`，计算下一个环形槽位。
2. 如果该槽位可能覆盖读端当前占用的槽位，立即返回 `false`，不等待、不覆盖。
3. 将完整数据、递增帧序号、单调时钟时间戳和 `valid=true` 写入槽位。
4. 使用 CAS 更新 `latest`，把新帧一次性发布给读端。

读取 `PeekLatest()` 的流程：

1. 使用 CAS 将当前 `latest` 同时登记为 `reader`，临时占用该槽位。
2. 检查槽位是否写入过，并检查时间戳是否超时。
3. 数据有效时复制完整帧，并用槽位帧序号更新输出结构的 `cnt`。
4. 将 `reader` 恢复为无效值，释放槽位。

`latest` 和 `reader` 被打包在同一个 64 位原子变量中，避免分别更新两个索引产生不一致快照。索引达到容量的 100 倍后会在安全条件下回绕，既保持取模后的槽位不变，也避免计数长期增长。

## 4. 有效性与故障处理

代码中的默认数据有效期为 **10 ms**，可通过 `SetDataValidTimeoutMs()` 修改，设置为 `-1` 时关闭超时检查。`PeekLatest()` 在以下情况返回 `false`：

- 共享内存还没有收到第一帧。
- 最新槽位尚未标记为有效。
- 最新帧超过有效期。

Torque Thread 在此基础上还会检查 `RobotFeedback.cnt > 0`、关节名称映射和机器人状态字。只有共享内存反馈有效且电机处于 Ready/Enabled 状态时，才生成 `JointRawMeasurement`；否则重置测量缓冲区，使 Planner Manager 回到 `Init`，避免使用陈旧或错误反馈继续控制。

控制侧在每个 1 kHz 周期完成模式处理后，先复制 `RobotControl`、按硬件关节名排序并更新时间戳，再写入 `ControlSharedMemory`。dexe_ethercat 始终读取最新控制帧，不追赶已经过期的控制指令。

## 5. 使用约束

- 每段共享内存按“单写端、读取最新值”的模型设计，不应增加第二个写进程。
- `latest_and_reader` 只记录一个活动读者，同一共享段在任一时刻只能有一次 `PeekLatest()` 占用槽位。`FeedbackSharedMemory` 当前同时服务 Torque Thread 和 ChassisNode；如需保证多读者并发，建议由一个线程读取后通过进程内 Latest Buffer 分发，或扩展为独立的多读者索引。
- Key 固定为 `0x1005` 和 `0x2005`，同一主机不适合并行运行多套真机实例；如需多实例，应先将 Key 参数化。
- 共享内存以 `0666` 权限创建，部署时应结合运行用户和主机安全策略限制非预期进程访问。
- 进程退出时只执行 `shmdt()`；仅当最后一个附着进程退出后才通过 `IPC_RMID` 删除共享段。进程异常退出后应检查是否残留旧共享内存。
- 启动真机前必须检查共享内存连接、反馈帧计数、更新时间、关节名称及 Robot Ready 状态，不能仅以共享内存段存在作为硬件在线依据。
