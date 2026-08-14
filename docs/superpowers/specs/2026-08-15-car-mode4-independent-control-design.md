# Car_F Mode4 独立同构控制设计

## 目标

将 `car_mode4.c` 从空实现重建为与当前 `car_mode2.c` 行为一致的世界坐标差速控制。Mode4 不调用 Mode2，不共享 PID、控制状态或可调参数，仅继续使用底盘公共反馈、目标和电机输出接口。

保留 `car_mode.c` 中已有的 Mode0 修改，并将 Mode4 分支从临时调用 `car_mode2_update_100HZ()` 改为调用 `car_mode4_update_100HZ()`。

## 控制实现

Mode4 完整复制 Mode2 的控制流程，并将内部常量、静态函数、PID、目标 yaw、制动状态和大角度转弯状态统一改为 `MODE4_*`、`mode4_*` 和 `s_mode4_*` 名称。

Mode4 使用 CH0/CH1 计算世界方向与目标速度，保留航向投影、yaw PD、gyro-z PI/FF、左右轮速度 PID、回中制动前馈及 BRAKE/PIVOT/EXIT 大角度状态机。所有阈值和固定常量与当前 Mode2 一致。

Mode4 不提供 Mode2 的车体速度入口和诊断接口，因为当前调度只需要世界坐标 `car_mode4_update_100HZ()`。

## 独立参数

Mode4 新增 14 项参数，默认值、步长和限幅与 Mode2 对应项一致：

- 左右轮速度 PID：`mode4_speed_left_kp/ki/kd`、`mode4_speed_right_kp/ki/kd`
- 速度公共参数：`mode4_speed_filter_alpha`、`mode4_speed_ff_static`
- gyro-z 参数：`mode4_gyroz_kp/ki/kff/k_turn`
- yaw 参数：`mode4_yaw_kp/kd`

车速滤波仍由 `car_loop.c` 统一执行，但在 Mode4 运行时选用 `mode4_speed_filter_alpha`，其他模式继续使用现有 `car_speed_filter_alpha`。这样避免复制编码器采集流程，同时保证 Mode4 滤波参数独立。

## 菜单与 Flash

本地车端参数总数由 14 增至 28。菜单在 `Car` 下增加 `Mode2` 和 `Mode4` 两组，每组分别显示 `L_Speed`、`R_Speed`、`Speed Com`、`Yaw Angle` 和 `Yaw Rate`，参数索引严格对应注册顺序。

继续使用现有菜单 Flash 顺序存档格式。原有 14 项保持在前，Mode4 的 14 项追加在后。现有 Flash 校验要求存档参数数量与当前目录完全一致，因此旧 14 项存档会判为无效，参数使用代码默认值；重新保存后写入完整 28 项。不增加旧存档兼容逻辑。

## 验证

- 前缀归一化比较 Mode2 与 Mode4 的世界坐标控制路径，确认固定常量和算法一致。
- 搜索确认 Mode4 不调用 Mode2，也不引用 Mode2 的 PID、状态或可调参数。
- 校验 28 个菜单索引、注册顺序、默认值、步长和限幅。
- 确认 Mode4 挡位调用 `car_mode4_update_100HZ()`，并保留现有 Mode0 修改。
- 检查 Flash 页面容量与旧 14 项存档加载边界，运行 `git diff --check`。
- 按项目规则不从命令行构建 IAR，最终交付手工编译和实车验证要点。
