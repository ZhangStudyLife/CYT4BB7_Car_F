# 计划：三环统一 100Hz + yaw 外环固定锁 0

## 目标

把控制链路改成：

1. 四轮速度环继续保持 100Hz。
2. yaw 角度环从 25Hz 改为 100Hz。
3. yaw 角速度环从 50Hz 改为 100Hz。
4. yaw 角度环目标全流程固定为 `0.0f` rad，不再使用“当前角度作为保持目标”的航向保持逻辑，也不再使用 `car_rotate_target` 做手动旋转目标。
5. PID 参数按采样频率变化做数学等效处理。
6. 不新增小函数，不绕远路；能直接写在现有控制链路里的逻辑就直接写。

这里的 `0.0f` 指 `Control_GetYawAngle()` 所在坐标系的 yaw 零点。也就是说，使能后车辆会主动回到 IMU/姿态解算定义的 0 角，而不是锁住使能瞬间的当前角度。

## 当前计划中必须修正的坑

原计划大方向对，但有几个地方照做会出问题：

1. `control_yaw_angle_current`、`control_yaw_angle_output`、`control_yaw_rate_current` 不能直接删。`Menu/menu_config.c` 的 PID 诊断页还在显示它们，直接删会编译失败。
2. PID 频率换算漏了 `i_limit`。当前 `PositionalPID` 是 `integral += err`，积分限幅钳的是原始误差累加值，不是积分输出。频率提高后只缩 `ki` 不放大 `i_limit`，会改变积分饱和时间和最大积分输出。
3. `control_pid_apply_all()` 现在在 `Control_100Hz()` 里、轮速 PID 前调用。合并 yaw 环后，它必须提前到 yaw 角度环和 yaw 角速度环之前，否则 yaw 环本周期用不到最新调参。
4. `car_rotate_target` 的引用不止 `car_mode0.c` 和 `car_mode.c`，`car_mode1.c`、`car_mode2.c` 也有多处清零。要么全删干净，要么至少保证控制层完全不读它，不能只删两处装样子。
5. `cm7_0_isr.c` 实际路径是 `CYT4bb7_Car/project/user/cm7_0_isr.c`，不是 `project/code/user/cm7_0_isr.c`。

## PID 数学等效

当前 `PositionalPID_Update()` 的核心是：

```c
err = target - current;
integral += err;
derivative = err - prev_err;
output = kp * err + ki * integral + kd * derivative;
```

它没有显式 `dt`，所以调度频率变化时要把离散系数补回来。

设旧频率为 `f_old`，新频率为 `f_new = 100Hz`：

- `kp`：不变。比例项不依赖采样周期。
- `ki`：乘 `f_old / f_new`。频率越高，单位时间累加次数越多，单次积分权重要缩小。
- `kd`：乘 `f_new / f_old`。频率越高，单次误差差分越小，差分权重要放大。
- `i_limit`：乘 `f_new / f_old`。因为限幅钳的是 `integral` 原始累加值；为了保持相同积分饱和时间和相同最大积分输出，需要和 `ki` 反向缩放。
- `output_limit`：不变。它限制的是物理输出量，yaw 角度环输出角速度目标，yaw 角速度环输出旋转轮速修正量。

换算表：

| 环 | 旧频率 | 新频率 | `ki` 系数 | `kd` 系数 | `i_limit` 系数 | `output_limit` |
|---|---:|---:|---:|---:|---:|---|
| yaw 角度环 | 25Hz | 100Hz | x0.25 | x4.0 | x4.0 | 不变 |
| yaw 角速度环 | 50Hz | 100Hz | x0.5 | x2.0 | x2.0 | 不变 |
| 四轮速度环 | 100Hz | 100Hz | x1.0 | x1.0 | 原样 | 不变 |

推荐在 `control.c` 顶部只加两个采样比例宏，不新增函数：

```c
#define CONTROL_YAW_ANGLE_DT_SCALE (0.25f)  /* 25Hz -> 100Hz: f_old / f_new */
#define CONTROL_YAW_RATE_DT_SCALE  (0.5f)   /* 50Hz -> 100Hz: f_old / f_new */
```

然后在 `control_pid_init_all()` 和 `control_pid_apply_all()` 两处统一用同一套换算：

```c
yaw_angle_pid.ki = yaw_angle_ki * CONTROL_YAW_ANGLE_DT_SCALE;
yaw_angle_pid.kd = yaw_angle_kd / CONTROL_YAW_ANGLE_DT_SCALE;
yaw_angle_pid.i_limit = yaw_angle_i_limit / CONTROL_YAW_ANGLE_DT_SCALE;
yaw_angle_pid.output_limit = yaw_angle_output_limit;

yaw_rate_pid.ki = yaw_rate_ki * CONTROL_YAW_RATE_DT_SCALE;
yaw_rate_pid.kd = yaw_rate_kd / CONTROL_YAW_RATE_DT_SCALE;
yaw_rate_pid.i_limit = yaw_rate_i_limit / CONTROL_YAW_RATE_DT_SCALE;
yaw_rate_pid.output_limit = yaw_rate_output_limit;
```

`kp_1`、`kp_2` 不做频率缩放。当前 yaw 两个 PID 的 `kp_2` 初始化为 `0.0f`，继续保持。

## 控制链路设计

合并后，`Control_100Hz()` 是唯一控制入口，顺序必须固定：

1. 应用最新 PID 参数，包含 yaw 换算后的参数。
2. 读取当前 yaw 角。
3. yaw 角度环：目标固定 `0.0f`，输出 yaw 角速度目标。
4. yaw 角速度环：目标来自角度环输出，反馈来自 `g_imufilter_1000hz.gyroz`，输出旋转修正量 `rot`。
5. 麦克纳姆四轮解算。
6. 四轮速度环 + 前馈 + 电机输出。

核心代码形态应接近这样：

```c
void Control_100Hz(float forward, float strafe)
{
    control_pid_apply_all();

    control_yaw_angle_current = Control_GetYawAngle();
    control_yaw_angle_output = PositionalPID_Update(&yaw_angle_pid,
                                                    0.0f,
                                                    control_yaw_angle_current);
    control_yaw_rate_target = control_yaw_angle_output;

    control_yaw_rate_current = -g_imufilter_1000hz.gyroz * CONTROL_DEG_TO_RAD;
    control_yaw_rate_output = PositionalPID_Update(&yaw_rate_pid,
                                                   control_yaw_rate_target,
                                                   control_yaw_rate_current);

    float rot = control_yaw_rate_output;
    float lf = forward - strafe - rot;
    float rf = forward + strafe + rot;
    float lr = forward + strafe - rot;
    float rr = forward - strafe + rot;

    /* 后续四轮反馈、前馈、轮速 PID、电机输出保持现有逻辑 */
}
```

这里不用 `PositionalPID_Update(&yaw_angle_pid, yaw_err, 0.0f)`，直接写 `target=0.0f, current=control_yaw_angle_current`，语义更干净：外环目标就是死锁 0。

`Control_GetYawAngle()` 可以继续负责角度归一化。为了不保留多余小函数，建议把原 `control_normalize_angle_rad()` 的两段 `while` 直接内联进 `Control_GetYawAngle()`：

```c
float Control_GetYawAngle(void)
{
    float yaw = -g_euler.yaw * CONTROL_DEG_TO_RAD;

    while (yaw > CONTROL_PI) yaw -= CONTROL_TWO_PI;
    while (yaw < -CONTROL_PI) yaw += CONTROL_TWO_PI;

    return yaw;
}
```

## 文件修改清单

### 1. `Controller/control.h`

修改头部架构注释：

```c
/* 串级PID控制模块
 *   100Hz -> yaw角度环(目标0) + yaw角速度环 + 四轮速度环
 */
```

删除这些接口声明：

- `Control_YawHoldReset`
- `Control_25Hz`
- `Control_50Hz`

保留这些调试变量声明，因为菜单诊断页在用：

- `control_yaw_angle_current`
- `control_yaw_angle_output`
- `control_yaw_rate_target`
- `control_yaw_rate_current`
- `control_yaw_rate_output`

删除 `control_yaw_rate_raw` 声明。它不参与新控制链路，当前菜单也没显示它。

### 2. `Controller/control.c`

删除航向保持状态：

- `s_yaw_hold_target`
- `s_last_rotate_active`
- `s_yaw_hold_active`

删除旧分频控制函数：

- `Control_YawHoldReset()`
- `Control_25Hz()`
- `Control_50Hz()`

删除 `control_yaw_rate_raw` 定义和赋值。

保留并更新诊断变量：

- `control_yaw_angle_current = Control_GetYawAngle();`
- `control_yaw_angle_output = yaw 角度环输出;`
- `control_yaw_rate_target = control_yaw_angle_output;`
- `control_yaw_rate_current = yaw 角速度反馈;`
- `control_yaw_rate_output = yaw 角速度环输出;`

调整 `control_pid_init_all()`：

- wheel PID 参数不变。
- yaw 角度环使用 25Hz -> 100Hz 换算后的 `ki/kd/i_limit`。
- yaw 角速度环使用 50Hz -> 100Hz 换算后的 `ki/kd/i_limit`。

调整 `control_pid_apply_all()`：

- wheel PID 参数不变。
- yaw 两个 PID 的实时调参也必须使用同样的频率换算。
- 不允许 `Control_25Hz()`、`Control_50Hz()` 里再各自偷偷覆盖 PID 参数，因为这两个函数要删掉。

调整 `Control_Reset()`：

```c
void Control_Reset(void)
{
    control_pid_init_all();
    control_yaw_angle_current = 0.0f;
    control_yaw_angle_output = 0.0f;
    control_yaw_rate_target = 0.0f;
    control_yaw_rate_current = 0.0f;
    control_yaw_rate_output = 0.0f;
}
```

调整 `Control_Stop()`：

```c
void Control_Stop(void)
{
    Control_Reset();
    mecanum_motor_stop();
}
```

重写 `Control_100Hz()`：

- 一进函数先 `control_pid_apply_all()`。
- yaw 角度环、yaw 角速度环、四轮速度环都在这里跑。
- 不读取 `car_rotate_target`。
- 不判断“是否正在旋转”。
- 不设置“当前角度为保持目标”。
- 外环目标永远是 `0.0f`。

### 3. `Controller/car_loop.c`

删除全局变量：

- `timer_50HZ_flag`
- `car_rotate_target`

`car_loop_runtime_reset()` 删除：

- `timer_50HZ_flag = 0U;`
- `car_rotate_target = 0.0f;`

`car_loop_100HZ()` 中控制部分改成：

```c
if (car_control_enabled != 0U)
{
    Control_100Hz(car_forward_target, car_strafe_target);
}
else
{
    Control_Stop();
}
```

删除额外的 `Control_YawHoldReset()` 调用。

删除整个 `car_loop_50HZ()`。

`car_loop_25HZ()` 只保留非控制任务：

```c
static void car_loop_25HZ(void)
{
    ALX_AOA_Update_25HZ(s_system_time_ms);
    car_mode_update_25HZ(s_system_time_ms);
}
```

`car_loop_poll()` 删除 50Hz 块：

```c
if (timer_25HZ_flag)
{
    timer_25HZ_flag = 0U;
    car_loop_25HZ();
}

if (timer_100HZ_flag)
{
    timer_100HZ_flag = 0U;
    car_loop_100HZ();
}
```

### 4. `Controller/car_loop.h`

删除：

- `extern volatile uint8_t timer_50HZ_flag;`
- `extern float car_rotate_target;`

### 5. `CYT4bb7_Car/project/user/cm7_0_isr.c`

删除 50Hz 计数变量：

- `pit_ch0_50HZ_count`

删除 50Hz flag 设置块：

```c
pit_ch0_50HZ_count++;
if(pit_ch0_50HZ_count >= 20)
{
    pit_ch0_50HZ_count = 0;
    timer_50HZ_flag = 1;
}
```

保留：

- 1000Hz IMU tick
- 100Hz flag
- 25Hz flag

### 6. `Controller/car_mode.c`

删除所有 `car_rotate_target = 0.0f;`。

更新注释，把“mode_update 内部写 car_forward/strafe/rotate_target”改成“mode_update 只写 car_forward/strafe_target，yaw 由控制层锁 0”。

### 7. `Controller/car_mode0.c`

删除所有 `car_rotate_target = 0.0f;`。

更新文件头注释：

- 原来“旋转固定0”容易误解成角速度目标为 0。
- 改成“模式层只给前后/左右速度目标，yaw 由 100Hz 控制层固定回 0”。

### 8. `Controller/car_mode1.c`

删除：

- `car_rotate_target = 0.0f;`

更新注释：

- 原来“rotate_target 始终为0”改成“模式层不输出旋转目标，yaw 由控制层锁 0”。

### 9. `Controller/car_mode2.c`

删除所有 `car_rotate_target = 0.0f;`。

更新注释：

- 原来“rotate 始终为0”改成“模式层不输出旋转目标，yaw 由控制层锁 0”。

## 不改动的部分

- `pid.c` / `pid.h`：PID 算法不变，不加 `dt` 参数。
- 四轮速度环频率和参数不变。
- `ALX_AOA_Update_25HZ()` 继续 25Hz。
- `car_mode_update_25HZ()` 继续 25Hz，只负责更新 `forward/strafe` 目标。
- `menu_config.c` 的调参变量名不变。菜单里显示的 yaw 诊断变量继续保留。
- `yaw_angle_output_limit` 和 `yaw_rate_output_limit` 不做频率缩放。

## 验证计划

1. 全工程搜索确认没有残留旧接口：
   - `Control_25Hz`
   - `Control_50Hz`
   - `Control_YawHoldReset`
   - `timer_50HZ_flag`
   - `pit_ch0_50HZ_count`
   - `car_rotate_target`
   - `control_yaw_rate_raw`

2. 编译通过，重点防止这些文件报未定义：
   - `control.h`
   - `control.c`
   - `car_loop.c`
   - `car_mode*.c`
   - `menu_config.c`
   - `cm7_0_isr.c`

3. 上电静止时看诊断：
   - `YawCur` 接近 0。
   - `YawOut/RateT` 接近 0。
   - `RateC` 接近 0。
   - `RateO` 接近 0。

4. 手动把车体转离 0 角后松手：
   - `YawCur` 应逐步回到 0。
   - `YawOut/RateT` 应给出回正方向的角速度目标。
   - `RateO` 应驱动四轮产生回正旋转修正。

5. 遥控只给前后/左右平移，不给旋转：
   - 车辆可以平移。
   - yaw 仍主动回 0，不是保持当前角，也不是只保证角速度为 0。

6. 如果回正方向反了，只检查这两处符号，不要到处乱改：
   - `Control_GetYawAngle()` 里的 `-g_euler.yaw`
   - `control_yaw_rate_current = -g_imufilter_1000hz.gyroz * CONTROL_DEG_TO_RAD`

## 执行原则

- 不新增 helper 函数。
- 不改 PID 算法文件。
- 不让 25Hz/50Hz 再参与 yaw 控制。
- yaw 外环目标只能是 `0.0f`。
- 所有 yaw PID 参数更新集中在 `control_pid_init_all()` 和 `control_pid_apply_all()`。
- 旧旋转目标链路要么删干净，要么就绝不能再被控制层读取；本计划按删干净处理。
