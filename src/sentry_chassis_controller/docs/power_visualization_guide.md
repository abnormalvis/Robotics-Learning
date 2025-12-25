# 功率可视化与标定指南

本文档汇总了功率模型的物理推导、实验标定步骤，以及答辩时的简明陈述要点，便于在调试与答辩中使用。

## 1 物理推导（精要）

我们在代码中使用的经验模型形式为：

$$P(s)=a\cdot s^2 + b\cdot s + c$$

其中通过对单轮或全车求和得到：
- 控制指令缩放：$cmd'_i = s\cdot cmd_i$。
- 机械做功项：$P_{mech}=\sum_i |T_i\cdot \omega_i|$，若扭矩 $T\propto cmd$ 则出现 $\sum |cmd_i\cdot v_i|$ 的一次项（这里 $v_i$ 代表轮端线速度）。
- 电热损耗（铜损）近似与电流平方成正比：$P_{cu}\propto I^2R$。因为 $T\propto I$，若 $cmd\propto T$，则铜损可等效表示为 $\alpha\cdot cmd^2$，这给出了二次项 $a\cdot s^2$ 的物理来源。
- 转速相关损耗（铁损、风阻、部分机械摩擦）常随转速的幂次增长，工程上常近似为 $\beta\cdot v^2$，这对应模型中的 $c$ 中的速度平方项。

因此可以将系数解释为：
- $a = k_{effort}\cdot \sum cmd_i^2$：等效放大电流/扭矩相关的平方损耗（铜损为主）；
- $b = \sum |cmd_i\cdot v_i|$：机械做功的线性耦合项；
- $c = k_{vel}\cdot \sum v_i^2 - P_{offset} - P_{limit}$：速度相关的耗散与偏置合并项（铁损、风阻等）。

注意：这是经验模型，整合了多种物理效应。真实电机损耗更加复杂（与温度、磁滞、效率曲线相关），但该模型在在线实时计算与控制中足够实用。

## 2 标定（实验步骤与回归拟合）

目标：通过实验数据拟合出可用的 $k_{effort}$ 与 $k_{vel}$，使得模型在实际工况下给出合理的 scaling_factor。

推荐步骤：

1. 数据准备：
   - 在机器人或测试台上准备若干工况组合 (cmd, speed)。建议覆盖低/中/高三档 cmd 和低/中/高 三档速度，总计 9~20 组左右。
   - 对每组工况记录：控制命令 `cmd_i`（每轮或汇总）、轮端速度 `v_i`、整车实时功率 P_meas（可以通过测量电源侧电压×电流或读取电机驱动的电流/电压计算）。
   - 使用 rosbag 或 CSV 记录以下话题/变量：`/power_debug`（代码中已发布的辅助量）、`/motor_current`（若可得）、`/odom`（速度）、时间戳。

2. 数据整理与特征构造：对每条记录计算：
   - S_cmd2 = \sum_i cmd_i^2
   - S_cmdvel = \sum_i |cmd_i\cdot v_i|
   - S_vel2 = \sum_i v_i^2

3. 线性回归拟合：在 s=1 的前提下对测得功率做拟合：

$$P_{meas} \approx A\cdot S_{cmd2} + B\cdot S_{cmdvel} + C\cdot S_{vel2} + D$$

   - 用最小二乘（numpy.linalg.lstsq 或 sklearn.linear_model.LinearRegression）求解 A,B,C,D。
   - 解释：A 对应 k_effort 的等效份额（注意尺度因子与单位），C 对应 k_vel 的等效份额，B 对应做功能量耦合。

4. 映射回控制参数：根据模型中 a,b,c 的定义，把拟合得到的 A,C 映射为 `effort_coeff` 与 `velocity_coeff` 的初始值（视实现可能需要缩放或单位转换）。

5. 验证与微调：
   - 在未参与拟合的工况下验证 scaling_factor 是否与观测的功率越界一致；
   - 使用 `rqt_reconfigure` 在线调整 `effort_coeff`、`velocity_coeff` 和 `power_limit`，并用 `rqt_plot` 观察 `/power_debug` 的字段（a,b,c,scaling_factor 等）。

简单的 Python 回归示例（离线，示意）：

```python
import numpy as np
from sklearn.linear_model import LinearRegression

# X columns: [S_cmd2, S_cmdvel, S_vel2, 1]
X = np.column_stack([S_cmd2, S_cmdvel, S_vel2, np.ones(len(S_cmd2))])
y = P_meas
model = LinearRegression(fit_intercept=False).fit(X, y)
coef = model.coef_  # [A, B, C, D]
print('A,B,C,D =', coef)
```

## 3 答辩中可用的简短陈述（2~3句）

“我们使用一个二次经验模型来近似整车功率：二次项代表与电流或扭矩平方相关的电热损耗（铜损），一次项代表扭矩与速度的做功能量，而速度平方项代表铁损、风阻等与转速相关的损耗。通过在不同 (cmd, speed) 工况下做回归拟合得到 `effort_coeff` 与 `velocity_coeff` 的初始值，然后在运行中使用 `rqt_reconfigure` 微调，保证 scaling_factor 在极端工况下能正确触发限幅。”

可选补充（如被问到具体如何量测）：
- 使用电源测量电压与电流或驱动器的电流回读结合速度编码器，按时间对齐做离线回归；验证时重点看 `scaling_factor<1` 的时刻与实际功率峰值是否一致。

## 问题 A — “为什么缩放因子应该接近 1？若 > 1 怎么办？”

理由：
理想情况下，缩放因子 s 表示“把当前命令按比例缩小后刚好耗尽允许功率”。当系统在正常工作范围内且功率富余时，不需缩放 → s ≥ 1（表示放大也可满足等式，但我们无法放大命令，实际上应把 s 限定为 1）。
因此在运行中我们期望 s ≈ 1（既不需要限制，也不产生误判），若 s 明显小于 1 表示现在命令太大或功率阈值太小。

当 s > 1 时的含义：
数学上说明以当前参数替换进方程时，等式在 s=1 时已经满足（F(1) = a + b + c ≤ 0），或者求得的根大于 1（代表理论上需要放大命令才能到达边界）。但是我们只允许缩小命令，因此 s>1 没意义。
处理方法（工程实践）：
把 s 限定到不超过 1： scaling = std::min(1.0, scaling);
如果你看到 s 因数略大于 1（例如 1.02），直接 clamp 到 1 并忽略（可能是舍入/建模误差）。
若频繁出现 s≫1（比如 2、10），反而提示模型参数（a,b,c）或单位尺度不一致（单位/量纲错误）——需要检查 cmd 单位、effort_coeff、velocity_coeff、power_limit 的单位和定义是否一致。


## 问题 B — “在 rqt_plot 里看到缩放因子接近 0 但仍大于 0 且小于 1，这说明什么？如何调整？”

含义（物理 & 数学）：
s ≈ 0（但 >0）表示方程要求将命令强烈压低才能满足功率限制：当前命令和速度综合产生的“预测功率”远超 allowed（power_limit）。换言之，a,b,c 的值导致根很小。

可能原因：
power_limit 太小（目标功率设定低）；
effort_coeff 或 velocity_coeff 过大（模型对功耗估计过高）；
cmd 值过大（控制器输出力矩/速度命令过高）或速度 vel 很大（快速运动导致速度平方项大）；
单位不匹配（例如 cmd 单位应为 N·m，但被当成其他量）；
瞬态脉冲/步变：短时间内突然大的 steering/acceleration 导致 Σ(cmd²) 很大。

