# LQR 线性二次型调节器

LQR（Linear Quadratic Regulator，线性二次型调节器）是一类**针对线性系统、通过最小化二次型代价函数来设计最优反馈控制**的方法。它在理论上非常优雅，在工程上广泛应用于：

- 轨迹跟踪（机器人底盘、机械臂、无人机）
- 姿态稳定（倒立摆、平衡车、四旋翼）
- 振动抑制（结构控制、主动悬挂）
- 需要多目标权衡（速度 vs 能耗、精度 vs 平滑度）

LQR 的核心优势：

1. **自动平衡多个目标**：通过权重矩阵 $Q,R$ 显式描述"状态误差"和"控制代价"的重要性
2. **保证稳定性**：闭环系统自动稳定（前提是系统可控）
3. **易于调参**：调 $Q,R$ 比手调 PID 的 $K_p,K_i,K_d$ 更系统化
4. **可扩展性强**：从 SISO 到 MIMO、从调节到跟踪、从离散到连续都有成熟方法

下面按"问题定义 → 数学推导 → 工程实现 → 调参经验 → 扩展应用"展开。

---

## 1. 核心问题：什么时候用 LQR？

考虑一个线性系统：

$$
\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}
$$

或离散形式：

$$
\mathbf{x}_{k+1} = A_d\mathbf{x}_k + B_d\mathbf{u}_k
$$

- $\mathbf{x}$：状态向量（例如位置、速度、角度、角速度）
- $\mathbf{u}$：控制输入（例如力、力矩、电压）
- $A,B$（或 $A_d,B_d$）：系统矩阵

LQR 回答的问题：**如何选择控制律 $\mathbf{u}=-K\mathbf{x}$，让系统稳定，并且"既要快又要省力"？**

具体来说，它最小化一个二次型代价函数（以无限时域为例）：

$$
J = \int_0^\infty \left(\mathbf{x}^\top Q\mathbf{x} + \mathbf{u}^\top R\mathbf{u}\right)dt
$$

- $Q\geq 0$：状态权重矩阵（对角线表示各状态误差的惩罚）
- $R>0$：控制权重矩阵（对角线表示各控制量的代价）

### 1.1 直观理解代价函数

- **$\mathbf{x}^\top Q\mathbf{x}$**：你希望状态误差尽量小（例如位置误差、速度误差）
- **$\mathbf{u}^\top R\mathbf{u}$**：你希望控制量尽量小（省能量、减少磨损、避免饱和）

权重的作用：

- $Q$ 大：更关心状态误差，允许更大的控制输出 → 响应更快但更"猛"
- $R$ 大：更关心控制代价，限制控制输出 → 响应更柔和但可能慢

### 1.2 为什么叫"二次型"？

因为代价函数里 $\mathbf{x}^\top Q\mathbf{x}$ 和 $\mathbf{u}^\top R\mathbf{u}$ 都是二次型（平方项）。这使得问题可以解析求解（得到 Riccati 方程）。

---

## 2. LQR 解析解：Riccati 方程 + 反馈增益

对于连续时间无限时域 LQR，最优控制律是**线性状态反馈**：

$$
\mathbf{u}^* = -K\mathbf{x}
$$

其中反馈增益矩阵：

$$
K = R^{-1}B^\top P
$$

$P$ 是代数 Riccati 方程（ARE）的解：

$$
A^\top P + PA - PBR^{-1}B^\top P + Q = 0
$$

### 2.1 离散时间 LQR（更常用）

对于离散系统 $\mathbf{x}_{k+1} = A_d\mathbf{x}_k + B_d\mathbf{u}_k$，离散 Riccati 方程（DARE）：

$$
P = A_d^\top P A_d - A_d^\top P B_d(R + B_d^\top P B_d)^{-1}B_d^\top P A_d + Q
$$

反馈增益：

$$
K = (R + B_d^\top P B_d)^{-1}B_d^\top P A_d
$$

最优控制律仍然是：

$$
\mathbf{u}_k = -K\mathbf{x}_k
$$

### 2.2 工程上怎么算 $K$？

你不需要手算 Riccati 方程！各种工具都有现成函数：

- **MATLAB/Octave**：`K = lqr(A, B, Q, R)` 或 `K = dlqr(Ad, Bd, Q, R)`
- **Python (control library)**：`K, S, E = control.lqr(A, B, Q, R)` 或 `dlqr`
- **C++/嵌入式**：可以离线算好 $K$ 固化到代码，或用轻量级求解器（如 DARE/CARE）

---

## 3. 一个简单例子：倒立摆稳定（二阶系统）

### 3.1 系统模型

考虑倒立摆线性化后（小角度近似）：

$$
\ddot{\theta} = \frac{g}{l}\theta + \frac{1}{ml^2}u
$$

状态定义：$\mathbf{x} = [\theta, \dot{\theta}]^\top$，控制 $u$ 是力矩。

状态空间：

$$
\begin{bmatrix}\dot{\theta}\\ \ddot{\theta}\end{bmatrix} = \begin{bmatrix}0 & 1\\ g/l & 0\end{bmatrix}\begin{bmatrix}\theta\\ \dot{\theta}\end{bmatrix} + \begin{bmatrix}0\\ 1/(ml^2)\end{bmatrix}u
$$

即 $A = \begin{bmatrix}0 & 1\\ g/l & 0\end{bmatrix}$，$B = \begin{bmatrix}0\\ 1/(ml^2)\end{bmatrix}$

### 3.2 选择 $Q,R$

假设：

- 更关心角度误差 $\theta$：$Q_{11}=100$
- 也关心角速度 $\dot{\theta}$：$Q_{22}=1$
- 控制代价适中：$R=0.1$

即：

$$
Q = \begin{bmatrix}100 & 0\\ 0 & 1\end{bmatrix},\quad R = 0.1
$$

### 3.3 求解 $K$（MATLAB 示例）

```matlab
A = [0, 1; g/l, 0];
B = [0; 1/(m*l^2)];
Q = [100, 0; 0, 1];
R = 0.1;
K = lqr(A, B, Q, R);
```

你会得到一个 $1\times 2$ 的增益矩阵，例如 $K = [k_1, k_2]$。

### 3.4 控制律

$$
u = -k_1\theta - k_2\dot{\theta}
$$

这实际上就是**PD 控制**，但参数是通过优化得到的！

---

## 4. 从调节到跟踪：LQR 跟踪参考轨迹

上面的 LQR 是**调节器**（Regulator），目标是把状态拉回原点 $\mathbf{x}=0$。

实际应用中，你往往想跟踪一个参考轨迹 $\mathbf{x}_r(t)$（例如期望位置、速度）。

### 4.1 误差反馈形式

定义误差：

$$
\mathbf{e}_k = \mathbf{x}_k - \mathbf{x}_{r,k}
$$

如果参考是常值或缓变，可以用误差作为新状态做 LQR：

$$
\mathbf{u}_k = -K\mathbf{e}_k = -K(\mathbf{x}_k - \mathbf{x}_{r,k})
$$

### 4.2 前馈 + 反馈（更通用）

对于动态参考，通常分解为：

$$
\mathbf{u}_k = \mathbf{u}_{ff,k} - K(\mathbf{x}_k - \mathbf{x}_{r,k})
$$

- $\mathbf{u}_{ff,k}$：前馈项（根据参考轨迹计算"理想输入"）
- $-K(\mathbf{x}_k - \mathbf{x}_{r,k})$：反馈项（修正误差）

例如，对于 $\mathbf{x}_{k+1} = A_d\mathbf{x}_k + B_d\mathbf{u}_k$，理想情况下：

$$
\mathbf{x}_{r,k+1} = A_d\mathbf{x}_{r,k} + B_d\mathbf{u}_{ff,k}
$$

若 $B_d$ 列满秩，可解出：

$$
\mathbf{u}_{ff,k} = B_d^\dagger(\mathbf{x}_{r,k+1} - A_d\mathbf{x}_{r,k})
$$

（$B_d^\dagger$ 是伪逆）

---

## 5. 工程实现：离散 LQR 控制循环

假设采样周期 $T_s$，控制循环结构：

```text
given: Ad, Bd, Q, R
offline: compute K = dlqr(Ad, Bd, Q, R)

loop every Ts:
	measure x (state from sensors/observer)
	get x_ref (reference trajectory)
	
	# LQR feedback
	e = x - x_ref
	u_fb = -K * e
	
	# (optional) feedforward
	u_ff = compute_feedforward(x_ref, x_ref_next)
	
	# total control
	u = u_ff + u_fb
	u = clamp(u, u_min, u_max)
	
	apply u
```

### 5.1 如何离散化连续系统？

如果你的模型是连续时间 $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$，需要先离散化：

$$
A_d = e^{AT_s} \approx I + AT_s + \frac{(AT_s)^2}{2} + \cdots
$$

$$
B_d = \int_0^{T_s}e^{A\tau}d\tau\,B \approx T_s B
$$

简单情况（小 $T_s$）：

$$
A_d \approx I + AT_s,\quad B_d \approx T_s B
$$

MATLAB/Python 有 `c2d` 函数自动做这件事：

```matlab
sys_c = ss(A, B, C, D);
sys_d = c2d(sys_c, Ts);
```

或者 Python：

```python
import control
sys_d = control.c2d(control.ss(A, B, C, D), Ts)
Ad, Bd = sys_d.A, sys_d.B
```

---

## 6. 调参指南：如何选择 $Q$ 和 $R$

### 6.1 基本原则

- **$Q$ 对角线元素**：对应各状态的权重
	- 值越大：越希望该状态快速收敛、误差小
	- 例如：位置误差比速度误差重要 → $Q_{11} > Q_{22}$
- **$R$ 对角线元素**：对应各控制输入的代价
	- 值越大：越不想用大控制量（省能量、避免饱和）
	- 例如：电机功率有限、想避免过热 → 增大 $R$

### 6.2 调参流程（从零开始）

1. **归一化状态和输入**：
	- 把状态 $\mathbf{x}$ 和控制 $\mathbf{u}$ 的量纲/幅值归一化到相近量级
	- 例如：角度 rad、速度 m/s、力 N 可能差几个数量级，先做无量纲化

2. **初始权重**：
	- 所有状态权重相同：$Q = \text{diag}(1,1,\ldots)$
	- 控制权重：$R = \text{diag}(1,1,\ldots)$

3. **增大关键状态权重**：
	- 观察哪些状态收敛慢/误差大，增大对应 $Q_{ii}$（例如乘以 10~100）
	- 重新算 $K$，仿真/测试

4. **调节 $R$ 控制饱和**：
	- 如果控制量经常饱和/抖动：增大 $R$
	- 如果响应太慢：减小 $R$

5. **迭代优化**：
	- 用仿真/实验反复调整 $Q,R$，观察闭环性能
	- 也可以用 Bryson's rule（见下文）

### 6.3 Bryson's Rule（快速起点）

一个经验公式：

$$
Q_{ii} = \frac{1}{(\text{max acceptable deviation of } x_i)^2}
$$

$$
R_{jj} = \frac{1}{(\text{max acceptable magnitude of } u_j)^2}
$$

例如：

- 位置误差允许 $\pm 0.1$ m → $Q_{11} = 1/0.1^2 = 100$
- 速度误差允许 $\pm 0.5$ m/s → $Q_{22} = 1/0.5^2 = 4$
- 控制力允许 $\pm 10$ N → $R = 1/10^2 = 0.01$

这给你一个合理起点，之后再微调。

---

## 7. LQR 的优势与局限

### 7.1 优势

1. **系统化设计**：不用试 $K_p,K_i,K_d$，而是试 $Q,R$（更有物理意义）
2. **自动稳定**：如果系统可控，LQR 闭环一定稳定（保证鲁棒稳定裕度）
3. **多输入多输出（MIMO）友好**：一次设计多个耦合控制通道
4. **易于集成观测器**：可以与卡尔曼滤波器（Kalman Filter）组合成 LQG（线性二次高斯）

### 7.2 局限

1. **需要线性模型**：
	- 实际系统往往非线性（例如摩擦、饱和、空气阻力）
	- 需要在工作点附近线性化（小扰动假设）
	
2. **模型依赖**：
	- $A,B$ 不准会影响性能
	- 解决办法：增益调度、自适应、鲁棒控制（$H_\infty$）

3. **无约束处理**：
	- LQR 不直接处理输入/状态约束（饱和、死区）
	- 需要事后 clamp 或用 MPC 替代

4. **离线计算 $K$**：
	- 如果系统参数变化，需要重新算 $K$
	- 解决办法：增益调度（多个工作点预算 $K$）

---

## 8. 扩展：时变 LQR（Time-Varying LQR）

对于轨迹跟踪，系统可能在不同时刻线性化得到不同的 $A_k,B_k$。

此时可以用**有限时域 LQR**：

$$
J = \mathbf{x}_N^\top Q_f\mathbf{x}_N + \sum_{k=0}^{N-1}\left(\mathbf{x}_k^\top Q\mathbf{x}_k + \mathbf{u}_k^\top R\mathbf{u}_k\right)
$$

反向递推 Riccati 方程：

$$
P_k = Q + A_k^\top P_{k+1}A_k - A_k^\top P_{k+1}B_k(R + B_k^\top P_{k+1}B_k)^{-1}B_k^\top P_{k+1}A_k
$$

从 $P_N = Q_f$ 开始，逐步计算 $P_{N-1},\ldots,P_0$，得到时变增益：

$$
K_k = (R + B_k^\top P_{k+1}B_k)^{-1}B_k^\top P_{k+1}A_k
$$

这在轨迹优化中非常有用（例如 iLQR、DDP）。

---

## 9. LQR 与状态观测器：LQG（Linear Quadratic Gaussian）

实际中你往往不能直接测到所有状态（例如速度需要估计）。

解决办法：用**卡尔曼滤波器**（Kalman Filter）估计状态 $\hat{\mathbf{x}}$，然后用 LQR 控制：

$$
\mathbf{u}_k = -K\hat{\mathbf{x}}_k
$$

这叫做**分离定理**（Separation Principle）：

- 设计 LQR 增益 $K$（假设状态已知）
- 设计卡尔曼滤波器（假设控制已知）
- 两者独立设计，组合后仍然最优（在线性高斯假设下）

工程上非常实用：传感器噪声大、部分状态不可测，用卡尔曼滤波 + LQR 是标准组合。

---

## 10. 实际案例：机器人底盘速度控制（麦轮/差速）

### 10.1 模型

假设底盘动力学简化为：

$$
\dot{v}_x = -\frac{c}{m}v_x + \frac{1}{m}F_x
$$

离散化（$T_s=0.01$ s）：

$$
v_{x,k+1} = (1 - \frac{cT_s}{m})v_{x,k} + \frac{T_s}{m}F_{x,k}
$$

状态 $\mathbf{x} = [v_x]$，控制 $\mathbf{u} = [F_x]$。

### 10.2 LQR 设计

$$
A_d = [1 - cT_s/m],\quad B_d = [T_s/m]
$$

选择：

$$
Q = [10],\quad R = [0.1]
$$

（希望速度误差小，控制力代价适中）

计算 $K$：

```python
import numpy as np
import control

Ad = np.array([[1 - c*Ts/m]])
Bd = np.array([[Ts/m]])
Q = np.array([[10]])
R = np.array([[0.1]])

K, S, E = control.dlqr(Ad, Bd, Q, R)
```

得到 $K \approx [k]$，控制律：

$$
F_x = -k(v_x - v_{x,ref})
$$

### 10.3 对比 PID

传统 PID 需要试 $K_p,K_i,K_d$；LQR 直接给你一个"根据动力学优化"的比例增益 $k$。

如果加积分作用（消除稳态误差），可以扩展状态增加积分项（见下节）。

---

## 11. 常见扩展技巧

### 11.1 增加积分作用（LQI：LQR with Integral Action）

LQR 本身没有积分项，稳态误差可能不为零（尤其是有常值扰动）。

解决办法：扩展状态，增加误差积分 $z_k = z_{k-1} + e_k$：

$$
\mathbf{x}_{aug} = \begin{bmatrix}\mathbf{x}\\ z\end{bmatrix}
$$

新系统：

$$
\begin{bmatrix}\mathbf{x}_{k+1}\\ z_{k+1}\end{bmatrix} = \begin{bmatrix}A_d & 0\\ C_d & I\end{bmatrix}\begin{bmatrix}\mathbf{x}_k\\ z_k\end{bmatrix} + \begin{bmatrix}B_d\\ 0\end{bmatrix}\mathbf{u}_k
$$

对扩展系统做 LQR，得到 $K = [K_x, K_z]$：

$$
\mathbf{u}_k = -K_x\mathbf{x}_k - K_z z_k
$$

其中 $-K_z z_k$ 就是积分项！

### 11.2 增益调度（Gain Scheduling）

对于大范围运动（例如不同速度、不同姿态），可以在多个工作点线性化，计算多个 $K_i$，然后根据当前状态插值：

$$
K(\mathbf{x}) = \sum_i w_i(\mathbf{x})K_i
$$

### 11.3 轨迹优化（iLQR/DDP）

iLQR（迭代 LQR）和 DDP（微分动态规划）是**非线性轨迹优化**方法，核心思路：

- 在初始轨迹附近做线性化
- 用时变 LQR 计算修正
- 迭代更新轨迹

这在机器人运动规划、无人机机动、机械臂操作中非常流行。

---

## 12. 调试与常见问题

### 12.1 闭环不稳定

可能原因：

- 模型 $A,B$ 错误（符号/量纲/数值）
- 离散化不当（$T_s$ 太大）
- 系统不可控（检查 controllability matrix）

排查：

- 检查开环系统的极点（`eig(Ad)`）
- 检查闭环系统的极点（`eig(Ad - Bd*K)`），应该都在单位圆内

### 12.2 响应太慢

- 增大 $Q$（特别是慢变状态对应的权重）
- 减小 $R$

### 12.3 控制量抖动/饱和

- 减小 $Q$
- 增大 $R$
- 检查传感器噪声，加滤波/状态估计

### 12.4 稳态误差

- LQR 没有积分作用 → 用 LQI
- 或在前馈项里补偿已知扰动

---

## 13. LQR vs PID vs MPC：什么时候选谁？

| 方法     | 优点                               | 缺点                       | 适用场景                   |
| ------ | -------------------------------- | ------------------------ | ---------------------- |
| **PID**  | 简单、直观、单变量好用                      | 多变量耦合难调、无模型信息            | 单输入单输出、模型不清楚           |
| **LQR**  | 系统化、MIMO 友好、自动稳定、调参有物理意义         | 需要模型、无约束处理、离线计算          | 线性系统、多输入多输出、模型已知       |
| **MPC**  | 显式约束处理、预测未来、最通用                  | 计算量大、需要求解器、调参复杂          | 需要约束、长时域优化、高性能处理器      |

一般建议：

- **先用 PID 跑通**：验证执行器、传感器、基本稳定性
- **再上 LQR**：如果有模型、需要多变量协调、希望系统化调参
- **最后考虑 MPC**：如果需要约束、预测、复杂代价函数

---

## 14. 实现示例：Python 完整代码（倒立摆）

```python
import numpy as np
import control
import matplotlib.pyplot as plt

# 系统参数
m = 1.0  # 质量
l = 1.0  # 摆长
g = 9.81

# 连续时间模型
A = np.array([[0, 1],
              [g/l, 0]])
B = np.array([[0],
              [1/(m*l**2)]])

# 离散化
Ts = 0.01
sys_c = control.ss(A, B, np.eye(2), 0)
sys_d = control.c2d(sys_c, Ts)
Ad, Bd = sys_d.A, sys_d.B

# LQR 设计
Q = np.diag([100, 1])   # 重视角度误差
R = np.array([[0.1]])    # 控制代价适中

K, S, E = control.dlqr(Ad, Bd, Q, R)
print(f"LQR Gain K = {K}")
print(f"Closed-loop poles: {E}")

# 仿真
N = 500
x = np.array([0.1, 0])  # 初始扰动：0.1 rad
x_hist = [x.copy()]
u_hist = []

for k in range(N):
    u = -K @ x
    u = np.clip(u, -10, 10)  # 饱和
    u_hist.append(u[0])
    
    x = Ad @ x + Bd @ u.reshape(-1)
    x_hist.append(x.copy())

x_hist = np.array(x_hist)
time = np.arange(N+1) * Ts

# 绘图
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(time, x_hist[:, 0], label='θ (rad)')
plt.plot(time, x_hist[:, 1], label='dθ/dt (rad/s)')
plt.legend()
plt.grid()
plt.title('LQR Inverted Pendulum Control')

plt.subplot(2, 1, 2)
plt.plot(time[:-1], u_hist, label='u (control)')
plt.legend()
plt.grid()
plt.xlabel('Time (s)')

plt.tight_layout()
plt.show()
```

---

## 15. 总结与下一步

### 核心要点

1. **LQR 本质**：通过最小化二次代价函数，自动平衡"状态误差"和"控制代价"
2. **设计流程**：建模 → 离散化 → 选 $Q,R$ → 求解 $K$ → 实现反馈 $u=-K\mathbf{x}$
3. **调参核心**：$Q$ 控制响应速度，$R$ 控制代价/饱和
4. **扩展能力强**：可加积分、可时变、可与卡尔曼滤波结合、可扩展到非线性（iLQR）

### 适合你的场景

如果你在做：

- **机器人底盘控制**：速度 $v_x,v_y,\omega$ 跟踪（麦轮/全向）
- **云台姿态控制**：pitch/yaw 角度稳定
- **机械臂轨迹跟踪**：关节角度/速度控制

LQR 都是非常好的选择！尤其是：

- 你已经有一个粗略的动力学模型（质量、转动惯量、摩擦系数）
- 控制频率足够高（100 Hz 以上）
- 传感器噪声不太大（或已经做了滤波/观测器）

### 进阶方向

- **LQG**：LQR + 卡尔曼滤波（处理传感器噪声/部分可观）
- **$H_\infty$ 控制**：更鲁棒的 LQR（处理模型不确定性）
- **MPC**：加约束的 LQR（处理输入/状态限制）
- **iLQR/DDP**：非线性轨迹优化（机器人运动规划）

---

## 16. 参考资料

- **经典教材**：
	- *Optimal Control Theory* by Kirk
	- *Linear Optimal Control* by Anderson & Moore
	- *Feedback Systems* by Åström & Murray（免费在线）
- **工具文档**：
	- MATLAB Control System Toolbox: `lqr`, `dlqr`
	- Python Control Systems Library: `control.lqr`, `control.dlqr`
- **机器人应用**：
	- Drake（MIT）：机器人轨迹优化、LQR 树
	- Underactuated Robotics（Russ Tedrake）：倒立摆、四旋翼 LQR 案例

---

**如果你想要针对具体对象（底盘/云台/机械臂）写一套调参起点和 ROS 控制循环代码，随时告诉我！**
