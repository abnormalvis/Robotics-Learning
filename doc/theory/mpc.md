# MPC 模型预测控制

MPC（Model Predictive Control，模型预测控制）是一类**基于模型在线优化未来控制序列**的先进控制方法。它在工程上的核心优势是：

- **显式处理约束**：输入饱和、状态限制、多变量耦合约束
- **预测未来**：利用模型预测未来轨迹，提前规避问题
- **滚动优化**：每个时刻重新规划，自动适应扰动和模型误差
- **统一框架**：从 SISO 到 MIMO、从线性到非线性、从跟踪到经济优化都能涵盖

MPC 在这些领域广泛应用：

- 化工过程控制（炼油、反应器、蒸馏塔）
- 自动驾驶（轨迹规划与跟踪）
- 机器人运动控制（移动底盘、机械臂）
- 电力系统（微电网、可再生能源调度）
- 航空航天（无人机、火箭着陆）

核心思想可以用一句话概括：

> **在每个时刻，根据当前状态和未来参考，求解一个有限时域的最优控制问题，执行第一步，然后在下一时刻重新求解。**

下面按"核心思想 → 数学建模 → 求解方法 → 工程实现 → 调参经验 → 与 LQR/PID 对比"展开。

---

## 1. 核心思想：预测 + 优化 + 滚动

### 1.1 预测（Prediction）

给定当前状态 $\mathbf{x}_0$ 和未来控制序列 $\mathbf{u}_0,\mathbf{u}_1,\ldots,\mathbf{u}_{N-1}$，用**模型**预测未来状态：

$$
\begin{aligned}
\mathbf{x}_1 &= f(\mathbf{x}_0, \mathbf{u}_0) \\
\mathbf{x}_2 &= f(\mathbf{x}_1, \mathbf{u}_1) \\
&\vdots \\
\mathbf{x}_N &= f(\mathbf{x}_{N-1}, \mathbf{u}_{N-1})
\end{aligned}
$$

- $f(\cdot)$：系统动力学（可以是线性或非线性）
- $N$：预测时域（Prediction Horizon）

这一步的关键是**模型质量**：模型越准，预测越准，MPC 效果越好。

### 1.2 优化（Optimization）

在预测的基础上，求解一个优化问题，找到"最好的控制序列"：

$$
\min_{\mathbf{u}_0,\ldots,\mathbf{u}_{N-1}} J = \sum_{k=0}^{N-1}\left(l(\mathbf{x}_k,\mathbf{u}_k) + \Phi(\mathbf{x}_N)\right)
$$

- $l(\mathbf{x}_k,\mathbf{u}_k)$：每一步的代价（例如 $\|\mathbf{x}_k - \mathbf{x}_{ref}\|_Q^2 + \|\mathbf{u}_k\|_R^2$）
- $\Phi(\mathbf{x}_N)$：终端代价（Terminal Cost，让终端状态接近目标）

同时满足约束：

$$
\begin{aligned}
\mathbf{u}_{min} &\leq \mathbf{u}_k \leq \mathbf{u}_{max} \quad (\text{输入约束}) \\
\mathbf{x}_{min} &\leq \mathbf{x}_k \leq \mathbf{x}_{max} \quad (\text{状态约束}) \\
\mathbf{x}_k &= f(\mathbf{x}_{k-1}, \mathbf{u}_{k-1}) \quad (\text{动力学约束})
\end{aligned}
$$

### 1.3 滚动（Receding Horizon）

MPC 的"魔法"在于：

1. 在时刻 $t$ 求解上述优化问题，得到最优控制序列 $\mathbf{u}_0^*,\mathbf{u}_1^*,\ldots,\mathbf{u}_{N-1}^*$
2. **只执行第一步** $\mathbf{u}_0^*$
3. 在下一时刻 $t+1$，重新测量状态，重新求解（从 $t+1$ 开始的新优化问题）

这叫做**滚动时域**（Receding Horizon）或**移动窗口**（Moving Horizon）。

直观理解：

- 就像开车：你看前面 50 米（预测），决定怎么打方向盘（优化），但只打当前这一下（执行第一步），然后再看前面 50 米重新决策（滚动）。
- 即使模型不完美、有扰动，每次重新优化都能"纠正"之前的预测误差。

---

## 2. MPC 的数学表述（线性 MPC 为例）

### 2.1 线性离散时间系统

考虑线性时不变（LTI）系统：

$$
\mathbf{x}_{k+1} = A\mathbf{x}_k + B\mathbf{u}_k
$$

- $\mathbf{x}_k \in \mathbb{R}^{n_x}$：状态向量
- $\mathbf{u}_k \in \mathbb{R}^{n_u}$：控制输入

### 2.2 MPC 优化问题（标准二次型形式）

在时刻 $k$，已知当前状态 $\mathbf{x}_k$，求解：

$$
\min_{\mathbf{u}_k,\ldots,\mathbf{u}_{k+N-1}} J = \sum_{i=0}^{N-1}\left(\|\mathbf{x}_{k+i} - \mathbf{x}_{ref}\|_Q^2 + \|\mathbf{u}_{k+i}\|_R^2\right) + \|\mathbf{x}_{k+N} - \mathbf{x}_{ref}\|_P^2
$$

约束：

$$
\begin{aligned}
\mathbf{x}_{k+i+1} &= A\mathbf{x}_{k+i} + B\mathbf{u}_{k+i}, \quad i=0,\ldots,N-1 \\
\mathbf{u}_{min} &\leq \mathbf{u}_{k+i} \leq \mathbf{u}_{max}, \quad i=0,\ldots,N-1 \\
\mathbf{x}_{min} &\leq \mathbf{x}_{k+i} &\leq \mathbf{x}_{max}, \quad i=1,\ldots,N
\end{aligned}
$$

- $Q,R,P$：权重矩阵（类似 LQR）
- $N$：预测时域
- $\mathbf{x}_{ref}$：参考状态（可以是轨迹 $\mathbf{x}_{ref,k+i}$）

### 2.3 求解得到最优控制序列

优化求解器给出：

$$
\mathbf{u}_k^*, \mathbf{u}_{k+1}^*, \ldots, \mathbf{u}_{k+N-1}^*
$$

只执行 $\mathbf{u}_k^*$，然后在下一时刻重新求解。

---

## 3. 线性 MPC 的矩阵形式（便于 QP 求解）

### 3.1 预测模型的批量形式

把所有未来状态写成一个向量：

$$
\mathbf{X} = \begin{bmatrix}\mathbf{x}_{k+1}\\ \mathbf{x}_{k+2}\\ \vdots\\ \mathbf{x}_{k+N}\end{bmatrix}, \quad \mathbf{U} = \begin{bmatrix}\mathbf{u}_k\\ \mathbf{u}_{k+1}\\ \vdots\\ \mathbf{u}_{k+N-1}\end{bmatrix}
$$

动力学约束可以展开：

$$
\mathbf{X} = \mathcal{A}\mathbf{x}_k + \mathcal{B}\mathbf{U}
$$

其中：

$$
\mathcal{A} = \begin{bmatrix}A\\ A^2\\ \vdots\\ A^N\end{bmatrix}, \quad \mathcal{B} = \begin{bmatrix}B & 0 & \cdots & 0\\ AB & B & \cdots & 0\\ \vdots & \vdots & \ddots & \vdots\\ A^{N-1}B & A^{N-2}B & \cdots & B\end{bmatrix}
$$

### 3.2 二次型代价函数

代价函数可以写成：

$$
J = \mathbf{X}^\top \bar{Q}\mathbf{X} + \mathbf{U}^\top \bar{R}\mathbf{U} + \text{线性项}
$$

其中 $\bar{Q},\bar{R}$ 是分块对角矩阵。

把 $\mathbf{X} = \mathcal{A}\mathbf{x}_k + \mathcal{B}\mathbf{U}$ 代入，得到关于 $\mathbf{U}$ 的二次型：

$$
J = \frac{1}{2}\mathbf{U}^\top H\mathbf{U} + \mathbf{f}^\top\mathbf{U} + \text{常数}
$$

### 3.3 QP 问题（Quadratic Programming）

MPC 变成了一个**凸二次规划**（Convex QP）：

$$
\min_{\mathbf{U}} \frac{1}{2}\mathbf{U}^\top H\mathbf{U} + \mathbf{f}^\top\mathbf{U}
$$

约束：

$$
\begin{aligned}
\mathbf{U}_{min} &\leq \mathbf{U} \leq \mathbf{U}_{max} \\
\mathcal{C}\mathbf{U} &\leq \mathbf{b} \quad (\text{线性不等式约束，包含状态约束})
\end{aligned}
$$

### 3.4 求解工具

成熟的 QP 求解器：

- **MATLAB**：`quadprog`
- **Python**：`cvxpy`, `qpsolvers`, `osqp`（开源、快速）
- **C++/嵌入式**：`qpOASES`, `HPIPM`, `TinyMPC`（实时控制）

求解后得到 $\mathbf{U}^* = [\mathbf{u}_k^*, \mathbf{u}_{k+1}^*, \ldots]$，取第一个 $\mathbf{u}_k^*$ 执行。

---

## 4. 非线性 MPC（NMPC）

### 4.1 非线性系统

对于非线性动力学：

$$
\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k)
$$

例如：

- 移动机器人：$\dot{x} = v\cos\theta$，$\dot{y} = v\sin\theta$，$\dot{\theta} = \omega$
- 无人机姿态：四元数/欧拉角动力学
- 机械臂：非线性耦合动力学

### 4.2 NMPC 优化问题

$$
\min_{\mathbf{u}_k,\ldots,\mathbf{u}_{k+N-1}} J = \sum_{i=0}^{N-1}l(\mathbf{x}_{k+i},\mathbf{u}_{k+i}) + \Phi(\mathbf{x}_{k+N})
$$

约束：

$$
\begin{aligned}
\mathbf{x}_{k+i+1} &= f(\mathbf{x}_{k+i}, \mathbf{u}_{k+i}) \\
\mathbf{u}_{min} &\leq \mathbf{u}_{k+i} \leq \mathbf{u}_{max} \\
g(\mathbf{x}_{k+i}, \mathbf{u}_{k+i}) &\leq 0 \quad (\text{非线性约束})
\end{aligned}
$$

这是一个**非线性规划**（NLP）问题，求解难度大得多。

### 4.3 求解方法

- **直接法（Direct Methods）**：
	- 把连续时间离散化成有限变量，用 NLP 求解器（如 IPOPT, SNOPT）
	- 工具：CasADi（Python/C++）、ACADO、do-mpc

- **间接法（Indirect Methods）**：
	- 基于最优性条件（Pontryagin 极小值原理），求解边界值问题
	- 较少用于实时 MPC

- **序列二次规划（SQP）**：
	- 每次迭代解一个 QP 子问题，逐步逼近 NLP 解
	- 实时性好，适合机器人应用

---

## 5. MPC 的关键参数与调参

### 5.1 预测时域 $N$（Prediction Horizon）

**含义**：MPC 预测未来 $N$ 步的状态。

**影响**：

- $N$ 大：
	- 优点：能"看得更远"，提前规避约束、优化长期目标
	- 缺点：计算量大（QP/NLP 变量增多），可能超过实时要求
- $N$ 小：
	- 优点：计算快
	- 缺点："目光短浅"，可能过于贪心、忽略长期约束

**经验值**：

- 快速系统（高频控制）：$N=5\sim 20$
- 慢速系统（化工过程）：$N=20\sim 100$
- 根据采样周期 $T_s$ 和期望的"预测时长"确定：$N \cdot T_s \approx 1\sim 5$ 秒

### 5.2 控制时域 $N_c$（Control Horizon）

有时为了减少计算量，只优化前 $N_c$ 步的控制，后面的控制保持不变或用简单策略。

**常见做法**：$N_c \leq N$，例如 $N_c = N/2$。

### 5.3 权重矩阵 $Q,R,P$

与 LQR 类似：

- $Q$：状态误差权重（对角线元素对应各状态的重要性）
- $R$：控制代价权重（抑制控制量大小/变化率）
- $P$：终端权重（让最后一步状态接近目标，通常取 LQR 的 Riccati 解）

**调参建议**：

1. 从 LQR 权重开始（如果系统接近线性）
2. 增大 $Q$ → 更快跟踪，但可能更激进
3. 增大 $R$ → 更平滑控制，但可能慢
4. 增大 $P$ → 强制终端收敛，避免"短视"

### 5.4 约束设置

MPC 的核心优势就是**显式处理约束**！

**常见约束**：

- **输入约束**：$u_{min} \leq u \leq u_{max}$（执行器饱和）
- **输入变化率约束**：$\Delta u_{min} \leq u_k - u_{k-1} \leq \Delta u_{max}$（避免抖动）
- **状态约束**：$x_{min} \leq x \leq x_{max}$（例如速度限制、安全区域）
- **障碍物/碰撞约束**：非线性不等式 $g(\mathbf{x}) \leq 0$

**调参建议**：

- 先不加约束，验证基本性能
- 逐步加约束，观察是否频繁触及约束边界
- 如果约束总是饱和，说明参考轨迹/权重设置不合理

---

## 6. MPC 的稳定性：为什么需要终端约束/代价？

### 6.1 有限时域的问题

如果 $N$ 很小，MPC 可能"目光短浅"：只关心眼前几步，忽略长期稳定性。

例如：倒立摆 MPC，如果 $N=3$，可能会选择"先往前冲、最后再拉回来"的策略，但实际上来不及拉回来就倒了。

### 6.2 解决办法：终端约束/代价

**方法 1：终端代价**（最常用）

选择 $P$ 为 LQR 的 Riccati 解（对应无限时域的最优代价）：

$$
\Phi(\mathbf{x}_N) = \mathbf{x}_N^\top P\mathbf{x}_N
$$

这样终端状态的"未来代价"被合理估计，MPC 会自动考虑长期稳定性。

**方法 2：终端约束**

强制 $\mathbf{x}_N$ 进入一个"稳定不变集"（例如原点附近的小球）：

$$
\|\mathbf{x}_N\|_P \leq \epsilon
$$

这保证从 $\mathbf{x}_N$ 出发，系统能稳定收敛。

**工程实践**：

- 终端代价更灵活、计算友好
- 终端约束理论上更严格，但可能导致优化问题无解（如果时域太短）

---

## 7. 工程实现：MPC 控制循环

### 7.1 基本流程

```text
given: model (A, B or f), weights (Q, R, P), horizon N, constraints

initialize: x0 (current state), u_prev

loop every Ts:
	# 1. 测量/估计状态
	x_current = measure_or_estimate_state()
	
	# 2. 获取参考轨迹
	x_ref = get_reference_trajectory(N)  # 未来 N 步
	
	# 3. 构建 MPC 优化问题
	problem = build_MPC_problem(x_current, x_ref, N, Q, R, P, constraints)
	
	# 4. 求解 QP/NLP
	U_optimal = solve(problem)  # [u0*, u1*, ..., u_{N-1}*]
	
	# 5. 执行第一步控制
	u_current = U_optimal[0]
	u_current = clamp(u_current, u_min, u_max)
	apply(u_current)
	
	# 6. 记录（用于下一时刻）
	u_prev = u_current
```

### 7.2 Python 实现示例（线性 MPC + OSQP）

```python
import numpy as np
import osqp
from scipy import sparse

# 系统参数
A = np.array([[1, 0.1], [0, 1]])  # 离散化后的 A
B = np.array([[0], [0.1]])        # 离散化后的 B
nx, nu = 2, 1

# MPC 参数
N = 10  # 预测时域
Q = sparse.diags([10, 1])   # 状态权重
R = 0.1 * sparse.eye(nu)    # 控制权重
P = Q                        # 终端权重

# 约束
u_min, u_max = -1.0, 1.0

# 构建预测矩阵
def build_MPC_matrices(A, B, N):
    nx, nu = B.shape
    # 预测矩阵 [x1; x2; ...; xN] = Ax * x0 + Bx * [u0; u1; ...; u_{N-1}]
    Ax = np.vstack([np.linalg.matrix_power(A, i) for i in range(1, N+1)])
    
    Bx = np.zeros((N*nx, N*nu))
    for i in range(N):
        for j in range(i+1):
            Bx[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = np.linalg.matrix_power(A, i-j) @ B
    
    return Ax, Bx

Ax, Bx = build_MPC_matrices(A, B, N)

# 代价函数矩阵
Qbar = sparse.block_diag([Q]*(N-1) + [P])
Rbar = sparse.block_diag([R]*N)
H = Bx.T @ Qbar @ Bx + Rbar
H = sparse.csc_matrix(H)

# 约束矩阵
A_ineq = sparse.vstack([
    sparse.eye(N*nu),     # u <= u_max
    -sparse.eye(N*nu)     # -u <= -u_min
])
l_ineq = np.hstack([np.full(N*nu, u_min), np.full(N*nu, -u_max)])
u_ineq = np.hstack([np.full(N*nu, u_max), np.full(N*nu, -u_min)])

# 初始化求解器
prob = osqp.OSQP()
prob.setup(H, np.zeros(N*nu), A_ineq, l_ineq, u_ineq, verbose=False)

# 仿真
x = np.array([1.0, 0.0])  # 初始状态
x_ref = np.zeros(nx)       # 参考状态（原点）
sim_steps = 50
x_hist = [x.copy()]
u_hist = []

for t in range(sim_steps):
    # 更新代价函数的线性项
    q = Bx.T @ Qbar @ (Ax @ x - np.tile(x_ref, N))
    prob.update(q=q)
    
    # 求解
    res = prob.solve()
    
    if res.info.status != 'solved':
        print(f"Warning: MPC not solved at step {t}")
        u = 0
    else:
        u = res.x[0]  # 取第一步控制
    
    u = np.clip(u, u_min, u_max)
    u_hist.append(u)
    
    # 应用控制，更新状态
    x = A @ x + B @ np.array([u])
    x_hist.append(x.copy())

x_hist = np.array(x_hist)

# 绘图
import matplotlib.pyplot as plt
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(x_hist[:, 0], label='x1 (position)')
plt.plot(x_hist[:, 1], label='x2 (velocity)')
plt.legend()
plt.grid()
plt.title('MPC Control')

plt.subplot(2, 1, 2)
plt.plot(u_hist, label='u (control)')
plt.axhline(u_min, color='r', linestyle='--', label='u_min/max')
plt.axhline(u_max, color='r', linestyle='--')
plt.legend()
plt.grid()
plt.xlabel('Time step')
plt.tight_layout()
plt.show()
```

---

## 8. MPC 的优势与局限

### 8.1 优势

1. **显式约束处理**：
	- 执行器饱和、状态限制、障碍物避障都可以直接写进约束
	- 不像 PID/LQR 需要事后 clamp

2. **预测能力**：
	- 利用模型"看未来"，提前调整控制
	- 适合有延迟、慢动态、需要提前规避约束的系统

3. **多变量协调**：
	- MIMO 系统天然支持
	- 可以优化多个目标（速度 vs 能耗、精度 vs 平滑度）

4. **鲁棒性**（滚动优化）：
	- 每次重新测量、重新优化，自动补偿模型误差和扰动

5. **统一框架**：
	- 从跟踪、稳定、经济优化、路径规划都可以用 MPC 建模

### 8.2 局限

1. **计算量大**：
	- 每个周期求解 QP/NLP，实时性要求高
	- 非线性 MPC 尤其困难（需要快速 NLP 求解器 + 强大硬件）

2. **模型依赖**：
	- 模型不准，预测就不准，性能下降
	- 需要良好的系统辨识/建模

3. **调参复杂**：
	- $Q,R,P,N,N_c$ + 约束 → 多维参数空间
	- 需要仿真/实验反复调试

4. **可能无解**：
	- 约束过严、时域太短、初始状态太差 → 优化问题无可行解
	- 需要设计合理的 fallback 策略（例如放宽约束、用备用控制器）

---

## 9. MPC vs LQR vs PID：什么时候选 MPC？

| 特性       | PID        | LQR          | MPC            |
| -------- | ---------- | ------------ | -------------- |
| **模型需求** | 无          | 线性模型         | 线性/非线性模型       |
| **约束处理** | 无（事后clamp） | 无（事后clamp）   | **显式约束（核心优势）** |
| **多变量**  | 难（需多个回路）   | 容易（MIMO友好）   | 容易（MIMO友好）     |
| **计算量**  | 极小         | 小（离线算K）      | **大（在线优化）**    |
| **实时性**  | 高          | 高            | 中~低（取决于求解器）    |
| **调参难度** | 中          | 中            | 高              |
| **适用场景** | 单变量、简单系统   | 线性、无约束、快速响应  | 约束多、预测重要、慢动态   |

**何时选 MPC？**

- 系统有**硬约束**（例如输入饱和必须严格满足、状态不能越界）
- 需要**预测未来**（例如路径规划、避障、提前减速）
- 系统**慢动态**（化工过程、温度控制），可以承受在线优化的计算
- **MIMO 耦合**强（例如多关节机械臂、多输入车辆）
- 有**高性能计算平台**（嵌入式 CPU/GPU、实时 Linux）

**何时不选 MPC？**

- 系统**超快**（例如高频电机电流环），在线优化来不及
- **模型很不准**（此时预测不可靠，不如用鲁棒反馈如 ADRC）
- **计算资源极受限**（单片机、无浮点运算）
- **约束不严格**（偶尔饱和可以接受），此时 LQR + clamp 足够

---

## 10. 实际案例：移动机器人轨迹跟踪

### 10.1 问题描述

差速轮机器人，状态 $\mathbf{x} = [x, y, \theta]^\top$，控制 $\mathbf{u} = [v, \omega]^\top$（线速度、角速度）。

动力学（连续时间）：

$$
\begin{aligned}
\dot{x} &= v\cos\theta \\
\dot{y} &= v\sin\theta \\
\dot{\theta} &= \omega
\end{aligned}
$$

约束：

- $|v| \leq v_{max}$（最大速度）
- $|\omega| \leq \omega_{max}$（最大角速度）
- 避障：$\|(x,y) - (x_{obs}, y_{obs})\| \geq r_{safe}$

目标：跟踪参考轨迹 $(x_{ref}(t), y_{ref}(t), \theta_{ref}(t))$。

### 10.2 MPC 建模

1. **离散化**（欧拉法，$T_s=0.1$ s）：
$$
\begin{aligned}
x_{k+1} &= x_k + T_s v_k\cos\theta_k \\
y_{k+1} &= y_k + T_s v_k\sin\theta_k \\
\theta_{k+1} &= \theta_k + T_s \omega_k
\end{aligned}
$$

2. **线性化**（在参考轨迹附近）：
   - 如果参考轨迹平滑，可以在 $(x_{ref,k}, y_{ref,k}, \theta_{ref,k})$ 处线性化
   - 得到时变线性系统 $\mathbf{x}_{k+1} = A_k\mathbf{x}_k + B_k\mathbf{u}_k$

3. **代价函数**：
$$
J = \sum_{i=0}^{N-1}\left(\|\mathbf{x}_{k+i} - \mathbf{x}_{ref,k+i}\|_Q^2 + \|\mathbf{u}_{k+i}\|_R^2\right) + \|\mathbf{x}_{k+N} - \mathbf{x}_{ref,k+N}\|_P^2
$$

4. **约束**：
$$
\begin{aligned}
-v_{max} &\leq v_{k+i} \leq v_{max} \\
-\omega_{max} &\leq \omega_{k+i} \leq \omega_{max} \\
\|(x_{k+i}, y_{k+i}) - (x_{obs}, y_{obs})\| &\geq r_{safe}
\end{aligned}
$$

### 10.3 实现（Python + CasADi）

```python
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# 参数
Ts = 0.1
N = 10
nx, nu = 3, 2

# 权重
Q = np.diag([10, 10, 1])   # x, y 更重要
R = np.diag([0.1, 0.1])
P = Q

# 约束
v_max = 1.0
omega_max = 1.0

# CasADi 变量
x = ca.SX.sym('x', nx)
u = ca.SX.sym('u', nu)

# 动力学
x_next = ca.vertcat(
    x[0] + Ts * u[0] * ca.cos(x[2]),
    x[1] + Ts * u[0] * ca.sin(x[2]),
    x[2] + Ts * u[1]
)
f = ca.Function('f', [x, u], [x_next])

# 构建 MPC 优化问题
opti = ca.Opti()
X = opti.variable(nx, N+1)  # 状态轨迹
U = opti.variable(nu, N)    # 控制序列
x0_param = opti.parameter(nx)  # 初始状态
x_ref_param = opti.parameter(nx, N+1)  # 参考轨迹

# 代价函数
cost = 0
for k in range(N):
    cost += ca.mtimes([(X[:, k] - x_ref_param[:, k]).T, Q, (X[:, k] - x_ref_param[:, k])])
    cost += ca.mtimes([U[:, k].T, R, U[:, k]])
cost += ca.mtimes([(X[:, N] - x_ref_param[:, N]).T, P, (X[:, N] - x_ref_param[:, N])])

opti.minimize(cost)

# 动力学约束
for k in range(N):
    opti.subject_to(X[:, k+1] == f(X[:, k], U[:, k]))

# 初始约束
opti.subject_to(X[:, 0] == x0_param)

# 输入约束
for k in range(N):
    opti.subject_to(opti.bounded(-v_max, U[0, k], v_max))
    opti.subject_to(opti.bounded(-omega_max, U[1, k], omega_max))

# 求解器
opts = {'ipopt.print_level': 0, 'print_time': 0}
opti.solver('ipopt', opts)

# 仿真
x_current = np.array([0, 0, 0])  # 初始位置
x_ref_traj = np.array([[np.sin(0.1*t), 0.5*t, 0.1*t] for t in range(N+1)]).T  # 示例参考
sim_steps = 50
x_hist = [x_current.copy()]

for t in range(sim_steps):
    # 更新参考轨迹（滚动窗口）
    x_ref_current = np.array([[np.sin(0.1*(t+i)), 0.5*(t+i)*Ts, 0.1*(t+i)] for i in range(N+1)]).T
    
    # 设置参数
    opti.set_value(x0_param, x_current)
    opti.set_value(x_ref_param, x_ref_current)
    
    # 求解
    try:
        sol = opti.solve()
        u_opt = sol.value(U[:, 0])
    except:
        print(f"MPC failed at step {t}, using zero control")
        u_opt = np.zeros(nu)
    
    # 应用控制
    x_current = np.array(f(x_current, u_opt)).flatten()
    x_hist.append(x_current.copy())

x_hist = np.array(x_hist)

# 绘图
plt.figure(figsize=(8, 6))
plt.plot(x_hist[:, 0], x_hist[:, 1], 'b-', label='MPC trajectory')
t_vec = np.arange(len(x_hist)) * Ts
x_ref_plot = [np.sin(0.1*t) for t in t_vec]
y_ref_plot = [0.5*t*Ts for t in range(len(x_hist))]
plt.plot(x_ref_plot, y_ref_plot, 'r--', label='Reference')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.legend()
plt.grid()
plt.axis('equal')
plt.title('MPC Path Tracking')
plt.show()
```

---

## 11. 高级话题

### 11.1 鲁棒 MPC（Robust MPC）

处理模型不确定性和扰动：

- **管控不变集**（Tube MPC）：把扰动限制在一个"管子"里
- **min-max MPC**：优化最坏情况下的性能
- **随机 MPC**：考虑随机扰动的概率分布

### 11.2 经济 MPC（Economic MPC）

代价函数不是跟踪误差，而是直接优化经济指标（能耗、产量、利润）：

$$
J = \sum_{k=0}^{N-1} c(\mathbf{x}_k, \mathbf{u}_k)
$$

常用于化工过程、电力调度。

### 11.3 学习型 MPC（Learning-based MPC）

- 用数据学习模型（神经网络、高斯过程）
- 在线更新模型参数（自适应 MPC）
- 结合强化学习（RL）和 MPC

### 11.4 显式 MPC（Explicit MPC）

离线求解所有可能初始状态的最优控制（分段仿射反馈律）：

$$
\mathbf{u}^*(\mathbf{x}) = K_i\mathbf{x} + \mathbf{c}_i, \quad \text{if } \mathbf{x} \in \mathcal{R}_i
$$

优点：在线只需查表，计算量极小。

缺点：离线计算量大、存储量大，只适合低维系统。

---

## 12. 调试与常见问题

### 12.1 优化问题无解（Infeasible）

可能原因：

- 约束过严（例如时域太短、初始状态太远、障碍物太近）
- 终端约束太紧
- 模型线性化误差大

解决办法：

- 增大 $N$（更长预测时域）
- 放宽约束（例如用软约束：惩罚项代替硬约束）
- 增加 slack 变量（允许违反约束，但有大惩罚）
- fallback 控制器（如果 MPC 失败，切换到 PID/LQR）

### 12.2 计算时间超过采样周期

- 减小 $N$（预测时域）
- 减小 $N_c$（控制时域）
- 用更快的求解器（OSQP、qpOASES、HPIPM）
- 增加计算资源（更快的 CPU、并行化）
- 用显式 MPC 或近似方法

### 12.3 控制抖动

- 增大 $R$（惩罚控制量）
- 增加 $\Delta u$ 约束（限制控制变化率）
- 用低通滤波后处理 $u^*$

### 12.4 跟踪滞后

- 增大 $Q$（更关注跟踪误差）
- 增大 $N$（更早"看到"未来参考）
- 增加前馈项（如果参考轨迹已知）

---

## 13. 软件工具与库

| 语言/平台      | 工具/库                               | 特点                    |
| ---------- | ---------------------------------- | --------------------- |
| **MATLAB** | MPC Toolbox                        | 商业工具，功能全面，文档丰富        |
| **Python** | `cvxpy`, `do-mpc`, `pyomo`         | 开源，灵活建模              |
|            | `osqp`, `qpsolvers`                | 快速 QP 求解器            |
|            | `CasADi`                           | 符号建模，支持 NLP（推荐）      |
| **C++**    | `qpOASES`, `HPIPM`, `ACADO`        | 嵌入式/实时控制             |
|            | `TinyMPC`                          | 超轻量级，适合微控制器           |
| **ROS**    | `mpc_local_planner`（move_base插件）  | 移动机器人路径跟踪             |
|            | `control_toolbox`（自己封装MPC）       | 需要自己集成求解器             |

**推荐新手流程**：

1. 用 Python + `cvxpy` 或 `do-mpc` 仿真验证
2. 用 CasADi + IPOPT 处理非线性问题
3. 用 C++ + `qpOASES` 或 OSQP 部署到实时系统

---

## 14. 总结与建议

### 核心要点

1. **MPC 的本质**：在线优化 + 滚动时域 = 自适应的最优控制
2. **最大优势**：显式处理约束（执行器饱和、状态限制、障碍物）
3. **最大挑战**：计算量大、模型依赖、调参复杂
4. **适用场景**：约束重要、预测有价值、系统慢动态、有计算资源

### 学习路线

1. **基础**：先掌握 LQR（理解二次代价、Riccati 方程、状态反馈）
2. **线性 MPC**：理解 QP 问题、预测时域、约束建模
3. **非线性 MPC**：学习 CasADi/ACADO、NLP 求解
4. **实战**：仿真 → 硬件在环 → 实物部署

### 什么时候从 LQR 升级到 MPC？

- LQR 够用：系统接近线性、无硬约束、响应快
- 需要 MPC：
	- 约束频繁触及（LQR + clamp 性能差）
	- 需要预测未来（路径规划、避障）
	- 系统非线性强（NMPC）
	- 多目标优化（经济性 + 性能）

---

## 15. 参考资料

- **经典教材**：
	- *Model Predictive Control: Theory and Design* by Rawlings & Mayne
	- *Predictive Control for Linear and Hybrid Systems* by Borrelli, Bemporad, Morari
- **在线课程**：
	- Coursera: *Model Predictive Control* (Politecnico di Milano)
	- YouTube: Steve Brunton 的 MPC 系列
- **软件文档**：
	- CasADi: https://web.casadi.org/
	- do-mpc: https://www.do-mpc.com/
	- OSQP: https://osqp.org/

---

**如果你想针对具体应用（底盘跟踪、机械臂、无人机）写一套 MPC 实现代码和调参指南,随时告诉我!**
