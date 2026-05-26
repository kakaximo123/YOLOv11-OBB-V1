import numpy as np
import matplotlib.pyplot as plt

# =============================
# 参数定义
# =============================
Ts = 0.01  # 采样周期
N = 2000  # 仿真步数
time = np.arange(N) * Ts

# 电机真实参数（用于生成“真实系统”）
K_real = 0.8
T_real = 0.2

# 用于辨识的“假设模型”： y(k) = a1*y(k-1) + b1*u(k-1)
a_true = np.exp(-Ts / T_real)
b_true = K_real * (1 - a_true)

# PI控制器参数（初始值）
Kp = 2.0
Ki = 40.0

# 目标转速
omega_ref = np.ones(N) * 1.0  # 设定为1 rad/s阶跃

# 初始化变量
y = np.zeros(N)  # 电机输出（转速）
u = np.zeros(N)  # 控制输入（电压）
e = np.zeros(N)  # 误差
integral = 0.0

# RLS 初始化
theta = np.zeros(2)  # [a1_hat, b1_hat]
P = np.eye(2) * 1000
lam = 0.99  # 遗忘因子

# 存储辨识结果
a_hat_list, b_hat_list = [], []

# =============================
# 仿真主循环
# =============================
for k in range(1, N):
    # ---------- 真值系统 ----------
    # 模拟真实系统（加入少量噪声）
    y[k] = a_true * y[k - 1] + b_true * u[k - 1] + 0.01 * np.random.randn()

    # ---------- PI 控制器 ----------
    e[k] = omega_ref[k] - y[k]
    integral += e[k] * Ts
    u[k] = Kp * e[k] + Ki * integral

    # 限幅
    u[k] = np.clip(u[k], -5, 5)

    # ---------- RLS 在线辨识 ----------
    phi = np.array([y[k - 1], u[k - 1]])
    y_hat = np.dot(phi, theta)
    err = y[k] - y_hat
    K = P.dot(phi) / (lam + phi.T.dot(P).dot(phi))
    theta = theta + K * err
    P = (P - np.outer(K, phi).dot(P)) / lam

    a_hat_list.append(theta[0])
    b_hat_list.append(theta[1])

    # ---------- 自适应调整 PI ----------
    # 根据辨识的b_hat动态调整PI增益（简单映射法）
    b_hat = max(1e-3, theta[1])
    Kp = 1.0 / (10 * b_hat)
    Ki = 5.0 / (10 * b_hat)

# =============================
# 绘图结果
# =============================
plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(time, omega_ref, 'r--', label='Reference')
plt.plot(time, y, 'b', label='Motor speed')
plt.ylabel('Speed (rad/s)')
plt.legend();
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time, u, 'g', label='Control voltage')
plt.ylabel('Voltage (V)')
plt.legend();
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time[1:], a_hat_list, label='a1_hat')
plt.plot(time[1:], b_hat_list, label='b1_hat')
plt.axhline(a_true, color='r', linestyle='--', label='a1_true')
plt.axhline(b_true, color='g', linestyle='--', label='b1_true')
plt.ylabel('Estimated Params')
plt.xlabel('Time (s)')
plt.legend();
plt.grid()

plt.tight_layout()
plt.show()
