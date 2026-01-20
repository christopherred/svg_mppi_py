import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle

# 导入编译好的 C++ 模块 (假设编译在 build 目录)
sys.path.append(os.path.join(os.path.dirname(__file__), '../build'))
import mppi_py

# === 参数配置 ===
MAP_SIZE = 20.0  # 米
RES = 0.1        # 分辨率
W, H = int(MAP_SIZE/RES), int(MAP_SIZE/RES)
ROBOT_L = 0.33   # 轴距

# 1. 构造地图
grid_map = mppi_py.GridMap()
grid_map.setGeometry(MAP_SIZE, MAP_SIZE, RES, 0.0, 0.0)

# 造一个障碍物 (GridMap 数据是 MatrixXf)
# 障碍物是一个 2D 数组，值 0-100
obs_data = np.zeros((W, H), dtype=np.float32)
# 在中间放个方块障碍
obs_data[80:120, 80:120] = 100.0 
grid_map.add("collision_layer", obs_data)

# 2. 构造参考路径场 (Reference SDF)
# 这是一个简化的场，引导车向右上方跑
ref_sdf = mppi_py.GridMap()
ref_sdf.setGeometry(MAP_SIZE, MAP_SIZE, RES, 0.0, 0.0)

# 生成坐标网格
x = np.linspace(-MAP_SIZE/2, MAP_SIZE/2, W)
y = np.linspace(-MAP_SIZE/2, MAP_SIZE/2, H)
X, Y = np.meshgrid(x, y) # 注意 meshgrid 生成的是 (H, W)

# 简单的目标路径：Y = 0.5 * X (斜线)
# 距离场：点到直线的距离
# 直线方程: 0.5x - y = 0 -> A=0.5, B=-1, C=0
dist_field = np.abs(0.5*X - Y) / np.sqrt(0.5**2 + 1)
# 角度场：直线的角度 atan2(0.5, 1)
target_angle = np.arctan2(0.5, 1.0)
angle_field = np.full_like(X, target_angle)
# 速度场
speed_field = np.full_like(X, 3.0) 

# 注入数据 (注意转置，因为 Eigen 是列优先，numpy 是行优先)
ref_sdf.add("distance_field", dist_field.T.astype(np.float32))
ref_sdf.add("angle_field", angle_field.T.astype(np.float32))
ref_sdf.add("speed_field", speed_field.T.astype(np.float32))

# 3. 初始化控制器
common_p = mppi_py.Params.Common()
common_p.thread_num = 8
common_p.prediction_step_size = 30
common_p.prediction_interval = 0.1
common_p.max_steer_angle = 0.5
common_p.reference_speed = 3.0
common_p.speed_prediction_mode = "REFERENCE"
# 必须设置这个，不然 C++ 会报除以零错误
common_p.steer_1st_delay = 0.1 

svg_p = mppi_py.Params.SVGuidedMPPI()
svg_p.sample_batch_num = 100
svg_p.lambda_ = 10.0
svg_p.alpha = 0.1
svg_p.steer_cov = 0.5
svg_p.guide_sample_num = 1
svg_p.num_svgd_iteration = 5
svg_p.svgd_step_size = 0.01
svg_p.is_use_nominal_solution = True
svg_p.is_covariance_adaptation = True
svg_p.gaussian_fitting_lambda = 0.1
svg_p.min_steer_cov = 0.01
svg_p.max_steer_cov = 1.0

controller = mppi_py.SVGuidedMPPI(common_p, svg_p)
controller.set_obstacle_map(grid_map)
controller.set_reference_map(ref_sdf)

# 4. 机器人初始状态 [x, y, yaw, vel, steer]
state = np.array([-8.0, -4.0, 0.0, 0.0, 0.0])

# === 可视化循环 ===
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.imshow(obs_data.T, origin='lower', extent=[-10, 10, -10, 10], cmap='Reds', alpha=0.3)
ax.plot(x, 0.5*x, 'k--', label='Ref Path') # 画出参考线

robot_patch = Rectangle((0,0), 0.6, 0.3, color='blue')
ax.add_patch(robot_patch)
traj_line, = ax.plot([], [], 'g-', lw=2)
particles_scatter = ax.scatter([], [], c='gray', s=1, alpha=0.5)

def update(frame):
    global state
    
    # 1. 计算控制
    ctrl, collision_rate = controller.solve(state)
    steer_cmd = ctrl[0, 0] # 取第一个控制量
    
    # 2. 模拟物理 (Kinematic Bicycle)
    x, y, yaw, v, steer = state
    dt = 0.05
    
    # 速度控制 (P控制器趋向于 3.0)
    acc = 1.0 * (3.0 - v)
    
    # 运动学更新
    beta = np.arctan(0.5 * np.tan(steer))
    dx = v * np.cos(yaw + beta)
    dy = v * np.sin(yaw + beta)
    dyaw = v * np.sin(beta) / ROBOT_L
    
    # 更新状态
    state[0] += dx * dt
    state[1] += dy * dt
    state[2] += dyaw * dt
    state[3] += acc * dt
    # 模拟转向延迟
    state[4] += (steer_cmd - steer) * dt * 5.0 
    
    # 3. 获取可视化数据
    # 最优预测轨迹
    best_traj_tuple = controller.get_predictive_seq(state, ctrl)
    best_traj = best_traj_tuple[0] # tuple第0个是状态序列 MatrixXd
    
    # 候选采样轨迹
    candidates, _ = controller.get_state_seq_candidates(50)
    
    # 4. 绘图更新
    # 更新机器人
    robot_patch.set_xy((state[0]-0.3, state[1]-0.15))
    # 这里省略了 matplotlib 的旋转 transform，实际需要加上
    
    # 更新轨迹
    traj_line.set_data(best_traj[:, 0], best_traj[:, 1])
    
    # 更新粒子
    px, py = [], []
    for c in candidates:
        px.extend(c[:, 0])
        py.extend(c[:, 1])
    particles_scatter.set_offsets(np.c_[px, py])
    
    ax.set_title(f"V={v:.2f}, Col={collision_rate:.2f}")
    return robot_patch, traj_line, particles_scatter

ani = FuncAnimation(fig, update, frames=200, interval=50, blit=True)
plt.show()