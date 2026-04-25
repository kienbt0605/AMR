"""
===================================================================
  AMR Odometry Simulation - Differential Drive Robot
  Point-to-Point Navigation with Odometry Calculation
===================================================================
  Thông số xe:
    - Khoảng cách tâm 2 bánh (L) : 205 mm
    - Đường kính bánh xe (D)      : 66 mm
    - Encoder ticks / vòng        : 495 ticks
  
  Công thức odometry:
    - Chu vi bánh xe: C = π * D
    - Quãng đường mỗi tick: d_tick = C / ticks_per_rev
    - Quãng đường bánh trái:  SL = ticksL * d_tick
    - Quãng đường bánh phải:  SR = ticksR * d_tick
    - Quãng đường trung bình: S  = (SL + SR) / 2
    - Góc quay:    delta_theta = (SL - SR) / L
    - Toạ độ mới:  x += S * cos(theta)
    -              y += S * sin(theta)
===================================================================
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch
import matplotlib.animation as animation
from matplotlib.widgets import Button, Slider
import math
import heapq
import socket
import threading
import queue
import time

# =====================================================================
#  THÔNG SỐ ROBOT
# =====================================================================
WHEEL_BASE = 205.0        # Khoảng cách tâm 2 bánh xe (mm)
WHEEL_DIAMETER = 66.0     # Đường kính bánh xe (mm)
TICKS_PER_REV = 495       # Số tick encoder mỗi vòng quay

WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER          # Chu vi bánh xe (mm)
DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REV # Quãng đường mỗi tick (mm)

# Thông số map
GRID_SIZE = 100           # Kích thước 1 ô lưới (mm)
MAP_WIDTH = 2000          # Chiều rộng map (mm)
MAP_HEIGHT = 2000         # Chiều cao map (mm)

# Thông số mô phỏng
ROBOT_SPEED = 200.0       # Tốc độ robot (mm/s)
DT = 0.05                 # Bước thời gian (s)
TURN_SPEED = 2.0          # Tốc độ quay (rad/s)
ROBOT_BODY_LENGTH = 150   # Chiều dài thân xe để vẽ (mm)
ROBOT_BODY_WIDTH = 205    # Chiều rộng thân xe để vẽ (mm)

# Thông số A* Pathfinding
OBSTACLE_CLEARANCE = 100  # Khoảng cách an toàn từ vật cản (mm)

# =====================================================================
#  CẤU HÌNH WIFI - KẾT NỐI ESP32
# =====================================================================
ESP32_IP   = '192.168.4.1'   # IP mặc định khi ESP32 phát AP
ESP32_PORT = 8080


# =====================================================================
#  LỚP ROBOT DIFFERENTIAL DRIVE
# =====================================================================
class DifferentialDriveRobot:
    """Mô hình xe 2 bánh vi sai với tính toán odometry."""

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        # Vị trí và hướng hiện tại
        self.x = x
        self.y = y
        self.theta = theta  # Góc hướng tuyệt đối (rad)
        self.reference_theta = theta  # Trục X tham chiếu mới

        # Tổng encoder tích luỹ
        self.total_ticks_left = 0
        self.total_ticks_right = 0

        # Vận tốc hiện tại
        self.v_left = 0.0       # Vận tốc bánh trái (mm/s)
        self.v_right = 0.0      # Vận tốc bánh phải (mm/s)
        self.v_linear = 0.0     # Vận tốc tịnh tiến xe (mm/s)
        self.omega = 0.0        # Vận tốc góc xe (rad/s)

        # Lịch sử di chuyển
        self.path_history = [(x, y)]
        self.theta_history = [theta]

    def compute_odometry(self, delta_ticks_left, delta_ticks_right):
        """
        Tính toán odometry từ số tick encoder.
        
        Công thức:
            SL = delta_ticks_left * distance_per_tick
            SR = delta_ticks_right * distance_per_tick
            S  = (SL + SR) / 2
            delta_theta = (SL - SR) / L
            x += S * cos(theta + delta_theta / 2)
            y += S * sin(theta + delta_theta / 2)
            theta += delta_theta
        
        Vận tốc (theo encoder):
            v_left  = (Δtick_L × π × D) / (N × Δt)    [mm/s]
            v_right = (Δtick_R × π × D) / (N × Δt)    [mm/s]
            v_linear = (v_left + v_right) / 2           [mm/s]
            omega    = (v_left - v_right) / L           [rad/s]
        """
        # Quãng đường từng bánh xe
        SL = delta_ticks_left * DISTANCE_PER_TICK   # Quãng đường bánh trái
        SR = delta_ticks_right * DISTANCE_PER_TICK   # Quãng đường bánh phải

        # Quãng đường trung bình
        S = (SL + SR) / 2.0

        # Thay đổi góc
        delta_theta = (SL - SR) / WHEEL_BASE

        # === Tính vận tốc từ encoder ===
        # v = (Δtick × π × D) / (N × Δt)
        if DT > 0:
            self.v_left  = (delta_ticks_left  * math.pi * WHEEL_DIAMETER) / (TICKS_PER_REV * DT)
            self.v_right = (delta_ticks_right * math.pi * WHEEL_DIAMETER) / (TICKS_PER_REV * DT)
        else:
            self.v_left  = 0.0
            self.v_right = 0.0

        # Vận tốc tịnh tiến xe = trung bình 2 bánh
        self.v_linear = (self.v_left + self.v_right) / 2.0

        # Vận tốc góc xe = (v_left - v_right) / L
        self.omega = (self.v_left - self.v_right) / WHEEL_BASE

        # Cập nhật toạ độ (dùng mid-point approximation)
        self.x += S * math.cos(self.theta + delta_theta / 2.0)
        self.y += S * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Chuẩn hoá theta về [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Cập nhật tổng ticks
        self.total_ticks_left += delta_ticks_left
        self.total_ticks_right += delta_ticks_right

        # Lưu lịch sử
        self.path_history.append((self.x, self.y))
        self.theta_history.append(self.theta)

        return S, delta_theta

    @property
    def relative_theta(self):
        """Góc hướng tương đối so với trục X tham chiếu."""
        rel = self.theta - self.reference_theta
        return math.atan2(math.sin(rel), math.cos(rel))

    def get_position(self):
        return self.x, self.y, self.theta


# =====================================================================
#  TÍNH TOÁN ĐIỀU HƯỚNG POINT-TO-POINT
# =====================================================================
def compute_angle_to_target(robot_x, robot_y, target_x, target_y):
    """Tính góc từ vị trí robot đến điểm đích."""
    dx = target_x - robot_x
    dy = target_y - robot_y
    return math.atan2(dy, dx)


def compute_distance_to_target(robot_x, robot_y, target_x, target_y):
    """Tính khoảng cách giữa 2 điểm (Euclidean)."""
    dx = target_x - robot_x
    dy = target_y - robot_y
    return math.sqrt(dx**2 + dy**2)


def normalize_angle(angle):
    """Chuẩn hoá góc về [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# =====================================================================
#  TẠO CHUỖI LỆNH DI CHUYỂN (tick encoder theo thời gian)
# =====================================================================
def generate_motion_commands(robot, waypoints):
    """
    Sinh chuỗi lệnh encoder ticks cho robot di chuyển qua các waypoint.
    
    Quy trình cho mỗi waypoint:
      1. Tính góc giữa tia mục tiêu mới và tia mục tiêu cũ (trục X mới)
      2. Xoay tại chỗ góc theta tương đối đó
      3. Đi thẳng đến điểm đích
    
    Khi đến cuối 1 mục tiêu, tia mục tiêu đã đi (vector từ vị trí trước
    → waypoint vừa đến) trở thành trục X mới. Góc xoay cho mục tiêu tiếp
    theo = góc hợp giữa tia mục tiêu mới và tia mục tiêu cũ.
    """
    all_commands = []  # Mỗi phần tử: (delta_ticks_L, delta_ticks_R, label)

    sim_x, sim_y, sim_theta = robot.x, robot.y, robot.theta

    # Hướng "trục X mới" = hướng tia mục tiêu cũ.
    # Ban đầu dùng hướng xe hiện tại làm trục X tham chiếu.
    prev_ray_angle = sim_theta

    for i, (tx, ty) in enumerate(waypoints):
        # === BƯỚC 1: Tính góc cần xoay ===
        # Hướng tia mục tiêu mới (từ vị trí hiện tại → waypoint tiếp theo)
        new_ray_angle = compute_angle_to_target(sim_x, sim_y, tx, ty)

        # Góc xoay = góc hợp giữa tia mới và tia cũ (trục X mới)
        angle_error = normalize_angle(new_ray_angle - prev_ray_angle)

        # === BƯỚC 2: Xoay tại chỗ ===
        if abs(angle_error) > 0.01:  # Ngưỡng nhỏ tránh xoay không cần thiết
            # Tính quãng đường cung mỗi bánh cần quay
            # Khi xoay tại chỗ: bánh trái và phải quay ngược nhau
            # arc = (angle * L) / 2
            arc_distance = abs(angle_error) * WHEEL_BASE / 2.0

            # Số tick cần cho mỗi bánh
            ticks_needed = arc_distance / DISTANCE_PER_TICK

            # Chia thành nhiều bước nhỏ
            turn_speed_ticks = TURN_SPEED * WHEEL_BASE / 2.0 / DISTANCE_PER_TICK * DT
            if turn_speed_ticks < 1:
                turn_speed_ticks = 1

            steps = max(1, int(ticks_needed / turn_speed_ticks))
            ticks_per_step = ticks_needed / steps

            for s in range(steps):
                if angle_error > 0:
                    # Quay trái (CCW): delta_theta > 0 => SL > SR
                    dL = ticks_per_step
                    dR = -ticks_per_step
                else:
                    # Quay phải (CW): delta_theta < 0 => SL < SR
                    dL = -ticks_per_step
                    dR = ticks_per_step

                all_commands.append((dL, dR, f"TURN_WP{i}"))

            # Cập nhật sim_theta = hướng tia mục tiêu mới (sau khi xoay xong)
            sim_theta = new_ray_angle

        # === Lệnh RESET ENCODER (trước khi đi thẳng) ===
        all_commands.append((0, 0, f"RESET_ENCODER_WP{i}"))

        # === BƯỚC 3: Đi thẳng ===
        distance = compute_distance_to_target(sim_x, sim_y, tx, ty)

        if distance > 1.0:  # Ngưỡng tối thiểu
            # Tốc độ tịnh tiến (ticks/step)
            speed_ticks = ROBOT_SPEED / DISTANCE_PER_TICK * DT
            if speed_ticks < 1:
                speed_ticks = 1

            total_ticks = distance / DISTANCE_PER_TICK
            steps = max(1, int(total_ticks / speed_ticks))
            ticks_per_step = total_ticks / steps

            for s in range(steps):
                all_commands.append((ticks_per_step, ticks_per_step, f"STRAIGHT_WP{i}"))

            # Cập nhật vị trí mô phỏng
            sim_x = tx
            sim_y = ty

        # Sau khi đến waypoint, tia vừa đi (new_ray_angle) trở thành trục X mới
        prev_ray_angle = new_ray_angle
        all_commands.append((0, 0, f"RESET_REF_WP{i}"))

    return all_commands, sim_x, sim_y, sim_theta


# =====================================================================
#  A* PATHFINDING ALGORITHM
# =====================================================================
class AStarPathfinder:
    """Thuật toán A* tìm đường đi ngắn nhất trên occupancy grid.
    
    - Hỗ trợ 8 hướng di chuyển (lên, xuống, trái, phải, 4 đường chéo)
    - Inflation layer để giữ khoảng cách an toàn từ vật cản
    - Heuristic: Euclidean distance
    """

    # 8 hướng neighbor: (delta_col, delta_row, cost)
    NEIGHBORS = [
        ( 0,  1, 1.0),    # Lên
        ( 0, -1, 1.0),    # Xuống
        ( 1,  0, 1.0),    # Phải
        (-1,  0, 1.0),    # Trái
        ( 1,  1, 1.414),  # Chéo phải-trên
        ( 1, -1, 1.414),  # Chéo phải-dưới
        (-1,  1, 1.414),  # Chéo trái-trên
        (-1, -1, 1.414),  # Chéo trái-dưới
    ]

    def __init__(self, map_width, map_height, grid_size, clearance):
        self.map_w = map_width
        self.map_h = map_height
        self.grid_size = grid_size
        self.cols = (map_width // grid_size) + 1    # Số cột (21)
        self.rows = (map_height // grid_size) + 1   # Số hàng (21)
        self.clearance_cells = max(1, int(clearance / grid_size))  # Số ô inflation

    def world_to_grid(self, x, y):
        """Chuyển toạ độ thế giới (mm) sang toạ độ lưới (col, row)."""
        col = int((x + self.grid_size / 2.0) // self.grid_size)
        row = int((y + self.grid_size / 2.0) // self.grid_size)
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        return col, row

    def grid_to_world(self, col, row):
        """Chuyển toạ độ lưới (col, row) sang toạ độ thế giới (mm) - tâm ô."""
        x = col * self.grid_size
        y = row * self.grid_size
        return x, y

    def build_occupancy_grid(self, obstacles, obstacle_radius, obstacle_rect_size):
        """Xây dựng occupancy grid từ danh sách vật cản.
        
        0 = ô trống, 1 = ô bị chiếm bởi vật cản.
        Kiểm tra từng ô xem tâm ô có nằm trong vật cản không.
        """
        grid = np.zeros((self.rows, self.cols), dtype=int)
        gs = self.grid_size

        for (ox, oy, shape, patch_obj, marker) in obstacles:
            if shape == 'circle':
                r = obstacle_radius
                # Xác định vùng ô có thể bị ảnh hưởng
                min_c, min_r = self.world_to_grid(ox - r, oy - r)
                max_c, max_r = self.world_to_grid(ox + r, oy + r)
                for rr in range(min_r, max_r + 1):
                    for cc in range(min_c, max_c + 1):
                        cx, cy = self.grid_to_world(cc, rr)
                        dist = math.sqrt((cx - ox)**2 + (cy - oy)**2)
                        if dist <= r:
                            grid[rr][cc] = 1
            else:  # rectangle
                half = obstacle_rect_size / 2.0
                min_c, min_r = self.world_to_grid(ox - half, oy - half)
                max_c, max_r = self.world_to_grid(ox + half, oy + half)
                for rr in range(min_r, max_r + 1):
                    for cc in range(min_c, max_c + 1):
                        cx, cy = self.grid_to_world(cc, rr)
                        if abs(cx - ox) <= half and abs(cy - oy) <= half:
                            grid[rr][cc] = 1
        return grid

    def inflate_grid(self, grid):
        """Mở rộng vùng cấm quanh vật cản thêm clearance_cells ô.
        
        Tạo 'vùng đệm' an toàn xung quanh vật cản để robot không đi quá gần.
        """
        inflated = grid.copy()
        obstacle_cells = np.argwhere(grid == 1)  # Tìm tất cả ô bị chiếm

        for (r, c) in obstacle_cells:
            for dr in range(-self.clearance_cells, self.clearance_cells + 1):
                for dc in range(-self.clearance_cells, self.clearance_cells + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.rows and 0 <= nc < self.cols:
                        inflated[nr][nc] = 1
        return inflated

    @staticmethod
    def _heuristic(a, b):
        """Euclidean distance heuristic."""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def astar(self, start_xy, goal_xy, obstacles, obstacle_radius, obstacle_rect_size):
        """Chạy thuật toán A* tìm đường đi từ start_xy đến goal_xy.
        
        Parameters:
            start_xy: (x, y) toạ độ thế giới bắt đầu (mm)
            goal_xy:  (x, y) toạ độ thế giới đích (mm)
            obstacles: danh sách vật cản [(x, y, shape, patch, marker), ...]
            obstacle_radius: bán kính vật tròn (mm)
            obstacle_rect_size: cạnh vật chữ nhật (mm)
        
        Returns:
            path: danh sách [(x, y), ...] toạ độ thế giới, hoặc None nếu không tìm được.
            grid: occupancy grid đã inflate (để hiển thị)
        """
        # Xây dựng occupancy grid
        raw_grid = self.build_occupancy_grid(obstacles, obstacle_radius, obstacle_rect_size)
        grid = self.inflate_grid(raw_grid)

        start = self.world_to_grid(*start_xy)
        goal = self.world_to_grid(*goal_xy)

        # Kiểm tra điểm đầu/cuối có hợp lệ không
        if grid[start[1]][start[0]] == 1:
            return None, grid  # Điểm xuất phát bị chặn
        if grid[goal[1]][goal[0]] == 1:
            return None, grid  # Điểm đích bị chặn

        # Priority queue: (f_score, (col, row))
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            current_f, current = heapq.heappop(open_set)

            if current == goal:
                # Truy ngược đường đi
                path = []
                node = current
                while node in came_from:
                    path.append(self.grid_to_world(*node))
                    node = came_from[node]
                path.append(self.grid_to_world(*start))
                path.reverse()
                return path, grid

            for dc, dr, move_cost in self.NEIGHBORS:
                nc, nr = current[0] + dc, current[1] + dr
                neighbor = (nc, nr)

                # Kiểm tra giới hạn
                if not (0 <= nc < self.cols and 0 <= nr < self.rows):
                    continue
                # Kiểm tra ô bị chặn
                if grid[nr][nc] == 1:
                    continue

                # Kiểm tra đường chéo: không cho đi chéo qua góc vật cản
                if dc != 0 and dr != 0:
                    if grid[current[1] + dr][current[0]] == 1 or grid[current[1]][current[0] + dc] == 1:
                        continue

                new_g = g_score[current] + move_cost

                if new_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = new_g
                    f = new_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return None, grid  # Không tìm được đường đi


# =====================================================================
#  VISUALISATION - VẼ MÔ PHỎNG
# =====================================================================
class OdometrySimulator:
    """Trình mô phỏng odometry với animation matplotlib."""

    def __init__(self, robot, waypoints, commands):
        self.robot = robot
        self.waypoints = list(waypoints)
        self.commands = list(commands)
        self.current_step = 0
        self.finished = False
        self.simulation_time = 0.0

        # State tracking cho kéo bản đồ (panning)
        self.panning = False
        self.pan_start_x = 0
        self.pan_start_y = 0
        self.pan_start_xlim = (0, 0)
        self.pan_start_ylim = (0, 0)

        # State tracking cho click
        self.planning_x = robot.x
        self.planning_y = robot.y
        self.planning_theta = robot.theta

        # ===== Chế độ đặt vật cản =====
        self.placement_mode = 'waypoint'   # 'waypoint', 'obstacle', hoặc 'astar'
        self.obstacle_shape = 'rectangle'  # 'circle' hoặc 'rectangle'
        self.obstacles = []                # Danh sách: (x, y, shape, patch_object)
        # Kích thước vật cản = 4 ô lưới
        # Hình tròn: diện tích = 4 * GRID_SIZE^2 => bán kính r = sqrt(4/π) * GRID_SIZE
        # Hình chữ nhật: 2 x 2 ô => cạnh = 2 * GRID_SIZE
        self.obstacle_radius = math.sqrt(4.0 / math.pi) * GRID_SIZE  # ~112.8 mm
        self.obstacle_rect_size = 2 * GRID_SIZE                       # 200 mm (2x2 ô)
        self.collided = False            # Trạng thái va chạm
        self.robot_collision_radius = 100 # Bán kính thân xe để kiểm tra va chạm (mm)

        # ===== A* Pathfinding =====
        self.pathfinder = AStarPathfinder(MAP_WIDTH, MAP_HEIGHT, GRID_SIZE, OBSTACLE_CLEARANCE)
        self.astar_grid_patches = []      # Các patch hiển thị occupancy grid
        self.astar_wp_markers = []        # Các marker waypoint A*
        self.astar_goal_marker = None     # Marker điểm đích A*
        self.astar_no_path_text = None    # Text thông báo không tìm được đường

        # ===== WiFi Socket kết nối ESP32 =====
        self.wifi_socket = None
        self.wifi_connected = False
        self.wifi_lock = threading.Lock()
        self.wifi_queue = queue.Queue()
        self.physical_arrived = True
        
        # Bắt đầu luồng quản lý WiFi
        threading.Thread(target=self._wifi_manager_task, daemon=True).start()

        class DummyRobot:
            def __init__(self, x, y, th):
                self.x = x; self.y = y; self.theta = th
        dummy = DummyRobot(self.planning_x, self.planning_y, self.planning_theta)
        _, px, py, pth = generate_motion_commands(dummy, self.waypoints)
        self.planning_x, self.planning_y, self.planning_theta = px, py, pth


        # Lưu trạng thái ban đầu
        self.start_x = robot.x
        self.start_y = robot.y
        self.start_theta = robot.theta

        # ======= Setup Figure =======
        self.fig, self.ax = plt.subplots(1, 1, figsize=(10, 10))
        
        # Mở full màn hình nhưng giữ lại các thanh điều khiển cửa sổ
        manager = plt.get_current_fig_manager()
        try:
            manager.window.state('zoomed')  # Dành cho backend TkAgg mặc định trên Windows
        except Exception:
            try:
                manager.window.showMaximized()  # Dành cho backend QtAgg
            except Exception:
                pass
                
        self.fig.patch.set_facecolor('#1a1a2e')
        self.ax.set_facecolor('#16213e')

        # Tiêu đề
        self.fig.suptitle('         AMR Odometry Simulation - Differential Drive',
                          fontsize=16, fontweight='bold', color='#e94560',
                          x=0.535, y=0.97)

        self._setup_grid()
        self._setup_plots()

        # Textbox thông tin (đặt ở vùng trống bên trái figure)
        self.info_text = self.fig.text(
            0.015, 0.90, '', transform=self.fig.transFigure,
            fontsize=9.5, verticalalignment='top', horizontalalignment='left',
            fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#0f3460',
                      edgecolor='#e94560', alpha=0.9),
            color='#ffffff'
        )

        # Textbox công thức (đặt ở vùng trống bên trái figure, phía dưới)
        formula_text = (
            " ODOMETRY & VELOCITY FORMULAS \n"
            "------------------------------\n"
            f" C = π×D  = {WHEEL_CIRCUMFERENCE:.2f} mm\n"
            f" d/tick   = {DISTANCE_PER_TICK:.4f} mm\n"
            " SL = ticksL × d/tick\n"
            " SR = ticksR × d/tick\n"
            " S  = (SL + SR) / 2\n"
            " Δθ = (SL - SR) / L\n"
            " x += S×cos(θ + Δθ/2)\n"
            " y += S×sin(θ + Δθ/2)\n"
            "------------------------------\n"
            " vL = (ΔtickL×π×D) / (N×Δt)\n"
            " vR = (ΔtickR×π×D) / (N×Δt)\n"
            " v  = (vL + vR)/2     [mm/s]\n"
            " ω  = (vL - vR)/L     [rad/s]"
        )
        self.fig.text(
            0.015, 0.50, formula_text, transform=self.fig.transFigure,
            fontsize=9.5, verticalalignment='top', horizontalalignment='left',
            fontfamily='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#0f3460',
                      edgecolor='#533483', alpha=0.9),
            color='#00d2ff'
        )

        # Lịch sử các bước (để Undo)
        self.user_actions = []

        # Tọa độ các nút (X=0.85 trên figure, xếp dọc từ trên xuống dưới ở dải bên phải)
        # ====== Nút chuyển chế độ: Waypoint / Obstacle / A* ======
        self.ax_mode = plt.axes([0.85, 0.65, 0.13, 0.06])
        self.btn_mode = Button(self.ax_mode, '[W] Waypoint', color='#0f3460', hovercolor='#1a5276')
        self.btn_mode.label.set_color('#00ff88')
        self.btn_mode.label.set_fontweight('bold')
        self.btn_mode.on_clicked(self._toggle_placement_mode)

        # ====== Nút chuyển hình dạng vật cản ======
        self.ax_shape = plt.axes([0.85, 0.55, 0.13, 0.06])
        self.btn_shape = Button(self.ax_shape, '▬ Rect', color='#533483', hovercolor='#6c44a2')
        self.btn_shape.label.set_color('#ffcc00')
        self.btn_shape.label.set_fontweight('bold')
        self.btn_shape.on_clicked(self._toggle_obstacle_shape)

        # ====== Nút chế độ A* Path ======
        self.ax_astar = plt.axes([0.85, 0.45, 0.13, 0.06])
        self.btn_astar = Button(self.ax_astar, '[A*] Path', color='#1a5276', hovercolor='#2471a3')
        self.btn_astar.label.set_color('#ffcc00')
        self.btn_astar.label.set_fontweight('bold')
        self.btn_astar.on_clicked(self._activate_astar_mode)

        # ====== Nút Undo ======
        self.ax_undo = plt.axes([0.85, 0.35, 0.13, 0.06])
        self.btn_undo = Button(self.ax_undo, '↶ Undo', color='#e94560', hovercolor='#ff6b6b')
        self.btn_undo.label.set_color('white')
        self.btn_undo.label.set_fontweight('bold')
        self.btn_undo.on_clicked(self.undo_action)

        # ====== Slider Vận Tốc ======
        # (Để tiết kiệm bề ngang vì lề hẹp, ta đổi chữ Speed thành V:)
        self.ax_speed = plt.axes([0.85, 0.25, 0.13, 0.04], facecolor='#16213e')
        self.speed_slider = Slider(
            ax=self.ax_speed,
            label='V: ',
            valmin=50.0,
            valmax=600.0,
            valinit=ROBOT_SPEED,
            valfmt='%0.0f',
            color='#00ff88'
        )
        self.speed_slider.label.set_color('#ffffff')
        self.speed_slider.label.set_fontweight('bold')
        self.speed_slider.valtext.set_color('#ffcc00')
        self.speed_slider.valtext.set_fontweight('bold')
        
        self.fraction_in_current_step = 0.0

    def _setup_grid(self):
        """Vẽ lưới map."""
        ax = self.ax
        ax.set_xlim(-100, MAP_WIDTH + 100)
        ax.set_ylim(-100, MAP_HEIGHT + 100)
        ax.set_aspect('equal')
        ax.set_xlabel('X (mm)', color='#a0a0a0', fontsize=11)
        ax.set_ylabel('Y (mm)', color='#a0a0a0', fontsize=11)

        # Vẽ ô lưới sao cho tâm ô lưới nằm ở các toạ độ chẵn (0, 100, 200...)
        offset = GRID_SIZE // 2
        for x in range(-offset, MAP_WIDTH + offset + 1, GRID_SIZE):
            ax.axvline(x, color='#2a2a4a', linewidth=0.5, alpha=0.6)
        for y in range(-offset, MAP_HEIGHT + offset + 1, GRID_SIZE):
            ax.axhline(y, color='#2a2a4a', linewidth=0.5, alpha=0.6)

        # Vẽ viền map
        border = patches.Rectangle((-offset, -offset), MAP_WIDTH + GRID_SIZE, MAP_HEIGHT + GRID_SIZE,
                                    linewidth=2, edgecolor='#533483',
                                    facecolor='none')
        ax.add_patch(border)

        # Tick marks
        ax.set_xticks(range(0, MAP_WIDTH + 1, GRID_SIZE * 2))
        ax.set_yticks(range(0, MAP_HEIGHT + 1, GRID_SIZE * 2))
        ax.tick_params(colors='#606080', labelsize=8)

    def _setup_plots(self):
        """Khởi tạo các thành phần vẽ."""
        ax = self.ax

        # Đường đi (trail)
        self.path_line, = ax.plot([], [], color='#00d2ff', linewidth=2,
                                  alpha=0.8, label='Odometry Path')
        # Trail mờ dần
        self.trail_scatter = ax.scatter([], [], c=[], cmap='cool',
                                        s=3, alpha=0.5)

        # Vẽ waypoints
        wp_x = [w[0] for w in self.waypoints]
        wp_y = [w[1] for w in self.waypoints]
        self.wp_scatter = ax.scatter(wp_x, wp_y, color='#e94560', s=120, zorder=10,
                   marker='*', edgecolors='white', linewidths=0.5,
                   label='Waypoints')

        # Đánh số waypoints
        self.wp_annotations = []
        for i, (wx, wy) in enumerate(self.waypoints):
            an = ax.annotate(f'WP{i}', (wx, wy), textcoords="offset points",
                        xytext=(10, 10), fontsize=9, color='#e94560',
                        fontweight='bold')
            self.wp_annotations.append(an)

        # Đường nối waypoints (kế hoạch)
        plan_x = [self.start_x] + wp_x
        plan_y = [self.start_y] + wp_y
        self.plan_line, = ax.plot(plan_x, plan_y, color='#533483', linewidth=1.5,
                linestyle='--', alpha=0.6, label='Planned Path')

        # Đường A* path
        self.astar_path_line, = ax.plot([], [], color='#ffcc00', linewidth=2.5,
                                         alpha=0.9, linestyle='-', label='A* Path',
                                         zorder=8)

        # Điểm bắt đầu
        ax.scatter([self.start_x], [self.start_y], color='#00ff88',
                   s=150, zorder=10, marker='o', edgecolors='white',
                   linewidths=1, label='Start')
        ax.annotate('START', (self.start_x, self.start_y),
                    textcoords="offset points", xytext=(10, -15),
                    fontsize=9, color='#00ff88', fontweight='bold')


        # Mũi tên hướng xe
        self.direction_arrow = ax.annotate(
            '', xy=(0, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle='->', color='#ff6b6b',
                            lw=2.5, mutation_scale=15),
            zorder=20
        )
        # Khung thân xe (hình tròn)
        self.robot_circle = patches.Circle(
            (0, 0), radius=100,
            linewidth=1.5, edgecolor='#00ff88', facecolor='#0f3460',
            alpha=0.4, zorder=14
        )
        ax.add_patch(self.robot_circle)

        # Chấm tâm xe (màu xanh lá)
        self.robot_center_dot = ax.scatter([0], [0], color='#00ff88', s=60, zorder=21, marker='o')


        # Bánh xe trái
        self.wheel_left = patches.Rectangle(
            (-15, ROBOT_BODY_WIDTH / 2 - 7.5), 30, 15,
            linewidth=1, edgecolor='#ffcc00', facecolor='#ffcc00',
            alpha=0.9, zorder=16
        )
        ax.add_patch(self.wheel_left)

        # Bánh xe phải
        self.wheel_right = patches.Rectangle(
            (-15, -ROBOT_BODY_WIDTH / 2 - 7.5), 30, 15,
            linewidth=1, edgecolor='#ffcc00', facecolor='#ffcc00',
            alpha=0.9, zorder=16
        )
        ax.add_patch(self.wheel_right)

        ax.legend(loc='lower right', fontsize=8,
                  facecolor='#0f3460', edgecolor='#533483',
                  labelcolor='#a0a0a0')

        # Text hiển thị toạ độ khi di chuột
        self.hover_text = ax.text(0, 0, "", color="#00ff88", fontsize=9, 
                                  fontweight='bold', fontfamily='monospace',
                                  bbox=dict(facecolor='#16213e', alpha=0.8, edgecolor='#e94560', boxstyle='round,pad=0.3'),
                                  zorder=30, visible=False)

    def _update_robot_visual(self):
        """Cập nhật vị trí vẽ robot."""
        x, y, theta = self.robot.get_position()

        # Thông số xoay
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        half_w = ROBOT_BODY_WIDTH / 2
        angle_deg = math.degrees(theta)


        # === Mũi tên hướng ===
        arrow_len = 120
        ax_end = x + arrow_len * cos_t
        ay_end = y + arrow_len * sin_t
        self.direction_arrow.xy = (ax_end, ay_end)
        self.direction_arrow.xyann = (x, y)

        # Dùng Affine2D transform từ gốc toạ độ local sang global (x, y) và góc theta
        t_robot = plt.matplotlib.transforms.Affine2D().rotate(theta).translate(x, y) + self.ax.transData
        self.wheel_left.set_transform(t_robot)
        self.wheel_right.set_transform(t_robot)
        self.robot_circle.set_transform(t_robot)
        self.robot_center_dot.set_offsets([(x, y)])

    def _check_collision(self, x, y):
        """Kiểm tra va chạm giữa robot (hình tròn) và các vật cản.
        
        Robot được mô hình hoá là hình tròn với bán kính robot_collision_radius.
        Trả về True nếu có va chạm.
        """
        r_robot = self.robot_collision_radius
        for (ox, oy, shape, patch, marker) in self.obstacles:
            if shape == 'circle':
                # Va chạm 2 hình tròn: khoảng cách tâm < tổng bán kính
                dist = math.sqrt((x - ox)**2 + (y - oy)**2)
                if dist < r_robot + self.obstacle_radius:
                    return True
            else:  # rectangle
                # Va chạm hình tròn - hình chữ nhật
                half = self.obstacle_rect_size / 2
                # Tìm điểm gần nhất trên hình chữ nhật với tâm robot
                closest_x = max(ox - half, min(x, ox + half))
                closest_y = max(oy - half, min(y, oy + half))
                dist = math.sqrt((x - closest_x)**2 + (y - closest_y)**2)
                if dist < r_robot:
                    return True
        return False

    def _update_info(self):
        """Cập nhật thông tin hiển thị."""
        x, y, theta = self.robot.get_position()
        step = self.current_step
        total = len(self.commands)
        label = self.commands[min(step, total - 1)][2] if total > 0 else "IDLE"

        # Hiển thị trạng thái va chạm
        if self.collided:
            status_line = " ⊘ COLLISION! - STOPPED ⊘"
        else:
            status_line = f" Action : {label}"

        info = (
            f" ROBOT STATUS \n"
            f"------------------------------\n"
            f" X      : {x:>9.2f} mm\n"
            f" Y      : {y:>9.2f} mm\n"
            f" θ (rel): {math.degrees(self.robot.relative_theta):>9.2f} °\n"
            f" θ (abs): {math.degrees(theta):>9.2f} °\n"
            f"------------------------------\n"
            f" ENCODER \n"
            f" Ticks L: {self.robot.total_ticks_left:>9.0f}\n"
            f" Ticks R: {self.robot.total_ticks_right:>9.0f}\n"
            f"------------------------------\n"
            f" VELOCITY & TIME \n"
            f" vL     : {self.robot.v_left:>9.2f} mm/s\n"
            f" vR     : {self.robot.v_right:>9.2f} mm/s\n"
            f" v      : {self.robot.v_linear:>9.2f} mm/s\n"
            f" ω      : {self.robot.omega:>9.4f} rad/s\n"
            f" Time   : {self.simulation_time:>9.2f} s\n"
            f"------------------------------\n"
            f" Step   : {step}/{total}\n"
            f"{status_line}"
        )
        self.info_text.set_text(info)

        # Đổi màu textbox khi va chạm
        if self.collided:
            self.info_text.set_bbox(dict(boxstyle='round,pad=0.5', facecolor='#5c0000',
                                         edgecolor='#ff4444', alpha=0.95))
        else:
            self.info_text.set_bbox(dict(boxstyle='round,pad=0.5', facecolor='#0f3460',
                                         edgecolor='#e94560', alpha=0.9))

    def animate(self, frame):
        """Hàm animation cho mỗi frame."""
        if self.collided:
            self._update_robot_visual()
            self._update_info()
            return [self.path_line, self.wheel_left, self.wheel_right,
                    self.robot_circle, self.robot_center_dot, self.info_text]

        steps_per_frame = 3
        multiplier = self.speed_slider.val / ROBOT_SPEED  # Chia cho base speed

        for _ in range(steps_per_frame):
            if self.current_step < len(self.commands):
                advance = multiplier
                dL_sum = 0.0
                dR_sum = 0.0
                consumed_any = False
                
                while advance > 0 and self.current_step < len(self.commands):
                    dL, dR, label = self.commands[self.current_step]
                    
                    if label.startswith("RESET_ENCODER"):
                        self.robot.total_ticks_left = 0
                        self.robot.total_ticks_right = 0
                        self.current_step += 1
                        continue
                        
                    if label.startswith("RESET_REF"):
                        self.robot.reference_theta = self.robot.theta
                        self.current_step += 1
                        continue

                    rem = 1.0 - self.fraction_in_current_step
                    take = min(advance, rem)
                    
                    dL_sum += dL * take
                    dR_sum += dR * take
                    advance -= take
                    self.fraction_in_current_step += take
                    consumed_any = True
                    
                    if self.fraction_in_current_step >= 0.9999:
                        self.current_step += 1
                        self.fraction_in_current_step = 0.0
                        
                if consumed_any:
                    self.simulation_time += DT
                    prev_x, prev_y = self.robot.x, self.robot.y
                    self.robot.compute_odometry(dL_sum, dR_sum)

                    if self._check_collision(self.robot.x, self.robot.y):
                        self.robot.x = prev_x
                        self.robot.y = prev_y
                        self.robot.path_history[-1] = (prev_x, prev_y)
                        self.collided = True
                        self.robot_circle.set_edgecolor('#ff4444')
                        self.robot_circle.set_facecolor('#5c0000')
                        self.robot_circle.set_alpha(0.7)
                        break
            else:
                if not self.finished:
                    self.finished = True
                    # Gán vận tốc về 0 khi tới đích
                    self.robot.v_left = 0.0
                    self.robot.v_right = 0.0
                    self.robot.v_linear = 0.0
                    self.robot.omega = 0.0
                break

        # Cập nhật đường đi
        path_x = [p[0] for p in self.robot.path_history]
        path_y = [p[1] for p in self.robot.path_history]
        self.path_line.set_data(path_x, path_y)

        # Cập nhật robot
        self._update_robot_visual()
        self._update_info()

        return [self.path_line, self.wheel_left, self.wheel_right,
                self.robot_circle, self.robot_center_dot, self.info_text]

    def _toggle_placement_mode(self, event):
        """Chuyển đổi giữa chế độ đặt Waypoint và Obstacle."""
        if self.placement_mode == 'waypoint' or self.placement_mode == 'astar':
            self.placement_mode = 'obstacle'
            self.btn_mode.label.set_text('[O] Obstacle')
            self.btn_mode.color = '#8b0000'
            self.btn_mode.hovercolor = '#a52a2a'
            self.btn_mode.label.set_color('#ffcc00')
            # Reset A* button
            self.btn_astar.color = '#1a5276'
            self.btn_astar.hovercolor = '#2471a3'
        else:
            self.placement_mode = 'waypoint'
            self.btn_mode.label.set_text('[W] Waypoint')
            self.btn_mode.color = '#0f3460'
            self.btn_mode.hovercolor = '#1a5276'
            self.btn_mode.label.set_color('#00ff88')
            # Reset A* button
            self.btn_astar.color = '#1a5276'
            self.btn_astar.hovercolor = '#2471a3'
        self.fig.canvas.draw_idle()

    def _activate_astar_mode(self, event):
        """Kích hoạt chế độ A* Pathfinding."""
        self.placement_mode = 'astar'
        # Cập nhật giao diện nút
        self.btn_mode.label.set_text('[A*] Active')
        self.btn_mode.color = '#b8860b'
        self.btn_mode.hovercolor = '#daa520'
        self.btn_mode.label.set_color('#ffffff')
        # Highlight nút A*
        self.btn_astar.color = '#b8860b'
        self.btn_astar.hovercolor = '#daa520'
        self.fig.canvas.draw_idle()

    def _toggle_obstacle_shape(self, event):
        """Chuyển đổi hình dạng vật cản: hình tròn / hình chữ nhật."""
        if self.obstacle_shape == 'rectangle':
            self.obstacle_shape = 'circle'
            self.btn_shape.label.set_text('● Circle')
            self.btn_shape.color = '#1a5276'
            self.btn_shape.hovercolor = '#2471a3'
        else:
            self.obstacle_shape = 'rectangle'
            self.btn_shape.label.set_text('▬ Rect')
            self.btn_shape.color = '#533483'
            self.btn_shape.hovercolor = '#6c44a2'
        self.fig.canvas.draw_idle()

    def _place_obstacle(self, x, y):
        """Đặt vật cản tại vị trí (x, y)."""
        shape = self.obstacle_shape
        if shape == 'circle':
            r = self.obstacle_radius
            patch = patches.Circle(
                (x, y), radius=r,
                linewidth=2, edgecolor='#ff4444', facecolor='#ff4444',
                alpha=0.35, zorder=5, hatch='//'
            )
        else:  # rectangle
            size = self.obstacle_rect_size
            patch = patches.Rectangle(
                (x - size / 2, y - size / 2), size, size,
                linewidth=2, edgecolor='#ff4444', facecolor='#ff4444',
                alpha=0.35, zorder=5, hatch='//'
            )
        self.ax.add_patch(patch)

        # Vẽ dấu X ở tâm vật cản
        marker = self.ax.scatter([x], [y], color='#ff4444', s=40, zorder=6,
                                  marker='x', linewidths=1.5)

        self.obstacles.append((x, y, shape, patch, marker))
        self.fig.canvas.draw_idle()

    def _remove_obstacle_at(self, x, y):
        """Xóa vật cản gần nhất với vị trí click (nếu click nằm trong vật cản)."""
        for i, (ox, oy, shape, patch, marker) in enumerate(self.obstacles):
            if shape == 'circle':
                dist = math.sqrt((x - ox)**2 + (y - oy)**2)
                if dist <= self.obstacle_radius:
                    patch.remove()
                    marker.remove()
                    self.obstacles.pop(i)
                    self.fig.canvas.draw_idle()
                    return True
            else:  # rectangle
                half = self.obstacle_rect_size / 2
                if abs(x - ox) <= half and abs(y - oy) <= half:
                    patch.remove()
                    marker.remove()
                    self.obstacles.pop(i)
                    self.fig.canvas.draw_idle()
                    return True
        return False

    def _clear_astar_visuals(self):
        """Xóa tất cả các hiển thị liên quan đến A*."""
        # Xóa đường A* path
        self.astar_path_line.set_data([], [])
        # Xóa các patch occupancy grid
        for p in self.astar_grid_patches:
            p.remove()
        self.astar_grid_patches.clear()
        # Xóa các marker waypoint A*
        for m in self.astar_wp_markers:
            m.remove()
        self.astar_wp_markers.clear()
        # Xóa marker điểm đích
        if self.astar_goal_marker is not None:
            self.astar_goal_marker.remove()
            self.astar_goal_marker = None
        # Xóa text thông báo
        if self.astar_no_path_text is not None:
            self.astar_no_path_text.remove()
            self.astar_no_path_text = None

    def _on_astar_click(self, tx, ty):
        """Xử lý click trong chế độ A*: tìm đường và điều hướng robot."""
        # Xóa kết quả A* cũ
        self._clear_astar_visuals()

        # Vị trí hiện tại của robot (planning position)
        start_xy = (self.planning_x, self.planning_y)
        goal_xy = (tx, ty)

        self.astar_goal_marker = self.ax.scatter(
            [tx], [ty], color='#ffcc00', s=200, zorder=12,
            marker='*', edgecolors='white', linewidths=1
        )

        # Chạy A*
        path, inflated_grid = self.pathfinder.astar(
            start_xy, goal_xy, self.obstacles,
            self.obstacle_radius, self.obstacle_rect_size
        )

        # Hiển thị occupancy grid (ô bị inflation)
        gs = GRID_SIZE
        for r in range(self.pathfinder.rows):
            for c in range(self.pathfinder.cols):
                if inflated_grid[r][c] == 1:
                    rect = patches.Rectangle(
                        (c * gs - gs/2.0, r * gs - gs/2.0), gs, gs,
                        linewidth=0, facecolor='#ff4444',
                        alpha=0.15, zorder=2
                    )
                    self.ax.add_patch(rect)
                    self.astar_grid_patches.append(rect)

        if path is None:
            # Không tìm được đường đi
            self.astar_no_path_text = self.ax.text(
                tx, ty + 80, "⚠ NO PATH FOUND!",
                color='#ff4444', fontsize=12, fontweight='bold',
                fontfamily='monospace', ha='center',
                bbox=dict(facecolor='#1a1a2e', alpha=0.9,
                          edgecolor='#ff4444', boxstyle='round,pad=0.4'),
                zorder=30
            )
            self.fig.canvas.draw_idle()
            return

        # Rút gọn đường đi A* (Path Smoothing): Bỏ các điểm trung gian trên cùng 1 đường thẳng
        simplified_path = [path[0]]
        for i in range(1, len(path) - 1):
            prev = simplified_path[-1]
            curr = path[i]
            nxt = path[i+1]
            
            # Tính vector hướng
            dx1, dy1 = curr[0] - prev[0], curr[1] - prev[1]
            dx2, dy2 = nxt[0] - curr[0], nxt[1] - curr[1]
            
            len1 = math.sqrt(dx1**2 + dy1**2)
            len2 = math.sqrt(dx2**2 + dy2**2)
            if len1 == 0 or len2 == 0: continue
            
            dir1 = (dx1/len1, dy1/len1)
            dir2 = (dx2/len2, dy2/len2)
            
            # Nếu hướng thay đổi, giữ lại điểm curr (là góc cua)
            if abs(dir1[0] - dir2[0]) > 1e-4 or abs(dir1[1] - dir2[1]) > 1e-4:
                simplified_path.append(curr)
                
        simplified_path.append(path[-1])
        path = simplified_path

        # Vẽ đường A* path
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        self.astar_path_line.set_data(path_x, path_y)

        # Vẽ các điểm trung gian trên đường A*
        for i, (px, py) in enumerate(path):
            m = self.ax.scatter([px], [py], color='#ffcc00', s=30, zorder=9,
                                marker='o', alpha=0.7, edgecolors='#b8860b',
                                linewidths=0.5)
            self.astar_wp_markers.append(m)

        # Chuyển A* path thành waypoints cho robot
        # Bỏ qua điểm đầu (vị trí hiện tại), thêm điểm đích thực tế cuối cùng
        astar_waypoints = list(path[1:])  # Bỏ điểm đầu
        # Thay điểm cuối bằng toạ độ click chính xác
        if astar_waypoints:
            astar_waypoints[-1] = (tx, ty)

        # Sinh motion commands từ A* waypoints
        class SimState:
            def __init__(self, x, y, th):
                self.x = x; self.y = y; self.theta = th

        for i, (wx, wy) in enumerate(astar_waypoints):
            # Tính toạ độ local tương đối so với trục X mới
            px, py, ptheta = self.planning_x, self.planning_y, self.planning_theta
            dx = wx - px
            dy = wy - py
            dist = math.sqrt(dx**2 + dy**2)
            ray_angle = math.atan2(dy, dx)
            angle_error = normalize_angle(ray_angle - ptheta)
            local_x = dist * math.cos(angle_error)
            local_y = dist * math.sin(angle_error)

            state = SimState(px, py, ptheta)
            new_cmds, self.planning_x, self.planning_y, self.planning_theta = \
                generate_motion_commands(state, [(wx, wy)])

            idx = len(self.waypoints)
            self.waypoints.append((wx, wy))
            new_cmds = [(dL, dR, lbl.replace('WP0', f'WP{idx}')) for dL, dR, lbl in new_cmds]
            self.commands.extend(new_cmds)

            # ★ Gửi toạ độ local waypoint A* xuống ESP32 qua WiFi
            self.wifi_send_target(local_x, local_y)

        # Cập nhật hiển thị waypoints
        self.wp_scatter.set_offsets(self.waypoints)
        for an in self.wp_annotations:
            an.remove()
        self.wp_annotations.clear()
        for i, (wx, wy) in enumerate(self.waypoints):
            an = self.ax.annotate(f'WP{i}', (wx, wy), textcoords="offset points",
                        xytext=(10, 10), fontsize=7, color='#ffcc00',
                        fontweight='bold', alpha=0.7)
            self.wp_annotations.append(an)

        # Cập nhật planned path
        plan_x = [self.start_x] + [w[0] for w in self.waypoints]
        plan_y = [self.start_y] + [w[1] for w in self.waypoints]
        self.plan_line.set_data(plan_x, plan_y)

        self.finished = False
        self.fig.canvas.draw_idle()

    def _real_reset(self):
        """Khôi phục mô phỏng về trạng thái khởi tạo."""
        self.robot = DifferentialDriveRobot(self.start_x, self.start_y, self.start_theta)
        
        self.waypoints.clear()
        self.commands.clear()
        self.current_step = 0
        self.finished = False
        self.simulation_time = 0.0
        self.fraction_in_current_step = 0.0
        
        # Xóa hàng đợi WiFi và gửi lệnh dừng
        with self.wifi_queue.mutex:
            self.wifi_queue.queue.clear()
        
        if self.wifi_connected:
            try:
                self.wifi_socket.sendall(b"STOP\n")
            except:
                pass

        self.planning_x = self.start_x
        self.planning_y = self.start_y
        self.planning_theta = self.start_theta
        self.collided = False

        self.robot_circle.set_edgecolor('#00ff88')
        self.robot_circle.set_facecolor('#0f3460')
        self.robot_circle.set_alpha(0.4)

        self.wp_scatter.set_offsets(np.empty((0, 2)))
        for an in self.wp_annotations:
            an.remove()
        self.wp_annotations.clear()
        
        self.plan_line.set_data([self.start_x], [self.start_y])
        self.path_line.set_data([], [])

        for (ox, oy, shape, patch, marker) in self.obstacles:
            patch.remove()
            marker.remove()
        self.obstacles.clear()

        self._clear_astar_visuals()

        self.placement_mode = 'waypoint'
        self.btn_mode.label.set_text('[W] Waypoint')
        self.btn_mode.color = '#0f3460'
        self.btn_mode.hovercolor = '#1a5276'
        self.btn_mode.label.set_color('#00ff88')
        self.btn_astar.color = '#1a5276'
        self.btn_astar.hovercolor = '#2471a3'

    def undo_action(self, event):
        """Lùi lại 1 hành động trước đó."""
        if not hasattr(self, 'user_actions') or not self.user_actions:
            self._real_reset()
            self._update_robot_visual()
            self._update_info()
            self.fig.canvas.draw_idle()
            return
            
        old_step = self.current_step
        self.user_actions.pop()
        actions_to_replay = list(self.user_actions)
        
        self._real_reset()
        
        for action in actions_to_replay:
            atype = action[0]
            if atype == 'add_obstacle':
                old_shape = self.obstacle_shape
                self.obstacle_shape = action[3]
                self._place_obstacle(action[1], action[2])
                self.obstacle_shape = old_shape
            elif atype == 'remove_obstacle':
                self._remove_obstacle_at(action[1], action[2])
            elif atype == 'astar':
                self._on_astar_click(action[1], action[2])
            elif atype == 'waypoint':
                self._add_waypoint(action[1], action[2])
                
        # Fast-forward robot physical simulation
        target_step = min(old_step, len(self.commands))
        for i in range(target_step):
            dL, dR, label = self.commands[i]
            if label.startswith("RESET_ENCODER"):
                self.robot.total_ticks_left = 0
                self.robot.total_ticks_right = 0
            elif label.startswith("RESET_REF"):
                self.robot.reference_theta = self.robot.theta
            else:
                self.simulation_time += DT
                self.robot.compute_odometry(dL, dR)
        self.current_step = target_step
        self.fraction_in_current_step = 0.0
        
        # Đồng bộ lại path visual
        path_x = [p[0] for p in self.robot.path_history]
        path_y = [p[1] for p in self.robot.path_history]
        self.path_line.set_data(path_x, path_y)
        
        self._update_robot_visual()
        self._update_info()
        self.fig.canvas.draw_idle()

    # =====================================================
    # WiFi: Kết nối và gửi lệnh đến ESP32
    # =====================================================
    def _wifi_manager_task(self):
        """Luồng chạy ngầm quản lý việc gửi lệnh WiFi tuần tự."""
        buffer = ""
        while True:
            if not self.wifi_connected:
                with self.wifi_lock:
                    try:
                        self.wifi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.wifi_socket.settimeout(0.05) # Timeout cực ngắn để recv không block
                        self.wifi_socket.connect((ESP32_IP, ESP32_PORT))
                        self.wifi_connected = True
                        print(f"[WiFi] Đã kết nối ESP32 tại {ESP32_IP}:{ESP32_PORT}")
                    except:
                        self.wifi_connected = False
                
                if not self.wifi_connected:
                    time.sleep(1.0)
                    continue

            # 1. Đọc STATUS từ ESP32 để biết robot đã đến nơi chưa
            try:
                data = self.wifi_socket.recv(1024).decode()
                if data:
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.startswith("STATUS,"):
                            parts = line.split(',')
                            if len(parts) >= 5:
                                arrived = int(parts[4])
                                self.physical_arrived = (arrived == 1)
            except socket.timeout:
                pass
            except Exception as e:
                self.wifi_connected = False
                continue

            # 2. Nếu đã đến nơi và có lệnh mới trong hàng đợi -> Gửi lệnh tiếp theo
            if self.physical_arrived and not self.wifi_queue.empty():
                x, y = self.wifi_queue.get()
                try:
                    cmd = f"GO,{x:.1f},{y:.1f}\n"
                    with self.wifi_lock:
                        self.wifi_socket.sendall(cmd.encode())
                    print(f"[WiFi] Đang gửi tuần tự lệnh tiếp theo: {cmd.strip()}")
                    
                    self.physical_arrived = False # Đánh dấu đang chạy để không gửi đè
                    
                    # Đợi một chút để ESP32 kịp xử lý và đổi trạng thái arrived thành 0
                    time.sleep(0.2)
                    buffer = "" # Xoá rác
                except Exception as e:
                    print(f"[WiFi] Lỗi gửi: {e}")
                    self.wifi_connected = False
                    
            time.sleep(0.02)

    def wifi_send_target(self, x, y):
        """Thêm toạ độ mục tiêu (X, Y) vào hàng đợi để gửi tuần tự xuống ESP32."""
        self.wifi_queue.put((x, y))

    def _add_waypoint(self, tx, ty):
        """Thêm 1 waypoint."""
        self.waypoints.append((tx, ty))
        idx = len(self.waypoints) - 1
        
        # Tính toạ độ local tương đối so với trục X mới
        px, py, ptheta = self.planning_x, self.planning_y, self.planning_theta
        dx = tx - px
        dy = ty - py
        dist = math.sqrt(dx**2 + dy**2)
        ray_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(ray_angle - ptheta)
        local_x = dist * math.cos(angle_error)
        local_y = dist * math.sin(angle_error)
        
        class SimState:
            def __init__(self, x, y, th):
                self.x = x; self.y = y; self.theta = th
        state = SimState(px, py, ptheta)
        
        new_cmds, self.planning_x, self.planning_y, self.planning_theta = generate_motion_commands(state, [(tx, ty)])
        new_cmds = [(dL, dR, lbl.replace('WP0', f'WP{idx}')) for dL, dR, lbl in new_cmds]
        self.commands.extend(new_cmds)
        
        self.wp_scatter.set_offsets(self.waypoints)
        an = self.ax.annotate(f'WP{idx}', (tx, ty), textcoords="offset points",
                    xytext=(10, 10), fontsize=9, color='#e94560', fontweight='bold')
        self.wp_annotations.append(an)
        
        plan_x = [self.start_x] + [w[0] for w in self.waypoints]
        plan_y = [self.start_y] + [w[1] for w in self.waypoints]
        self.plan_line.set_data(plan_x, plan_y)
        self.finished = False

        # ★ Gửi toạ độ local xuống ESP32 qua WiFi
        self.wifi_send_target(local_x, local_y)

    def _snap_coordinate(self, x, y):
        """Bắt chuột vào tâm ô lưới (toạ độ chẵn trăm) nếu đủ gần."""
        nearest_x = round(x / GRID_SIZE) * GRID_SIZE
        nearest_y = round(y / GRID_SIZE) * GRID_SIZE
        
        # Snap threshold: 30 mm
        if abs(x - nearest_x) < 30.0 and abs(y - nearest_y) < 30.0:
            return nearest_x, nearest_y
        return x, y

    def on_press(self, event):
        """Xử lý sự kiện click trên đồ thị."""
        if event.inaxes != self.ax: return
        
        if event.button == 3:  # Chuột phải -> Kéo bản đồ
            self.panning = True
            self.pan_start_x = event.x
            self.pan_start_y = event.y
            self.pan_start_xlim = self.ax.get_xlim()
            self.pan_start_ylim = self.ax.get_ylim()
            return
            
        if event.button != 1:  # Chỉ chuột trái
            return

        tx, ty = event.xdata, event.ydata
        tx, ty = self._snap_coordinate(tx, ty)

        # === Chế độ đặt vật cản ===
        if self.placement_mode == 'obstacle':
            # Giữ Shift + Click để xóa vật cản
            if hasattr(event, 'key') and event.key == 'shift':
                if self._remove_obstacle_at(tx, ty):
                    self.user_actions.append(('remove_obstacle', tx, ty))
            else:
                self._place_obstacle(tx, ty)
                self.user_actions.append(('add_obstacle', tx, ty, self.obstacle_shape))
            return

        # === Chế độ A* Pathfinding ===
        if self.placement_mode == 'astar':
            self._on_astar_click(tx, ty)
            self.user_actions.append(('astar', tx, ty))
            return

        # === Chế độ đặt waypoint (mặc định) ===
        self._add_waypoint(tx, ty)
        self.user_actions.append(('waypoint', tx, ty))
        self.fig.canvas.draw_idle()

    def on_release(self, event):
        if event.button == 3:
            self.panning = False

    def on_hover(self, event):
        """Xử lý sự kiện di chuột để hiện toạ độ hoặc kéo bản đồ."""
        if getattr(self, 'panning', False):
            dx_display = event.x - self.pan_start_x
            dy_display = event.y - self.pan_start_y
            
            cur_xlim = self.pan_start_xlim
            cur_ylim = self.pan_start_ylim
            
            bbox = self.ax.get_window_extent()
            scale_x = (cur_xlim[1] - cur_xlim[0]) / bbox.width
            scale_y = (cur_ylim[1] - cur_ylim[0]) / bbox.height
            
            self.ax.set_xlim(cur_xlim[0] - dx_display * scale_x, cur_xlim[1] - dx_display * scale_x)
            self.ax.set_ylim(cur_ylim[0] - dy_display * scale_y, cur_ylim[1] - dy_display * scale_y)
            self.fig.canvas.draw_idle()
            return

        if getattr(self, 'hover_text', None) is None:
            return
            
        if event.inaxes != self.ax:
            if self.hover_text.get_visible():
                self.hover_text.set_visible(False)
                self.fig.canvas.draw_idle()
            return
            
        x, y = event.xdata, event.ydata
        x, y = self._snap_coordinate(x, y)
        self.hover_text.set_text(f"({x:.1f}, {y:.1f})")
        self.hover_text.set_position((x + 20, y + 20))  # Lệch lên 1 chút để không bị che
        self.hover_text.set_visible(True)
        self.fig.canvas.draw_idle()

    def on_scroll(self, event):
        """Xử lý sự kiện lăn chuột để zoom in/out."""
        if event.inaxes != self.ax:
            return
            
        # Tính tỷ lệ zoom
        base_scale = 1.2
        if event.button == 'up':
            # Cuộn lên -> Zoom In
            scale_factor = 1 / base_scale
        elif event.button == 'down':
            # Cuộn xuống -> Zoom Out
            scale_factor = base_scale
        else:
            scale_factor = 1
            
        # Lấy giới hạn hiện tại
        cur_xlim = self.ax.get_xlim()
        cur_ylim = self.ax.get_ylim()
        
        xdata = event.xdata
        ydata = event.ydata
        
        # Tính kích thước mới
        new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
        new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor
        
        # Tính tỷ lệ vị trí chuột
        relx = (cur_xlim[1] - xdata) / (cur_xlim[1] - cur_xlim[0])
        rely = (cur_ylim[1] - ydata) / (cur_ylim[1] - cur_ylim[0])
        
        # Đặt lại hệ trục
        self.ax.set_xlim([xdata - new_width * (1 - relx), xdata + new_width * relx])
        self.ax.set_ylim([ydata - new_height * (1 - rely), ydata + new_height * rely])
        
        self.fig.canvas.draw_idle()

    def run(self):
        """Chạy mô phỏng."""
        import itertools
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_hover)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.anim = animation.FuncAnimation(
            self.fig, self.animate, frames=itertools.count(),
            interval=20, blit=False, repeat=False, cache_frame_data=False
        )
        # Nới khung: chừa 25% lề trái (chứa text), và lùi lề phải về 84% để chừa không gian chứa nút điều khiển
        plt.tight_layout(rect=[0.25, 0, 0.84, 0.95])
        plt.show()


# =====================================================================
#  MAIN - CHẠY MÔ PHỎNG
# =====================================================================
def main():
    print("=" * 60)
    print("  AMR ODOMETRY SIMULATION - DIFFERENTIAL DRIVE")
    print("=" * 60)
    print(f"  Wheel Base (L)       : {WHEEL_BASE} mm")
    print(f"  Wheel Diameter (D)   : {WHEEL_DIAMETER} mm")
    print(f"  Wheel Circumference  : {WHEEL_CIRCUMFERENCE:.2f} mm")
    print(f"  Ticks per Revolution : {TICKS_PER_REV}")
    print(f"  Distance per Tick    : {DISTANCE_PER_TICK:.4f} mm")
    print("=" * 60)

    # Vị trí bắt đầu
    start_x = 0.0
    start_y = 0.0
    start_theta = 0.0  # Hướng ban đầu: 0 rad (trục X dương)

    # Danh sách waypoints (điểm đích)
    waypoints = []

    print(f"\n  Start Position: ({start_x}, {start_y}), theta = {math.degrees(start_theta):.1f} deg")
    print(f"  Waypoints:")
    for i, (wx, wy) in enumerate(waypoints):
        print(f"    WP{i}: ({wx}, {wy})")

    # Tạo robot
    robot = DifferentialDriveRobot(start_x, start_y, start_theta)

    # Sinh lệnh di chuyển
    print("\n  Generating motion commands...")
    commands, _, _, _ = generate_motion_commands(robot, waypoints)
    print(f"  Total commands: {len(commands)}")

    # Reset robot về vị trí ban đầu (vì generate_motion_commands không thay đổi robot thật)
    robot = DifferentialDriveRobot(start_x, start_y, start_theta)

    # Chạy mô phỏng
    print("\n  Starting simulation...")
    sim = OdometrySimulator(robot, waypoints, commands)
    sim.run()

    # In kết quả cuối cùng
    final_x, final_y, final_theta = robot.get_position()
    print(f"\n  Final Position: ({final_x:.2f}, {final_y:.2f})")
    print(f"  Final Heading : {math.degrees(final_theta):.2f} deg")
    print(f"  Total Ticks L : {robot.total_ticks_left:.0f}")
    print(f"  Total Ticks R : {robot.total_ticks_right:.0f}")
    print(f"  Movement Time : {sim.simulation_time:.2f} s")


if __name__ == "__main__":
    main()
