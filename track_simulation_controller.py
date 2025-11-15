import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import subprocess
import time
import math
import random # (新) 导入 random
from geopy.distance import geodesic
from geopy.point import Point
import queue  # 导入队列库，用于线程安全通信
import os     # 导入 os 库用于拼接路径
import json   # 导入 json 用于保存/加载设置

# 配置文件名
CONFIG_FILE = "track_sim_config.json"

# (新) 预设坐标
PRESETS = {
    "大连操场 (修正后)": {
        # (新) 根据 -5m N/S, -21m E/W 修正
        # (新) 同时将直道 (P2->P3, P4->P1) 缩短4米
        'p1_lat': '39.084957', 'p1_lon': '121.808666', # 经度 -4m
        'p2_lat': '39.085892', 'p2_lon': '121.808713',
        'p3_lat': '39.085893', 'p3_lon': '121.807886', # 经度 +4m
        'p4_lat': '39.084957', 'p4_lon': '121.807847',
        'offset_ns': '0.0', # 已修正，偏移归零
        'offset_ew': '0.0'
    },
    "大连操场 (原始偏移)": {
        # 这是你图片中的原始坐标 (已修复P3的纬度)
        'p1_lat': '39.092370', 'p1_lon': '121.820042',
        'p2_lat': '39.093305', 'p2_lon': '121.820043',
        'p3_lat': '39.093306', 'p3_lon': '121.819170', # (Bug 修复)
        'p4_lat': '39.092370', 'p4_lon': '121.819177',
        'offset_ns': '0.0', 
        'offset_ew': '0.0'
    }
}


# -----------------------------------------------------------------
# 核心地理计算函数
# -----------------------------------------------------------------

def calculate_initial_bearing(point_a: Point, point_b: Point) -> float:
    """
    计算从 point_a 到 point_b 的初始方位角（0-360度，0为正北）。
    """
    lat1 = math.radians(point_a.latitude)
    lon1 = math.radians(point_a.longitude)
    lat2 = math.radians(point_b.latitude)
    lon2 = math.radians(point_b.longitude)
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - \
        math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
    return bearing

def calculate_midpoint(point_a: Point, point_b: Point) -> Point:
    """
    计算两个GPS点之间的中点。
    """
    distance = geodesic(point_a, point_b).meters
    bearing = calculate_initial_bearing(point_a, point_b)
    # (已修复) 错误：point=a  ->  正确：point=point_a
    mid_point = geodesic(meters=distance / 2).destination(point=point_a, bearing=bearing)
    return mid_point

def interpolate_straight(p1: Point, p2: Point, step_meters: float) -> (list, float):
    """
    (新) 在两个点之间生成直线路径点。
    返回 (点列表[(Point, bearing)], 总长度)。
    """
    points = []
    total_distance = geodesic(p1, p2).meters
    if total_distance == 0:
        return [], 0.0
    bearing = calculate_initial_bearing(p1, p2)
    num_steps = int(total_distance / step_meters)
    for i in range(num_steps):
        dist = i * step_meters
        new_point = geodesic(meters=dist).destination(point=p1, bearing=bearing)
        points.append((new_point, bearing)) # (新) 存储 (点, 方向)
    points.append((p2, bearing)) # (新) 存储 (点, 方向)
    return points, total_distance

def interpolate_arc(p_start: Point, p_end: Point, step_meters: float, arc_degrees_total: float) -> (list, float):
    """
    (新) 重写了圆弧插值函数，支持自定义角度。
    返回 (点列表[(Point, bearing)], 总长度)。
    """
    points = []
    
    # 1. 计算弦长 (p_start 到 p_end 的直线距离)
    chord_len = geodesic(p_start, p_end).meters
    if chord_len == 0:
        return [], 0.0
    
    half_chord = chord_len / 2.0
    
    # 2. 用三角函数计算圆弧半径
    # sin(半角) = 对边(half_chord) / 斜边(radius)
    half_angle_rad = math.radians(arc_degrees_total / 2.0)
    
    # 避免除以零 (如果角度是0或360)
    if abs(math.sin(half_angle_rad)) < 1e-6:
        return [], 0.0 # 无法计算
        
    radius = half_chord / math.sin(half_angle_rad)
    
    # 3. 计算圆弧总长度
    arc_length = radius * math.radians(arc_degrees_total)
    num_steps = int(arc_length / step_meters)
    if num_steps == 0:
        return [], 0.0
    
    # 4. 计算圆心
    # 弦中点
    chord_midpoint = calculate_midpoint(p_start, p_end)
    
    # 弦心距 (圆心到弦中点的距离)
    # (新) 增加检查，避免因浮点数不精确导致 math.sqrt() 接收负数
    dist_to_center_sq = radius**2 - half_chord**2
    dist_to_center = math.sqrt(abs(dist_to_center_sq))
    
    # 5. 找到圆心
    # 弦的方位角
    bearing_chord = calculate_initial_bearing(p_start, p_end)
    # 圆心在弦中点的垂直方向上 (向外凸)
    bearing_to_center = (bearing_chord - 90 + 360) % 360
    true_center = geodesic(meters=dist_to_center).destination(point=chord_midpoint, bearing=bearing_to_center)
    
    # 6. 计算起始方位角 (从真圆心到 p_start)
    start_bearing = calculate_initial_bearing(true_center, p_start)
    
    # 7. 循环插值
    angle_step = arc_degrees_total / num_steps
    
    last_travel_bearing = 0
    for i in range(num_steps):
        # 顺时针旋转 (保持 "向外凸")
        current_bearing = (start_bearing - i * angle_step + 360) % 360
        new_point = geodesic(meters=radius).destination(point=true_center, bearing=current_bearing)
        
        # (新) 行进方向是圆弧切线方向 (半径方向 - 90度)
        travel_bearing = (current_bearing - 90 + 360) % 360
        points.append((new_point, travel_bearing))
        last_travel_bearing = travel_bearing
        
    points.append((p_end, last_travel_bearing)) # (新) 存储 (点, 方向)
    return points, arc_length


# -----------------------------------------------------------------
# 模拟器控制线程 (使用 dnconsole locate)
# -----------------------------------------------------------------

# 用于通知线程停止/暂停的事件
stop_simulation_event = threading.Event()
pause_event = threading.Event() # 用于暂停
skip_wait_event = threading.Event() # 用于立即跳过等待

def run_simulation_thread(status_queue, ld_folder_path, emulator_index, points_list, pace_info, step_m, random_offset_info):
    """
    在单独的线程中运行模拟。
    仅使用 dnconsole/ldconsole。
    """
    
    try:
        total_points = len(points_list)
        
        # 隐藏命令行窗口
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startupinfo.wShowWindow = subprocess.SW_HIDE
        
        # --- 自动启动模拟器 (可跳过) ---
        if not ld_folder_path:
             raise Exception("请提供雷电模拟器安装目录。")
             
        # 1. 查找控制台程序
        console_exe_path = os.path.join(ld_folder_path, "dnconsole.exe")
        if not os.path.exists(console_exe_path):
            console_exe_path = os.path.join(ld_folder_path, "ldconsole.exe")
            if not os.path.exists(console_exe_path):
                raise FileNotFoundError(f"在目录中未找到 dnconsole.exe 或 ldconsole.exe: {ld_folder_path}")
        
        initial_skip = skip_wait_event.is_set()
        
        if not initial_skip:
            # 2. 发送启动命令
            status_queue.put(("STATUS", f"正在启动模拟器 (索引 {emulator_index})...", "blue"))
            launch_command = [console_exe_path, "launch", "--index", str(emulator_index)]
            launch_result = subprocess.run(launch_command, check=True, startupinfo=startupinfo, capture_output=True, text=True, encoding='utf-8')
            
            if "fail" in launch_result.stdout.lower() or "error" in launch_result.stderr.lower():
                 raise Exception(f"启动模拟器失败:\nSTDOUT: {launch_result.stdout}\nSTDERR: {launch_result.stderr}")
                 
            # 3. (新) 可跳过的等待
            status_queue.put(("ENABLE_SKIP", True, None)) # 启用“跳过等待”按钮
            for i in range(23):
                if skip_wait_event.is_set():
                    status_queue.put(("STATUS", "已跳过等待...", "blue"))
                    break
                status_queue.put(("STATUS", f"等待模拟器启动 ({23-i}秒)...", "blue"))
                time.sleep(1)
            status_queue.put(("ENABLE_SKIP", False, None)) # 禁用“跳过等待”按钮
        else:
            status_queue.put(("STATUS", "已跳过启动等待...", "blue"))
            time.sleep(1) # 短暂等待以确保连接
        
        skip_wait_event.clear() # 重置事件
        
        status_queue.put(("STATUS", "连接成功，即将开始模拟...", "blue"))
        
        # (新) 平滑速度逻辑
        # pace_info: (const_delay) OR ("smooth", base_pace, variability, smoothness)
        
        # --- 循环发送 locate 命令 ---
        
        current_pace = 6.0 # 默认值
        target_pace = 6.0
        min_pace = 5.5
        max_pace = 6.5
        smooth_steps = 30 # 默认值
        is_smooth = False
        
        if isinstance(pace_info, tuple) and pace_info[0] == "smooth":
            is_smooth = True
            _, base_pace, variability, smoothness = pace_info
            current_pace = base_pace
            target_pace = base_pace
            min_pace = base_pace - variability
            max_pace = base_pace + variability
            # 计算达到目标配速需要的 *点* 数 (非秒)
            avg_delay = step_m / (1000.0 / (base_pace * 60.0))
            smooth_steps = max(1, int(smoothness / avg_delay)) # 至少1步
        else:
            const_delay = pace_info # 它是恒定的 delay_seconds

        loop_start_time = time.time()
        
        # (新) 循环解包 (点, 方向)
        for i, (point, travel_bearing) in enumerate(points_list):
            # 1. 检查是否需要停止
            if stop_simulation_event.is_set():
                status_queue.put(("STATUS", "模拟已手动停止。", "orange"))
                break
            
            # 2. 检查是否需要暂停
            pause_event.wait() # 如果 pause_event 被 clear(), 线程将在此阻塞
            
            # 3. (新) 动态计算延迟
            if is_smooth:
                # 每 N 步更新一次目标
                if i % smooth_steps == 0:
                    target_pace = random.uniform(min_pace, max_pace)
                
                # 逐渐接近目标 (简单的指数平滑)
                current_pace = current_pace * 0.98 + target_pace * 0.02
                
                current_speed_ms = 1000.0 / (current_pace * 60.0)
                delay = step_m / current_speed_ms
            else:
                delay = const_delay # 恒定延迟
            
            # 4. (新) 应用 *定向* 随机偏移
            final_point = point # 基础点
            if random_offset_info:
                chance_pct, max_range_m = random_offset_info
                # 只有 X% 的几率会偏移
                if random.random() < chance_pct:
                    # (新) 随机一个向左或向右的偏移距离 (-max .. +max)
                    offset_dist = random.uniform(-max_range_m, max_range_m) 
                    
                    # (新) 计算 "向右" 的方位角 (与行进方向垂直)
                    lateral_bearing = (travel_bearing + 90 + 360) % 360
                    
                    # 计算新点
                    final_point = geodesic(meters=offset_dist).destination(point=point, bearing=lateral_bearing)

            lon = final_point.longitude
            lat = final_point.latitude
            
            # 准备 LLI 参数: <Lng,Lat>
            lli_arg = f"{lon},{lat}"
            
            # 准备命令
            command = [
                console_exe_path,
                "locate",
                "--index", str(emulator_index),
                "--LLI", lli_arg
            ]

            # 执行 GPS 命令
            subprocess.run(command, startupinfo=startupinfo, capture_output=True, text=True, encoding='utf-8')
            
            # 5. 更新GUI
            if i % 10 == 0: 
                progress = f"正在模拟: {i+1} / {total_points} 个点"
                coords = f"当前坐标: {lat:.6f}, {lon:.6f}"
                status_queue.put(("UPDATE", progress, coords))
            
            # 6. 等待
            # (新) 精确计时，防止 time.sleep() 累积误差
            loop_end_time = loop_start_time + delay
            sleep_time = loop_end_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            loop_start_time = time.time() # 更新下一个循环的开始时间

        # ... (循环结束) ...
        if not stop_simulation_event.is_set():
            status_queue.put(("DONE", "模拟完成！", "green"))

    except FileNotFoundError as e:
        status_queue.put(("ERROR", f"错误: {e}", None))
    except subprocess.CalledProcessError as e:
        error_msg = f"命令执行失败 (Code {e.returncode}):\n\nSTDOUT (标准输出):\n{e.stdout}\n\nSTDERR (标准错误):\n{e.stderr}"
        status_queue.put(("ERROR", error_msg, None))
    except Exception as e:
        status_queue.put(("ERROR", f"线程发生未知错误: {e}", None))
    

# -----------------------------------------------------------------
# 主 GUI (使用 Tkinter)
# -----------------------------------------------------------------

class TrackSimulatorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("操场实时模拟控制器 (Tkinter版)")
        self.root.geometry("550x850") # (新) 窗口加高
        
        self.simulation_thread = None
        self.status_queue = queue.Queue() # 线程通信队列
        
        # --- 创建控件 ---
        main_frame = ttk.Frame(root, padding="10")
        main_frame.pack(fill="both", expand=True)
        
        # 1. 模拟器设置
        path_frame = ttk.Labelframe(main_frame, text="模拟器设置", padding="5")
        path_frame.pack(fill="x", expand=True)
        
        ttk.Label(path_frame, text="雷电模拟器安装目录:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.ld_folder_path = tk.StringVar()
        ttk.Entry(path_frame, textvariable=self.ld_folder_path, width=40).grid(row=0, column=0, sticky="w", padx=150, pady=5) 
        ttk.Button(path_frame, text="浏览...", command=self.browse_ld_folder).grid(row=0, column=1, padx=5)
        
        ttk.Label(path_frame, text="模拟器索引:").grid(row=2, column=0, sticky="w", padx=5, pady=5)
        self.emulator_index = tk.StringVar(value="0")
        ttk.Entry(path_frame, textvariable=self.emulator_index, width=10).grid(row=2, column=0, sticky="w", padx=100, pady=5)
        
        # (修改) 保持 Checkbox
        self.skip_wait_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(path_frame, text="跳过 23 秒启动等待", variable=self.skip_wait_var).grid(row=2, column=1, sticky="w", padx=5, pady=5)
        
        path_frame.grid_columnconfigure(0, weight=1)

        # 2. (新) 预设
        preset_frame = ttk.Labelframe(main_frame, text="坐标预设", padding="5")
        preset_frame.pack(fill="x", expand=True, pady=5)
        
        ttk.Label(preset_frame, text="选择预设:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.preset_var = tk.StringVar()
        preset_options = list(PRESETS.keys())
        self.preset_menu = ttk.Combobox(preset_frame, textvariable=self.preset_var, values=preset_options, state="readonly", width=30)
        self.preset_menu.grid(row=0, column=1, padx=5, pady=5)
        self.preset_menu.bind("<<ComboboxSelected>>", self.on_preset_select)
        
        # 3. 坐标输入
        self.coords_frame = ttk.Labelframe(main_frame, text="坐标 (WGS-84)", padding="5")
        self.coords_frame.pack(fill="x", expand=True, pady=5)
        
        self.coord_entries = {}
        coord_labels = {
            'p1': 'P1 (左下/SW):',
            'p2': 'P2 (右下/SE):',
            'p3': 'P3 (右上/NE):',
            'p4': 'P4 (左上/NW):'
        }
        
        row = 0
        for key, text in coord_labels.items():
            ttk.Label(self.coords_frame, text=text).grid(row=row, column=0, sticky="w", padx=5, pady=2)
            
            ttk.Label(self.coords_frame, text="纬:").grid(row=row, column=1, sticky="w", padx=5)
            lat_var = tk.StringVar()
            ttk.Entry(self.coords_frame, textvariable=lat_var, width=12).grid(row=row, column=2, sticky="w")
            self.coord_entries[f'{key}_lat'] = lat_var
            
            ttk.Label(self.coords_frame, text="经:").grid(row=row, column=3, sticky="w", padx=5)
            lon_var = tk.StringVar()
            ttk.Entry(self.coords_frame, textvariable=lon_var, width=12).grid(row=row, column=4, sticky="w")
            self.coord_entries[f'{key}_lon'] = lon_var
            row += 1
            
        self.coords_frame.grid_columnconfigure(2, weight=1)
        self.coords_frame.grid_columnconfigure(4, weight=1)

        # 4. 模拟参数
        params_frame = ttk.Labelframe(main_frame, text="模拟参数", padding="5")
        params_frame.pack(fill="x", expand=True)
        
        ttk.Label(params_frame, text="总距离 (米):").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.total_dist_m = tk.StringVar(value="10000")
        ttk.Entry(params_frame, textvariable=self.total_dist_m, width=10).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(params_frame, text="路径点间距 (米):").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        self.step_m = tk.StringVar(value="1.0")
        ttk.Entry(params_frame, textvariable=self.step_m, width=10).grid(row=1, column=1, padx=5, pady=5)
        
        # (新) 圆弧调节
        ttk.Label(params_frame, text="圆弧角度 (度):").grid(row=1, column=2, sticky="w", padx=5, pady=5)
        self.arc_degrees = tk.StringVar(value="170.0") # (新) 默认 170
        ttk.Entry(params_frame, textvariable=self.arc_degrees, width=10).grid(row=1, column=3, padx=5, pady=5)
        
        # (新) 平滑配速
        pace_smooth_frame = ttk.Labelframe(main_frame, text="平滑配速 (反作弊)", padding="5")
        pace_smooth_frame.pack(fill="x", expand=True, pady=5)
        
        self.random_pace_var = tk.BooleanVar(value=False)
        self.random_pace_check = ttk.Checkbutton(pace_smooth_frame, text="启用平滑配速", variable=self.random_pace_var, command=self.toggle_pace_entries)
        self.random_pace_check.grid(row=0, column=0, sticky="w", padx=10, pady=5)
        
        # (新) 配速输入
        ttk.Label(pace_smooth_frame, text="基础配速 (分钟/公里):").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        self.pace_minkm = tk.StringVar(value="6.0")
        self.pace_entry = ttk.Entry(pace_smooth_frame, textvariable=self.pace_minkm, width=10)
        self.pace_entry.grid(row=1, column=1, padx=5, pady=5)
        
        # (新) 随机配速范围
        ttk.Label(pace_smooth_frame, text="变异率 (± min/km):").grid(row=2, column=0, sticky="w", padx=5, pady=5)
        self.variability_var = tk.StringVar(value="0.2")
        self.variability_entry = ttk.Entry(pace_smooth_frame, textvariable=self.variability_var, width=10)
        self.variability_entry.grid(row=2, column=1, padx=5, pady=5)
        
        ttk.Label(pace_smooth_frame, text="变化平滑度 (秒):").grid(row=3, column=0, sticky="w", padx=5, pady=5)
        self.smoothness_var = tk.StringVar(value="30")
        self.smoothness_entry = ttk.Entry(pace_smooth_frame, textvariable=self.smoothness_var, width=10)
        self.smoothness_entry.grid(row=3, column=1, padx=5, pady=5)


        # 5. 路径微调
        offset_frame = ttk.Labelframe(main_frame, text="路径微调 (偏移)", padding="5")
        offset_frame.pack(fill="x", expand=True, pady=(0, 5))

        ttk.Label(offset_frame, text="北/南 偏移 (米):").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.offset_ns = tk.StringVar(value="0.0")
        self.offset_ns_entry = ttk.Entry(offset_frame, textvariable=self.offset_ns, width=10)
        self.offset_ns_entry.grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(offset_frame, text="(正数向北, 负数向南)").grid(row=0, column=2, sticky="w", padx=5, pady=5)

        ttk.Label(offset_frame, text="东/西 偏移 (米):").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        self.offset_ew = tk.StringVar(value="0.0")
        self.offset_ew_entry = ttk.Entry(offset_frame, textvariable=self.offset_ew, width=10)
        self.offset_ew_entry.grid(row=1, column=1, padx=5, pady=5)
        ttk.Label(offset_frame, text="(正数向东, 负数向西)").grid(row=1, column=2, sticky="w", padx=5, pady=5)

        # 6. (新) 随机偏移 (GPS 噪声)
        random_frame = ttk.Labelframe(main_frame, text="随机偏移 (GPS 噪声)", padding="5")
        random_frame.pack(fill="x", expand=True, pady=(0, 10))
        
        self.random_offset_var = tk.BooleanVar(value=False)
        self.random_offset_check = ttk.Checkbutton(random_frame, text="启用随机偏移", variable=self.random_offset_var, command=self.toggle_random_offset_entries)
        self.random_offset_check.grid(row=0, column=0, sticky="w", padx=5, pady=5)

        ttk.Label(random_frame, text="偏移几率 (%):").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        self.random_offset_chance = tk.StringVar(value="20")
        self.random_offset_chance_entry = ttk.Entry(random_frame, textvariable=self.random_offset_chance, width=10)
        self.random_offset_chance_entry.grid(row=1, column=1, padx=5, pady=5)
        
        # (新) 标签修改
        ttk.Label(random_frame, text="左/右最大偏移 (米):").grid(row=1, column=2, sticky="w", padx=5, pady=5)
        self.random_offset_range = tk.StringVar(value="1.5")
        self.random_offset_range_entry = ttk.Entry(random_frame, textvariable=self.random_offset_range, width=10)
        self.random_offset_range_entry.grid(row=1, column=3, padx=5, pady=5)

        # 7. 控制按钮
        button_frame = ttk.Frame(main_frame, padding="5")
        button_frame.pack(fill="x", expand=True)
        
        self.start_button = ttk.Button(button_frame, text="开始模拟", command=self.start_simulation)
        self.start_button.pack(side=tk.LEFT, fill="x", expand=True, padx=5)
        
        self.pause_button = ttk.Button(button_frame, text="暂停", command=self.toggle_pause, state=tk.DISABLED)
        self.pause_button.pack(side=tk.LEFT, fill="x", expand=True, padx=5)
        
        self.stop_button = ttk.Button(button_frame, text="停止模拟", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, fill="x", expand=True, padx=5)
        
        # (新) 跳过等待按钮
        self.skip_wait_button = ttk.Button(button_frame, text="跳过等待", command=self.skip_wait_now, state=tk.DISABLED)
        self.skip_wait_button.pack(side=tk.LEFT, fill="x", expand=True, padx=5)
        
        # 8. 状态显示
        status_frame = ttk.Labelframe(main_frame, text="状态", padding="5")
        status_frame.pack(fill="both", expand=True, pady=10)
        
        self.status_label = ttk.Label(status_frame, text="状态: 空闲", foreground="green")
        self.status_label.pack(anchor="w", padx=5, pady=2)
        
        self.coords_label = ttk.Label(status_frame, text="", foreground="darkblue")
        self.coords_label.pack(anchor="w", padx=5, pady=2)
        
        # 加载设置
        self.load_settings()
        
        # (新) 初始化预设
        if not self.preset_var.get():
            self.preset_menu.current(0) # 默认选择第一个
        self.load_preset(self.preset_var.get())
        
        # (新) 初始化随机配速框
        self.toggle_pace_entries()
        
        # (新) 初始化随机偏移框
        self.toggle_random_offset_entries()
        
        # 退出时停止线程
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 启动队列检查循环
        self.check_queue()

    # (新) 切换配速输入框
    def toggle_pace_entries(self):
        if self.random_pace_var.get():
            # 启用随机, 禁用固定
            self.pace_entry.config(state=tk.NORMAL) # 基础配速保持可用
            self.variability_entry.config(state=tk.NORMAL)
            self.smoothness_entry.config(state=tk.NORMAL)
        else:
            # 禁用随机, 启用固定
            self.pace_entry.config(state=tk.NORMAL)
            self.variability_entry.config(state=tk.DISABLED)
            self.smoothness_entry.config(state=tk.DISABLED)

    # (新) 切换随机偏移输入框
    def toggle_random_offset_entries(self):
        if self.random_offset_var.get():
            self.random_offset_chance_entry.config(state=tk.NORMAL)
            self.random_offset_range_entry.config(state=tk.NORMAL)
        else:
            self.random_offset_chance_entry.config(state=tk.DISABLED)
            self.random_offset_range_entry.config(state=tk.DISABLED)

    # (新) 选择预设时调用
    def on_preset_select(self, event):
        preset_name = self.preset_var.get()
        self.load_preset(preset_name)

    # (新) 加载预设数据到 GUI
    def load_preset(self, preset_name):
        data = PRESETS.get(preset_name)
        if not data:
            return
            
        self.coord_entries['p1_lat'].set(data['p1_lat'])
        self.coord_entries['p1_lon'].set(data['p1_lon'])
        self.coord_entries['p2_lat'].set(data['p2_lat'])
        self.coord_entries['p2_lon'].set(data['p2_lon'])
        self.coord_entries['p3_lat'].set(data['p3_lat'])
        self.coord_entries['p3_lon'].set(data['p3_lon'])
        self.coord_entries['p4_lat'].set(data['p4_lat'])
        self.coord_entries['p4_lon'].set(data['p4_lon'])
        
        # 只有在配置文件没加载的情况下，才设置偏移
        if not hasattr(self, 'settings_loaded') or not self.settings_loaded:
             self.offset_ns.set(data.get('offset_ns', '0.0'))
             self.offset_ew.set(data.get('offset_ew', '0.0'))

    def browse_ld_folder(self):
        directory = filedialog.askdirectory()
        if directory:
            self.ld_folder_path.set(directory)

    def start_simulation(self):
        try:
            # 1. 清除停止/暂停标记
            stop_simulation_event.clear()
            pause_event.set() # .set() 意味着 "未暂停"
            skip_wait_event.clear() # (新) 清除跳过事件
            
            # 2. 解析和验证输入
            ld_folder = self.ld_folder_path.get()
            
            if not ld_folder: 
                raise Exception("请提供雷电模拟器安装目录。")
            
            emu_index = int(self.emulator_index.get())
            p1 = Point(latitude=float(self.coord_entries['p1_lat'].get()), longitude=float(self.coord_entries['p1_lon'].get()))
            p2 = Point(latitude=float(self.coord_entries['p2_lat'].get()), longitude=float(self.coord_entries['p2_lon'].get()))
            p3 = Point(latitude=float(self.coord_entries['p3_lat'].get()), longitude=float(self.coord_entries['p3_lon'].get()))
            p4 = Point(latitude=float(self.coord_entries['p4_lat'].get()), longitude=float(self.coord_entries['p4_lon'].get()))
            
            total_dist_m = float(self.total_dist_m.get())
            step_m = float(self.step_m.get())
            offset_ns = float(self.offset_ns.get())
            offset_ew = float(self.offset_ew.get())
            skip_wait = self.skip_wait_var.get()
            
            # (新) 检查是否跳过
            if skip_wait:
                skip_wait_event.set()
            
            # (新) 动态计算 pace_info
            pace_info = None
            if self.random_pace_var.get():
                base_pace = float(self.pace_minkm.get()) # (新) 基础配速
                variability = float(self.variability_var.get()) # (新) 变异率
                smoothness = float(self.smoothness_var.get()) # (新) 平滑度
                if base_pace <= 0 or variability < 0 or smoothness <= 0:
                    raise Exception("平滑配速参数无效。")
                pace_info = ("smooth", base_pace, variability, smoothness)
            else:
                pace_minkm = float(self.pace_minkm.get())
                if pace_minkm <= 0:
                    raise Exception("配速必须 > 0")
                speed_ms = 1000.0 / (pace_minkm * 60.0)
                if speed_ms <= 0 or step_m <= 0:
                    raise Exception("速度或间距必须 > 0")
                pace_info = step_m / speed_ms # 恒定延迟
            
            # (新) 获取圆弧角度
            arc_degrees = float(self.arc_degrees.get())
            if arc_degrees <= 0 or arc_degrees >= 360:
                raise Exception("圆弧角度必须在 0 和 360 度之间。")
            
            # (新) 获取随机偏移
            random_offset_info = None
            if self.random_offset_var.get():
                chance = float(self.random_offset_chance.get())
                range_m = float(self.random_offset_range.get())
                if chance <= 0 or chance > 100 or range_m <= 0:
                    raise Exception("随机偏移参数无效")
                random_offset_info = (chance / 100.0, range_m) # 转换

            if total_dist_m <= 0:
                raise Exception("总距离必须 > 0")
            
            self.status_label.config(text=f"正在计算单圈路径...", foreground="blue")

            # 4. (新) **恢复为正确的几何**
            # (P1 -> P2 直道), (P2 -> P3 弯道), (P3 -> P4 直道), (P4 -> P1 弯道)
            
            s1_points, s1_len = interpolate_straight(p1, p2, step_m)
            a1_points, a1_len = interpolate_arc(p2, p3, step_m, arc_degrees)
            s2_points, s2_len = interpolate_straight(p3, p4, step_m)
            a2_points, a2_len = interpolate_arc(p4, p1, step_m, arc_degrees)

            single_lap_points = s1_points + a1_points + s2_points + a2_points
            
            if not single_lap_points: raise Exception("计算出的路径点为0，请检查坐标。")
            
            # 5. 计算完整路径
            total_points_needed = int(total_dist_m / step_m)
            full_path_points = []
            lap_point_count = len(single_lap_points)

            for i in range(total_points_needed):
                full_path_points.append(single_lap_points[i % lap_point_count])
            
            # 6. 应用路径偏移
            if offset_ns != 0.0 or offset_ew != 0.0:
                self.status_label.config(text=f"正在应用偏移: N/S {offset_ns}m, E/W {offset_ew}m...", foreground="blue")
                self.root.update_idletasks() 
                
                offset_path_points = []
                # (新) 处理 (point, bearing) 元组
                for point, bearing in full_path_points:
                    temp_point = geodesic(meters=offset_ns).destination(point, bearing=0)
                    final_point = geodesic(meters=offset_ew).destination(temp_point, bearing=90)
                    offset_path_points.append((final_point, bearing)) # (新) 保持 bearing
                
                full_path_points = offset_path_points
            
            self.status_label.config(text=f"路径计算完毕! 总点数: {len(full_path_points)}。即将开始连接...", foreground="blue")
            
            # 7. 启动线程
            self.start_button.config(state=tk.DISABLED)
            self.pause_button.config(text="暂停", state=tk.NORMAL) 
            self.stop_button.config(state=tk.NORMAL) 
            
            self.simulation_thread = threading.Thread(
                target=run_simulation_thread,
                args=(self.status_queue, ld_folder, emu_index, full_path_points, pace_info, step_m, random_offset_info), # (新) 传入
                daemon=True
            )
            self.simulation_thread.start()

        except Exception as e:
            messagebox.showerror("启动失败", f"启动模拟失败:\n{e}")
            self.status_label.config(text=f"失败: {e}", foreground="red")
            self.reset_gui_state() # (新) 启动失败时重置

    def toggle_pause(self):
        """ 切换暂停/继续状态 """
        if pause_event.is_set():
            pause_event.clear() 
            self.pause_button.config(text="继续")
            self.status_label.config(text="模拟已暂停。", foreground="orange")
        else:
            pause_event.set() 
            self.pause_button.config(text="暂停")
            self.status_label.config(text="模拟已恢复。", foreground="blue")

    def stop_simulation(self):
        """ 停止模拟 """
        self.status_label.config(text="正在发送停止信号...", foreground="orange")
        stop_simulation_event.set() 
        pause_event.set() # 确保线程没有卡在 pause_event.wait()
    
    # (新) 立即跳过等待
    def skip_wait_now(self):
        skip_wait_event.set()
        self.skip_wait_button.config(state=tk.DISABLED)
        
    def check_queue(self):
        """
        主 GUI 线程调用的函数，用于检查来自模拟线程的消息。
        """
        try:
            msg_type, msg, color_or_coords = self.status_queue.get_nowait()
            
            if msg_type == "ERROR":
                messagebox.showerror("线程错误", msg)
                self.status_label.config(text=f"错误: {msg}", foreground="red")
                self.reset_gui_state()
            elif msg_type == "UPDATE":
                progress = msg
                coords = color_or_coords
                self.status_label.config(text=progress, foreground="blue")
                self.coords_label.config(text=coords)
            elif msg_type == "STATUS":
                self.status_label.config(text=msg, foreground=color_or_coords)
                if msg == "模拟已手动停止。":
                    self.reset_gui_state()
            elif msg_type == "DONE":
                self.status_label.config(text=msg, foreground=color_or_coords)
                self.coords_label.config(text="")
                self.reset_gui_state()
            # (新) 处理跳过按钮
            elif msg_type == "ENABLE_SKIP":
                new_state = tk.NORMAL if color_or_coords else tk.DISABLED
                self.skip_wait_button.config(state=new_state)

        except queue.Empty:
            pass # 队列为空
        
        self.root.after(100, self.check_queue) # 100ms 轮询一次

    def reset_gui_state(self):
        """ 重置GUI按钮状态 """
        self.start_button.config(state=tk.NORMAL)
        self.pause_button.config(text="暂停", state=tk.DISABLED)
        self.stop_button.config(state=tk.DISABLED)
        self.skip_wait_button.config(state=tk.DISABLED) # (新)
        self.simulation_thread = None

    def load_settings(self):
        """ (新) 加载设置 """
        try:
            with open(CONFIG_FILE, 'r') as f:
                data = json.load(f)
                self.ld_folder_path.set(data.get("ld_folder_path", ""))
                self.offset_ns.set(data.get("offset_ns", "0.0"))
                self.offset_ew.set(data.get("offset_ew", "0.0"))
                self.preset_var.set(data.get("last_preset", "大连操场 (修正后)"))
                
                # (新) 加载随机设置
                self.random_offset_var.set(data.get("use_random_offset", False))
                self.random_offset_chance.set(data.get("random_offset_chance", "20"))
                self.random_offset_range.set(data.get("random_offset_range", "1.5"))
                
                # (新) 加载配速设置
                self.random_pace_var.set(data.get("use_random_pace", False))
                self.pace_minkm.set(data.get("base_pace", "6.0"))
                self.variability_var.set(data.get("pace_variability", "0.2"))
                self.smoothness_var.set(data.get("pace_smoothness", "30"))

                self.settings_loaded = True # 标记已加载
        except FileNotFoundError:
            self.preset_menu.current(0) # 配置文件不存在，默认选择第一个
            self.settings_loaded = False
        except Exception as e:
            print(f"加载配置文件失败: {e}")
            self.preset_menu.current(0)
            self.settings_loaded = False

    def save_settings(self):
        """ (新) 保存设置 """
        try:
            data = {
                "ld_folder_path": self.ld_folder_path.get(),
                "offset_ns": self.offset_ns.get(),
                "offset_ew": self.offset_ew.get(),
                "last_preset": self.preset_var.get(),
                
                "use_random_offset": self.random_offset_var.get(),
                "random_offset_chance": self.random_offset_chance.get(),
                "random_offset_range": self.random_offset_range.get(),
                
                # (新) 保存配速设置
                "use_random_pace": self.random_pace_var.get(),
                "base_pace": self.pace_minkm.get(),
                "pace_variability": self.variability_var.get(),
                "pace_smoothness": self.smoothness_var.get()
            }
            with open(CONFIG_FILE, 'w') as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            print(f"保存配置文件失败: {e}")

    def on_closing(self):
        """ (新) 在关闭时保存设置 """
        self.save_settings() 
        if self.simulation_thread and self.simulation_thread.is_alive():
            stop_simulation_event.set() 
            pause_event.set() 
        self.root.destroy()

# -----------------------------------------------------------------
# 启动应用
# -----------------------------------------------------------------

if __name__ == "__main__":
    root = tk.Tk()
    app = TrackSimulatorApp(root)
    root.mainloop()