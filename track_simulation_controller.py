import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import subprocess
import time
import math
from geopy.distance import geodesic
from geopy.point import Point
import queue  # 导入队列库，用于线程安全通信
import os     # 导入 os 库用于拼接路径

# -----------------------------------------------------------------
# 核心地理计算函数 (无变化)
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
    mid_point = geodesic(meters=distance / 2).destination(point=point_a, bearing=bearing)
    return mid_point

def interpolate_straight(p1: Point, p2: Point, step_meters: float) -> (list, float):
    """
    在两个点之间生成直线路径点。
    返回 (点列表[Point], 总长度)。
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
        points.append(new_point)
    points.append(p2)
    return points, total_distance

def interpolate_arc(p_start: Point, p_end: Point, step_meters: float) -> (list, float):
    """
    在两个点之间生成半圆弧路径点。
    返回 (点列表[Point], 总长度)。
    """
    points = []
    center = calculate_midpoint(p_start, p_end)
    radius = geodesic(center, p_start).meters
    if radius == 0:
        return [], 0.0
    arc_length = math.pi * radius
    num_steps = int(arc_length / step_meters)
    if num_steps == 0:
        return [], 0.0
    start_bearing = calculate_initial_bearing(center, p_start)
    angle_step = 180.0 / num_steps # 假设逆时针
    for i in range(num_steps):
        current_bearing = (start_bearing + i * angle_step + 360) % 360
        new_point = geodesic(meters=radius).destination(point=center, bearing=current_bearing)
        points.append(new_point)
    points.append(p_end)
    return points, arc_length

# -----------------------------------------------------------------
# 模拟器控制线程 (已简化为仅使用 dnconsole)
# -----------------------------------------------------------------

# 用于通知线程停止的事件
stop_simulation_event = threading.Event()

def run_simulation_thread(status_queue, ld_folder_path, emulator_index, points_list, delay_seconds):
    """
    在单独的线程中运行模拟。
    (已修改) 仅使用 dnconsole/ldconsole。
    """
    
    try:
        total_points = len(points_list)
        
        # 隐藏命令行窗口
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startupinfo.wShowWindow = subprocess.SW_HIDE
        
        # --- 自动启动模拟器 ---
        if not ld_folder_path:
             raise Exception("请提供雷电模拟器安装目录。")
             
        # 1. 查找控制台程序 (dnconsole.exe 或 ldconsole.exe)
        console_exe_path = os.path.join(ld_folder_path, "dnconsole.exe")
        if not os.path.exists(console_exe_path):
            console_exe_path = os.path.join(ld_folder_path, "ldconsole.exe")
            if not os.path.exists(console_exe_path):
                raise FileNotFoundError(f"在目录中未找到 dnconsole.exe 或 ldconsole.exe: {ld_folder_path}")
        
        # 2. 发送启动命令
        status_queue.put(("STATUS", f"正在启动模拟器 (索引 {emulator_index})...", "blue"))
        launch_command = [console_exe_path, "launch", "--index", str(emulator_index)]
        launch_result = subprocess.run(launch_command, check=True, startupinfo=startupinfo, capture_output=True, text=True, encoding='utf-8')
        
        if "fail" in launch_result.stdout.lower() or "error" in launch_result.stderr.lower():
             raise Exception(f"启动模拟器失败:\nSTDOUT: {launch_result.stdout}\nSTDERR: {launch_result.stderr}")
             
        # 3. 等待模拟器启动
        status_queue.put(("STATUS", "等待模拟器启动 (23秒)...", "blue"))
        time.sleep(23) 
        
        status_queue.put(("STATUS", "连接成功，即将开始模拟...", "blue"))
        
        # --- 循环发送 locate 命令 ---
        for i, point in enumerate(points_list):
            if stop_simulation_event.is_set():
                status_queue.put(("STATUS", "模拟已手动停止。", "orange"))
                break
                
            lon = point.longitude
            lat = point.latitude
            
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
            # 我们不检查 check=True，因为这个命令在成功时也可能返回非0值
            subprocess.run(command, startupinfo=startupinfo, capture_output=True, text=True, encoding='utf-8')
            
            # 3. 更新GUI
            if i % 10 == 0: 
                progress = f"正在模拟: {i+1} / {total_points} 个点"
                coords = f"当前坐标: {lat:.6f}, {lon:.6f}"
                status_queue.put(("UPDATE", progress, coords))
            
            # 4. 等待
            time.sleep(delay_seconds)

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
        self.root.geometry("550x550") # 窗口大小 (减小了)
        
        self.simulation_thread = None
        self.status_queue = queue.Queue() # 线程通信队列
        
        # 默认坐标 (大连)
        self.default_coords = {
            'p1_lat': '39.092370', 'p1_lon': '121.820042', # 左下 (SW)
            'p2_lat': '39.093305', 'p2_lon': '121.820043', # 右下 (SE)
            'p3_lat': '39.093306', 'p3_lon': '121.819170', # 右上 (NE)
            'p4_lat': '39.092370', 'p4_lon': '121.819177'  # 左上 (NW)
        }
        
        # --- 创建控件 ---
        main_frame = ttk.Frame(root, padding="10")
        main_frame.pack(fill="both", expand=True)
        
        # 1. 模拟器设置
        path_frame = ttk.Labelframe(main_frame, text="模拟器设置", padding="5")
        path_frame.pack(fill="x", expand=True)
        
        ttk.Label(path_frame, text="雷电模拟器安装目录:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.ld_folder_path = tk.StringVar()
        ttk.Entry(path_frame, textvariable=self.ld_folder_path, width=60).grid(row=0, column=0, sticky="w", padx=150, pady=5)
        ttk.Button(path_frame, text="浏览...", command=self.browse_ld_folder).grid(row=0, column=1, padx=5)
        
        ttk.Label(path_frame, text="模拟器索引:").grid(row=2, column=0, sticky="w", padx=5, pady=5)
        self.emulator_index = tk.StringVar(value="0")
        ttk.Entry(path_frame, textvariable=self.emulator_index, width=10).grid(row=2, column=0, sticky="w", padx=100, pady=5)
        
        # (移除了命令模式下拉菜单)
        
        path_frame.grid_columnconfigure(0, weight=1)

        # 2. 坐标输入
        coords_frame = ttk.Labelframe(main_frame, text="坐标 (逆时针 WGS-84)", padding="5")
        coords_frame.pack(fill="x", expand=True, pady=10)
        
        self.coord_entries = {}
        coord_labels = {
            'p1': 'P1 (左下/SW):',
            'p2': 'P2 (右下/SE):',
            'p3': 'P3 (右上/NE):',
            'p4': 'P4 (左上/NW):'
        }
        
        row = 0
        for key, text in coord_labels.items():
            ttk.Label(coords_frame, text=text).grid(row=row, column=0, sticky="w", padx=5, pady=2)
            
            ttk.Label(coords_frame, text="纬:").grid(row=row, column=1, sticky="w", padx=5)
            lat_var = tk.StringVar(value=self.default_coords[f'{key}_lat'])
            ttk.Entry(coords_frame, textvariable=lat_var, width=12).grid(row=row, column=2, sticky="w")
            self.coord_entries[f'{key}_lat'] = lat_var
            
            ttk.Label(coords_frame, text="经:").grid(row=row, column=3, sticky="w", padx=5)
            lon_var = tk.StringVar(value=self.default_coords[f'{key}_lon'])
            ttk.Entry(coords_frame, textvariable=lon_var, width=12).grid(row=row, column=4, sticky="w")
            self.coord_entries[f'{key}_lon'] = lon_var
            row += 1
            
        coords_frame.grid_columnconfigure(2, weight=1)
        coords_frame.grid_columnconfigure(4, weight=1)

        # 3. 模拟参数
        params_frame = ttk.Labelframe(main_frame, text="模拟参数", padding="5")
        params_frame.pack(fill="x", expand=True)
        
        ttk.Label(params_frame, text="总距离 (米):").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.total_dist_m = tk.StringVar(value="10000")
        ttk.Entry(params_frame, textvariable=self.total_dist_m, width=10).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(params_frame, text="速度 (米/秒):").grid(row=0, column=2, sticky="w", padx=5, pady=5)
        self.speed_ms = tk.StringVar(value="3.0")
        ttk.Entry(params_frame, textvariable=self.speed_ms, width=10).grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Label(params_frame, text="路径点间距 (米):").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        self.step_m = tk.StringVar(value="1.0")
        ttk.Entry(params_frame, textvariable=self.step_m, width=10).grid(row=1, column=1, padx=5, pady=5)

        # 4. 控制按钮
        button_frame = ttk.Frame(main_frame, padding="5")
        button_frame.pack(fill="x", expand=True)
        
        self.start_button = ttk.Button(button_frame, text="开始模拟", command=self.start_simulation)
        self.start_button.pack(side=tk.LEFT, fill="x", expand=True, padx=5)
        
        self.stop_button = ttk.Button(button_frame, text="停止模拟", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, fill="x", expand=True, padx=5)
        
        # 5. 状态显示
        status_frame = ttk.Labelframe(main_frame, text="状态", padding="5")
        status_frame.pack(fill="both", expand=True, pady=10)
        
        self.status_label = ttk.Label(status_frame, text="状态: 空闲", foreground="green")
        self.status_label.pack(anchor="w", padx=5, pady=2)
        
        self.coords_label = ttk.Label(status_frame, text="", foreground="darkblue")
        self.coords_label.pack(anchor="w", padx=5, pady=2)
        
        # 退出时停止线程
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 启动队列检查循环
        self.check_queue()

    def browse_ld_folder(self):
        directory = filedialog.askdirectory()
        if directory:
            self.ld_folder_path.set(directory)

    def start_simulation(self):
        try:
            # 1. 清除停止标记
            stop_simulation_event.clear()
            
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
            speed_ms = float(self.speed_ms.get())
            step_m = float(self.step_m.get())

            if speed_ms <= 0 or step_m <= 0 or total_dist_m <= 0:
                raise Exception("距离、速度和间距必须 > 0")
            
            # 3. 计算延迟
            delay_seconds = step_m / speed_ms
            
            self.status_label.config(text=f"正在计算单圈路径... 延迟: {delay_seconds:.3f} 秒/点", foreground="blue")

            # 4. 计算单圈路径点
            s1_points, s1_len = interpolate_straight(p1, p2, step_m)
            a1_points, a1_len = interpolate_arc(p2, p3, step_m)
            s2_points, s2_len = interpolate_straight(p3, p4, step_m)
            a2_points, a2_len = interpolate_arc(p4, p1, step_m)

            single_lap_points = s1_points + a1_points + s2_points + a2_points
            single_lap_length = s1_len + a1_len + s2_len + a2_len
            
            if not single_lap_points: raise Exception("计算出的路径点为0，请检查坐标。")
            
            # 5. 计算完整路径
            total_points_needed = int(total_dist_m / step_m)
            full_path_points = []
            lap_point_count = len(single_lap_points)

            for i in range(total_points_needed):
                full_path_points.append(single_lap_points[i % lap_point_count])
            
            self.status_label.config(text=f"路径计算完毕! 总点数: {len(full_path_points)}。即将开始连接...", foreground="blue")
            
            # 6. 启动线程
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.DISABLED)
            
            self.simulation_thread = threading.Thread(
                target=run_simulation_thread,
                args=(self.status_queue, ld_folder, emu_index, full_path_points, delay_seconds), # (移除了 command_mode)
                daemon=True
            )
            self.simulation_thread.start()

        except Exception as e:
            messagebox.showerror("启动失败", f"启动模拟失败:\n{e}")
            self.status_label.config(text=f"失败: {e}", foreground="red")

    def stop_simulation(self):
        # 发送停止信号给线程
        self.status_label.config(text="正在发送停止信号...", foreground="orange")
        stop_simulation_event.set()
        self.stop_button.config(state=tk.DISABLED)

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
            elif msg_type == "DONE":
                self.status_label.config(text=msg, foreground=color_or_coords)
                self.coords_label.config(text="")
                self.reset_gui_state()

        except queue.Empty:
            pass # 队列为空
        
        self.root.after(100, self.check_queue)

    def reset_gui_state(self):
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.simulation_thread = None

    def on_closing(self):
        if self.simulation_thread and self.simulation_thread.is_alive():
            stop_simulation_event.set() # 通知线程停止
        self.root.destroy()

# -----------------------------------------------------------------
# 启动应用
# -----------------------------------------------------------------

if __name__ == "__main__":
    root = tk.Tk()
    app = TrackSimulatorApp(root)
    root.mainloop()