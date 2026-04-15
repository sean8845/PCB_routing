import pcbnew
import math
import heapq
import os

# === 3D A* 演算法核心 ===
def a_star_search_3d(grid, start, valid_goals, via_cost=30):
    # grid 維度: [z][y][x]
    depth = len(grid)
    height = len(grid[0])
    width = len(grid[0][0])
    
    queue = []
    # 狀態: (f_cost, x, y, z, path)
    heapq.heappush(queue, (0, start[0], start[1], start[2], [start]))
    visited = {start: 0}
    
    # 同層移動方向
    directions = [
    (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),     # 上、下、左、右 (90度系列)
    (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414) # 斜向 (45度系列)
    ]

    
    # 啟發函數：取到所有有效目標點的最小曼哈頓距離
    def get_h(x, y):
        return min(abs(x - gx) + abs(y - gy) for gx, gy, gz in valid_goals)
    
    while queue:
        _, cx, cy, cz, path = heapq.heappop(queue)
        
        # 如果抵達任何一個合法的目標點 (考慮層級)
        if (cx, cy, cz) in valid_goals:
            return path
            
        current_g = visited[(cx, cy, cz)]
        
        # 1. 在同一層移動
        for dx, dy, move_cost in directions:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < width and 0 <= ny < height and grid[cz][ny][nx] == 0:
                new_g = current_g + move_cost # 使用對應的移動成本，斜向較貴
                if (nx, ny, cz) not in visited or new_g < visited[(nx, ny, cz)]:
                    visited[(nx, ny, cz)] = new_g
                    f = new_g + get_h(nx, ny)
                    heapq.heappush(queue, (f, nx, ny, cz, path + [(nx, ny, cz)]))
                    
        # 2. 換層 (打過孔 Via)
        nz = 1 - cz # 如果是 0 就變 1，如果是 1 就變 0
        # 換層的前提是：目標層的這個位置必須也是空的
        if grid[nz][cy][cx] == 0:
            new_g = current_g + via_cost
            if (cx, cy, nz) not in visited or new_g < visited[(cx, cy, nz)]:
                visited[(cx, cy, nz)] = new_g
                f = new_g + get_h(cx, cy)
                heapq.heappush(queue, (f, cx, cy, nz, path + [(cx, cy, nz)]))
                
    return None

# === 3D 障礙物設定函數 ===
def set_pad_obstacle_3d(grid, pad, min_x, min_y, grid_size, clearance, MM_TO_IU, is_obstacle=True):
    val = 1 if is_obstacle else 0
    eff_clearance = clearance 
    
    pad_bbox = pad.GetBoundingBox()
    p_min_x = (pad_bbox.GetX() / MM_TO_IU) - eff_clearance
    p_min_y = (pad_bbox.GetY() / MM_TO_IU) - eff_clearance
    p_max_x = (pad_bbox.GetRight() / MM_TO_IU) + eff_clearance
    p_max_y = (pad_bbox.GetBottom() / MM_TO_IU) + eff_clearance
    
    g_min_x = max(0, int((p_min_x - min_x) / grid_size))
    g_max_x = min(len(grid[0][0]) - 1, int((p_max_x - min_x) / grid_size))
    g_min_y = max(0, int((p_min_y - min_y) / grid_size))
    g_max_y = min(len(grid[0]) - 1, int((p_max_y - min_y) / grid_size))
    
    # 判斷這個 Pad 存在於哪些層
    layers = []
    if pad.IsOnLayer(pcbnew.F_Cu): layers.append(0)
    if pad.IsOnLayer(pcbnew.B_Cu): layers.append(1)
    if not layers: layers = [0, 1] # 預防萬一，當作兩層都有
    
    for z in layers:
        for y in range(g_min_y, g_max_y + 1):
            for x in range(g_min_x, g_max_x + 1):
                grid[z][y][x] = val

def mark_path_as_obstacle_3d(grid, path):
    for px, py, pz in path:
        # 為了安全間距，把路徑周圍九宮格設為障礙物
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                nx, ny = px + dx, py + dy
                if 0 <= ny < len(grid[0]) and 0 <= nx < len(grid[0][0]):
                    grid[pz][ny][nx] = 1

# === 主程式開始 ===
script_dir = os.path.dirname(os.path.abspath(__file__))
input_file = os.path.join(script_dir, "data", "cleared.kicad_pcb")
output_file = os.path.join(script_dir, "data", "routed_8direction.kicad_pcb")
board = pcbnew.LoadBoard(input_file)
MM_TO_IU = 1_000_000

grid_size_mm = 0.1
clearance_mm = 0.12
track_width_iu = int(0.15 * MM_TO_IU)
via_dia_iu = int(0.6 * MM_TO_IU)  # 過孔外徑
via_drill_iu = int(0.3 * MM_TO_IU) # 過孔鑽孔直徑

# 1. 收集並排序所有需要布線的網路
net_tasks = []
for name, net in board.GetNetsByName().items():
    if name and net.GetNetCode() > 0:
        pads = [p for p in board.GetPads() if p.GetNetCode() == net.GetNetCode()]
        if len(pads) >= 2:
            total_dist = sum(math.hypot(pads[i].GetPosition().x - pads[i+1].GetPosition().x, 
                                        pads[i].GetPosition().y - pads[i+1].GetPosition().y) 
                             for i in range(len(pads)-1))
            net_tasks.append({'name': name, 'code': net.GetNetCode(), 'pads': pads, 'dist': total_dist})

net_tasks.sort(key=lambda x: x['dist'])

# 2. 初始化 3D 雙層地圖 (z=0 為頂層, z=1 為底層)
bbox = board.ComputeBoundingBox()
min_x_mm = bbox.GetX() / MM_TO_IU
min_y_mm = bbox.GetY() / MM_TO_IU
grid_width = math.ceil((bbox.GetWidth() / MM_TO_IU) / grid_size_mm) + 1
grid_height = math.ceil((bbox.GetHeight() / MM_TO_IU) / grid_size_mm) + 1

# 建立 grid_map[z][y][x]
grid_map = [[[0 for _ in range(grid_width)] for _ in range(grid_height)] for _ in range(2)]

for pad in board.GetPads():
    set_pad_obstacle_3d(grid_map, pad, min_x_mm, min_y_mm, grid_size_mm, clearance_mm, MM_TO_IU, is_obstacle=True)

# 3. 依序進行 3D 布線
success_count = 0
total_pairs = 0

for task in net_tasks:
    net_name = task['name']
    net_code = task['code']
    pads = task['pads']
    
    print(f"\n[>] 準備 3D 布線: {net_name}")
    
    for pad in pads:
        set_pad_obstacle_3d(grid_map, pad, min_x_mm, min_y_mm, grid_size_mm, clearance_mm, MM_TO_IU, is_obstacle=False)
    
    net_paths = []
    
    for i in range(len(pads) - 1):
        total_pairs += 1
        start_pad = pads[i]
        goal_pad = pads[i+1]
        
        sx = int((start_pad.GetPosition().x/MM_TO_IU - min_x_mm)/grid_size_mm)
        sy = int((start_pad.GetPosition().y/MM_TO_IU - min_y_mm)/grid_size_mm)
        gx = int((goal_pad.GetPosition().x/MM_TO_IU - min_x_mm)/grid_size_mm)
        gy = int((goal_pad.GetPosition().y/MM_TO_IU - min_y_mm)/grid_size_mm)
        
        # 決定起點層級 (預設在頂層，除非它是純底層 SMD)
        sz = 1 if (start_pad.IsOnLayer(pcbnew.B_Cu) and not start_pad.IsOnLayer(pcbnew.F_Cu)) else 0
        start_node = (sx, sy, sz)
        
        # 決定終點合法層級 (如果是穿透孔，不管頂層還是底層連到都算成功)
        valid_goals = []
        if goal_pad.IsOnLayer(pcbnew.F_Cu): valid_goals.append((gx, gy, 0))
        if goal_pad.IsOnLayer(pcbnew.B_Cu): valid_goals.append((gx, gy, 1))
        if not valid_goals: valid_goals = [(gx, gy, 0), (gx, gy, 1)]
                      
        path = a_star_search_3d(grid_map, start_node, valid_goals)
        
        if path:
            net_paths.append(path)
            success_count += 1
            
            # 【關鍵】：轉譯 3D 路徑為實體線段與過孔
            for j in range(len(path) - 1):
                pt1 = path[j] # (x, y, z)
                pt2 = path[j+1]
                
                x1_iu = int((min_x_mm + pt1[0] * grid_size_mm + (grid_size_mm/2)) * MM_TO_IU)
                y1_iu = int((min_y_mm + pt1[1] * grid_size_mm + (grid_size_mm/2)) * MM_TO_IU)
                x2_iu = int((min_x_mm + pt2[0] * grid_size_mm + (grid_size_mm/2)) * MM_TO_IU)
                y2_iu = int((min_y_mm + pt2[1] * grid_size_mm + (grid_size_mm/2)) * MM_TO_IU)
                
                # 如果 Z 軸發生改變，代表演算法決定在這裡打一個過孔
                if pt1[2] != pt2[2]:
                    via = pcbnew.PCB_VIA(board)
                    via.SetPosition(pcbnew.VECTOR2I(x1_iu, y1_iu))
                    via.SetNetCode(net_code)
                    via.SetWidth(via_dia_iu)
                    via.SetDrill(via_drill_iu)
                    board.Add(via)
                else:
                    # Z 軸沒變，畫一般的銅線
                    track = pcbnew.PCB_TRACK(board)
                    track.SetStart(pcbnew.VECTOR2I(x1_iu, y1_iu))
                    track.SetEnd(pcbnew.VECTOR2I(x2_iu, y2_iu))
                    track.SetWidth(track_width_iu)
                    # 根據 Z 決定圖層：0 是 F_Cu(紅), 1 是 B_Cu(藍)
                    track.SetLayer(pcbnew.F_Cu if pt1[2] == 0 else pcbnew.B_Cu)
                    track.SetNetCode(net_code)
                    board.Add(track)
        else:
            print(f"    [-] 局部失敗：無法連接")

    for path in net_paths:
        mark_path_as_obstacle_3d(grid_map, path)
        
    for pad in pads:
        set_pad_obstacle_3d(grid_map, pad, min_x_mm, min_y_mm, grid_size_mm, clearance_mm, MM_TO_IU, is_obstacle=True)

pcbnew.SaveBoard(output_file, board)
print(f"\n[*] 3D 雙層批次布線結束！成功率: {success_count}/{total_pairs}")