import pcbnew
import math
import heapq
import os

# === 3D A* 演算法核心 ===
def a_star_search_3d(grid, start, valid_goals, via_cost=30):
    depth = len(grid)
    height = len(grid[0])
    width = len(grid[0][0])
    
    queue = []
    heapq.heappush(queue, (0, start[0], start[1], start[2], [start]))
    visited = {start: 0}
    
    directions = [
        (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
    ]

    def get_h(x, y):
        return min(abs(x - gx) + abs(y - gy) for gx, gy, gz in valid_goals)
    
    while queue:
        _, cx, cy, cz, path = heapq.heappop(queue)
        
        if (cx, cy, cz) in valid_goals:
            return path
            
        current_g = visited[(cx, cy, cz)]
        
        # 1. 在同一層移動
        for dx, dy, move_cost in directions:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < width and 0 <= ny < height and grid[cz][ny][nx] == 0:
                new_g = current_g + move_cost
                if (nx, ny, cz) not in visited or new_g < visited[(nx, ny, cz)]:
                    visited[(nx, ny, cz)] = new_g
                    f = new_g + get_h(nx, ny)
                    heapq.heappush(queue, (f, nx, ny, cz, path + [(nx, ny, cz)]))
                    
        # 2. 換層 (打通孔 Via)
        # 規則：通孔會貫穿電路板，必須確保該 (cx, cy) 座標在「所有」層都無障礙物
        can_place_via = True
        for z in range(depth):
            if z != cz and grid[z][cy][cx] != 0:
                can_place_via = False
                break
                
        if can_place_via:
            # 允許跳轉到任何其他層次
            for nz in range(depth):
                if nz != cz:
                    new_g = current_g + via_cost
                    if (cx, cy, nz) not in visited or new_g < visited[(cx, cy, nz)]:
                        visited[(cx, cy, nz)] = new_g
                        f = new_g + get_h(cx, cy)
                        heapq.heappush(queue, (f, cx, cy, nz, path + [(cx, cy, nz)]))
                
    return None

# === 3D 障礙物設定函數 ===
def set_pad_obstacle_3d(grid, pad, min_x, min_y, grid_size, clearance, MM_TO_IU, layer_to_z, is_obstacle=True):
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
    
    # 動態判斷 Pad 存在於哪些 Z 軸層
    layers_to_mark = []
    for layer_id, z_idx in layer_to_z.items():
        if pad.IsOnLayer(layer_id):
            layers_to_mark.append(z_idx)
            
    # 通孔 Pad 通常會涵蓋所有銅層
    for z in layers_to_mark:
        for y in range(g_min_y, g_max_y + 1):
            for x in range(g_min_x, g_max_x + 1):
                grid[z][y][x] = val

def mark_path_as_obstacle_3d(grid, path):
    depth = len(grid)
    for i in range(len(path)):
        px, py, pz = path[i]
        
        # 判斷是否為過孔 (與前一個或後一個節點層次不同)
        is_via = False
        if i > 0 and path[i-1][2] != pz: is_via = True
        if i < len(path) - 1 and path[i+1][2] != pz: is_via = True

        layers_to_block = range(depth) if is_via else [pz]

        # 針對需要阻擋的層次設定九宮格障礙
        for z in layers_to_block:
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    nx, ny = px + dx, py + dy
                    if 0 <= ny < len(grid[0]) and 0 <= nx < len(grid[0][0]):
                        grid[z][ny][nx] = 1

# === 主程式開始 ===
script_dir = os.path.dirname(os.path.abspath(__file__))
input_file = os.path.join(script_dir, "data", "d4_cleared.kicad_pcb")
output_file = os.path.join(script_dir, "data", "routed_8direction_multilayer.kicad_pcb")
board = pcbnew.LoadBoard(input_file)
MM_TO_IU = 1_000_000

grid_size_mm = 0.1
clearance_mm = 0.12
track_width_iu = int(0.15 * MM_TO_IU)
via_dia_iu = int(0.6 * MM_TO_IU) 
via_drill_iu = int(0.3 * MM_TO_IU) 

# 1. 動態讀取電路板的銅層資訊
copper_layers = []
for i in range(pcbnew.PCB_LAYER_ID_COUNT):
    if board.IsLayerEnabled(i) and pcbnew.IsCopperLayer(i):
        copper_layers.append(i)

layer_count = len(copper_layers)
# 建立實體 Layer ID 與 Z 軸索引的雙向映射表
layer_to_z = {layer: z for z, layer in enumerate(copper_layers)}
z_to_layer = {z: layer for z, layer in enumerate(copper_layers)}

print(f"偵測到 {layer_count} 層銅層: {[board.GetLayerName(l) for l in copper_layers]}")

# 2. 收集並排序所有需要布線的網路
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

# 3. 初始化動態 3D 地圖
bbox = board.ComputeBoundingBox()
min_x_mm = bbox.GetX() / MM_TO_IU
min_y_mm = bbox.GetY() / MM_TO_IU
grid_width = math.ceil((bbox.GetWidth() / MM_TO_IU) / grid_size_mm) + 1
grid_height = math.ceil((bbox.GetHeight() / MM_TO_IU) / grid_size_mm) + 1

# 建立 grid_map[z][y][x]，深度為 layer_count
grid_map = [[[0 for _ in range(grid_width)] for _ in range(grid_height)] for _ in range(layer_count)]

for pad in board.GetPads():
    set_pad_obstacle_3d(grid_map, pad, min_x_mm, min_y_mm, grid_size_mm, clearance_mm, MM_TO_IU, layer_to_z, is_obstacle=True)

# 4. 依序進行多層 3D 布線
success_count = 0
total_pairs = 0

for task in net_tasks:
    net_name = task['name']
    net_code = task['code']
    pads = task['pads']
    
    print(f"\n[>] 準備 3D 布線: {net_name}")
    
    for pad in pads:
        set_pad_obstacle_3d(grid_map, pad, min_x_mm, min_y_mm, grid_size_mm, clearance_mm, MM_TO_IU, layer_to_z, is_obstacle=False)
    
    net_paths = []
    
    for i in range(len(pads) - 1):
        total_pairs += 1
        start_pad = pads[i]
        goal_pad = pads[i+1]
        
        sx = int((start_pad.GetPosition().x/MM_TO_IU - min_x_mm)/grid_size_mm)
        sy = int((start_pad.GetPosition().y/MM_TO_IU - min_y_mm)/grid_size_mm)
        gx = int((goal_pad.GetPosition().x/MM_TO_IU - min_x_mm)/grid_size_mm)
        gy = int((goal_pad.GetPosition().y/MM_TO_IU - min_y_mm)/grid_size_mm)
        
        # 決定起點層級 (尋找 Pad 所在的層次，優先取第一個找到的層)
        start_z = 0
        for layer_id, z_idx in layer_to_z.items():
            if start_pad.IsOnLayer(layer_id):
                start_z = z_idx
                break
        start_node = (sx, sy, start_z)
        
        # 決定終點合法層級 (Pad 存在的所有層皆為合法終點)
        valid_goals = []
        for layer_id, z_idx in layer_to_z.items():
            if goal_pad.IsOnLayer(layer_id):
                valid_goals.append((gx, gy, z_idx))
                      
        path = a_star_search_3d(grid_map, start_node, valid_goals)
        
        if path:
            net_paths.append(path)
            success_count += 1
            
            # 轉譯 3D 路徑為實體線段與過孔
            for j in range(len(path) - 1):
                pt1 = path[j] # (x, y, z)
                pt2 = path[j+1]
                
                x1_iu = int((min_x_mm + pt1[0] * grid_size_mm + (grid_size_mm/2)) * MM_TO_IU)
                y1_iu = int((min_y_mm + pt1[1] * grid_size_mm + (grid_size_mm/2)) * MM_TO_IU)
                x2_iu = int((min_x_mm + pt2[0] * grid_size_mm + (grid_size_mm/2)) * MM_TO_IU)
                y2_iu = int((min_y_mm + pt2[1] * grid_size_mm + (grid_size_mm/2)) * MM_TO_IU)
                
                # Z 軸發生改變 -> 過孔
                if pt1[2] != pt2[2]:
                    via = pcbnew.PCB_VIA(board)
                    via.SetPosition(pcbnew.VECTOR2I(x1_iu, y1_iu))
                    via.SetNetCode(net_code)
                    via.SetWidth(via_dia_iu)
                    via.SetDrill(via_drill_iu)
                    board.Add(via)
                else:
                    # 一般銅線
                    track = pcbnew.PCB_TRACK(board)
                    track.SetStart(pcbnew.VECTOR2I(x1_iu, y1_iu))
                    track.SetEnd(pcbnew.VECTOR2I(x2_iu, y2_iu))
                    track.SetWidth(track_width_iu)
                    
                    # 透過 z_to_layer 對應回實體的 KiCad 層 ID
                    actual_layer = z_to_layer[pt1[2]]
                    track.SetLayer(actual_layer)
                    track.SetNetCode(net_code)
                    board.Add(track)
        else:
            print(f"    [-] 局部失敗：無法連接")

    for path in net_paths:
        mark_path_as_obstacle_3d(grid_map, path)
        
    for pad in pads:
        set_pad_obstacle_3d(grid_map, pad, min_x_mm, min_y_mm, grid_size_mm, clearance_mm, MM_TO_IU, layer_to_z, is_obstacle=True)

pcbnew.SaveBoard(output_file, board)
print(f"\n[*] 3D 多層批次布線結束！成功率: {success_count}/{total_pairs}")