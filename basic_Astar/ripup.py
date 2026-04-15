import pcbnew
import os
# 1. 載入我們之前畫好線的檔案
# 請確保這個檔案裡面有實體的銅線，例如我們之前生成的 output_routed.kicad_pcb
script_dir = os.path.dirname(os.path.abspath(__file__))
input_file = os.path.join(script_dir, "data", "d1.unrouted.kicad_pcb")
output_file = os.path.join(script_dir, "data", "cleared.kicad_pcb")
board = pcbnew.LoadBoard(input_file)

# 2. 收集所有要刪除的線段
# 為了避免在迴圈中直接刪除物件導致指標錯誤，我們先把它們存在一個清單裡
tracks_to_remove = []

# 遍歷板子上的所有線段 (Tracks)
for track in board.GetTracks():
    # 判斷條件：只要是有綁定真實網路的線段 (NetCode > 0)，就列入拆除名單
    # 未來你可以修改這裡，讓它只拆除特定的網路 (例如 if track.GetNetCode() == 2:)
    if track.GetNetCode() > 0:
        tracks_to_remove.append(track)

print(f"總共找到 {len(tracks_to_remove)} 段既有布線準備拆除...")

# 3. 執行拆除作業 (Rip-up)
for track in tracks_to_remove:
    board.Remove(track)

print("拆線完成！")

# 4. 存檔
pcbnew.SaveBoard(output_file, board)
print(f"已將清除布線後的電路板存檔至: {output_file}")