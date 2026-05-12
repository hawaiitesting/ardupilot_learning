import os

def convert_waypoints_to_cpp(file_path):
    if not os.path.exists(file_path):
        print(f"找不到文件: {file_path}")
        return

    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    print("// ================= 自动生成的 C++ 航点代码 ================= ")
    
    wp_index = 0
    for line in lines:
        line = line.strip()
        if not line or line.startswith("QGC"):
            continue
            
        parts = line.split('\t')
        if len(parts) < 12:
            continue

        seq = int(parts[0])
        frame = int(parts[2])
        command = int(parts[3])

        if seq == 0: # 跳过 Home 点
            continue

        if command != 16:
            continue

        # 提取并转换数据
        lat = int(float(parts[8]) * 1e7)
        lng = int(float(parts[9]) * 1e7)
        alt = int(float(parts[10]) * 100)
        rel_alt = 1 if frame == 3 else 0

        # 输出为 C++ 挨个赋值的格式（绝对不会报错）
        print(f"num_waypoint[{wp_index}].alt = {alt};")
        print(f"num_waypoint[{wp_index}].lng = {lng};")
        print(f"num_waypoint[{wp_index}].lat = {lat};")
        print(f"num_waypoint[{wp_index}].relative_alt = {rel_alt};")
        print("") # 加个空行分隔，看起来更舒服
        
        wp_index += 1
        
    print(f"// 总共提取了 {wp_index} 个航点")
    print("// ========================================================= ")

if __name__ == "__main__":
    # 脚本会读取同目录下的 waypoint1.waypoints
    convert_waypoints_to_cpp('waypoint.waypoints')