#!/usr/bin/env python3
import json
import re

def parse_pbtxt_to_json(pbtxt_file, json_file):
    """
    将Apollo的calibration_table.pb.txt文件转换为JSON格式
    
    Args:
        pbtxt_file (str): 输入的pb.txt文件路径
        json_file (str): 输出的json文件路径
    """
    # 存储所有校准数据
    calibration_data = []
    
    # 读取pb.txt文件
    with open(pbtxt_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 使用正则表达式提取所有calibration块
    calibration_blocks = re.findall(
        r'calibration\s*\{([^}]+)\}', 
        content, 
        re.DOTALL
    )
    
    # 解析每个calibration块
    for block in calibration_blocks:
        # 提取speed, acceleration, command值
        speed_match = re.search(r'speed:\s*([-\d.]+)', block)
        acceleration_match = re.search(r'acceleration:\s*([-\d.]+)', block)
        command_match = re.search(r'command:\s*([-\d.]+)', block)
        
        if speed_match and acceleration_match and command_match:
            calibration_entry = {
                "speed": float(speed_match.group(1)),
                "acceleration": float(acceleration_match.group(1)),
                "command": float(command_match.group(1))
            }
            calibration_data.append(calibration_entry)
    
    # 构造JSON对象
    json_data = {
        "calibration": calibration_data
    }
    
    # 写入JSON文件
    with open(json_file, 'w', encoding='utf-8') as f:
        json.dump(json_data, f, indent=2)
    
    print(f"成功转换 {len(calibration_data)} 条记录")
    print(f"输出文件: {json_file}")

if __name__ == "__main__":
    # 输入和输出文件路径
    input_file = "calibration_table.pb.txt"
    output_file = "calibration_table.json"
    
    # 执行转换
    parse_pbtxt_to_json(input_file, output_file)