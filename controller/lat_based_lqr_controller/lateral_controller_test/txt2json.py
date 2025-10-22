import json
import re

def parse_value(value_str):
    """
    解析字符串值为适当的数据类型
    """
    value_str = value_str.strip()
    
    # 布尔值
    if value_str.lower() == 'true':
        return True
    elif value_str.lower() == 'false':
        return False
    # 数字（包括负数和科学计数法）
    elif re.match(r'^-?\d+(\.\d+)?([eE][+-]?\d+)?$', value_str):
        if '.' in value_str or 'e' in value_str.lower():
            return float(value_str)
        else:
            return int(value_str)
    # 字符串（包括模块名等）
    else:
        # 移除引号（如果有的话）
        if value_str.startswith('"') and value_str.endswith('"'):
            return value_str[1:-1]
        return value_str

def parse_block(content):
    """
    解析块状内容（如 header, path_point 等）
    """
    data = {}
    lines = content.strip().split('\n')
    
    for line in lines:
        line = line.strip()
        if not line:
            continue
            
        # 跳过空行和单独的括号
        if line in ['{', '}']:
            continue
            
        # 如果包含冒号，说明是键值对
        if ':' in line:
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip()
            data[key] = parse_value(value)
    
    return data

def parse_trajectory_point(point_content):
    """
    解析单个轨迹点
    """
    point_data = {}
    
    # 找到 path_point 的开始和结束位置
    path_point_start = point_content.find('path_point {')
    path_point_content = ""
    trajectory_content = point_content
    
    if path_point_start != -1:
        # 从 path_point { 后面开始计算
        start_pos = path_point_start + len('path_point {')
        brace_count = 1
        end_pos = start_pos
        
        # 计算匹配的大括号以找到正确的结束位置
        while end_pos < len(point_content) and brace_count > 0:
            if point_content[end_pos] == '{':
                brace_count += 1
            elif point_content[end_pos] == '}':
                brace_count -= 1
            end_pos += 1
        
        if brace_count == 0:
            # 提取path_point内容（不包括大括号）
            path_point_content = point_content[start_pos:end_pos-1]
            point_data['path_point'] = parse_block(path_point_content)
            # 提取不包括path_point部分的内容用于解析trajectory_point层级字段
            trajectory_content = point_content[:path_point_start] + point_content[end_pos:]
    
    # 解析 v, a, relative_time 等字段，从trajectory_point层级解析（排除path_point内部字段）
    # 查找 v 字段
    v_match = re.search(r'v:\s*([^\n]+)', trajectory_content)
    if v_match:
        point_data['v'] = parse_value(v_match.group(1))
    
    # 查找 a 字段
    a_match = re.search(r'a:\s*([^\n]+)', trajectory_content)
    if a_match:
        point_data['a'] = parse_value(a_match.group(1))
    
    # 查找 relative_time 字段
    relative_time_match = re.search(r'relative_time:\s*([^\n]+)', trajectory_content)
    if relative_time_match:
        point_data['relative_time'] = parse_value(relative_time_match.group(1))
    
    return point_data

def parse_planning_file(file_path):
    """
    严格解析 planning 文件并转换为 JSON 格式
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 初始化结果对象
    result = {}
    
    # 解析 header 部分
    header_match = re.search(r'header\s*{([^}]+)}', content, re.DOTALL)
    if header_match:
        header_content = header_match.group(1)
        result['header'] = parse_block(header_content)
    
    # 解析顶层字段
    # total_path_length
    total_path_length_match = re.search(r'total_path_length:\s*([^\n]+)', content)
    if total_path_length_match:
        result['total_path_length'] = parse_value(total_path_length_match.group(1))
    
    # total_path_time
    total_path_time_match = re.search(r'total_path_time:\s*([^\n]+)', content)
    if total_path_time_match:
        result['total_path_time'] = parse_value(total_path_time_match.group(1))
    
    # estop
    estop_match = re.search(r'estop\s*{([^}]+)}', content, re.DOTALL)
    if estop_match:
        estop_content = estop_match.group(1)
        result['estop'] = parse_block(estop_content)
    
    # gear
    gear_match = re.search(r'gear:\s*([^\n]+)', content)
    if gear_match:
        result['gear'] = parse_value(gear_match.group(1))
    
    # 解析所有 trajectory_point
    result['trajectory_point'] = []
    
    # 使用更准确的方法提取trajectory_point块
    # 查找trajectory_point {...}结构，考虑嵌套的大括号
    pos = 0
    while True:
        # 查找trajectory_point标记
        start_marker = content.find('trajectory_point {', pos)
        if start_marker == -1:
            break
        
        # 找到开始位置（大括号后）
        start_pos = start_marker + len('trajectory_point {')
        brace_count = 1
        end_pos = start_pos
        
        # 计算匹配的大括号以找到正确的结束位置
        while end_pos < len(content) and brace_count > 0:
            if content[end_pos] == '{':
                brace_count += 1
            elif content[end_pos] == '}':
                brace_count -= 1
            end_pos += 1
        
        if brace_count == 0:
            # 提取trajectory_point内容
            point_content = content[start_pos-1:end_pos]
            point_data = parse_trajectory_point(point_content)
            result['trajectory_point'].append(point_data)
            pos = end_pos
        else:
            break
    
    return result

def main():
    # 将1_planning.txt转换为JSON格式
    try:
        planning_data = parse_planning_file('1_planning.txt')
        
        # 保存为JSON文件
        with open('1_planning_fixed.json', 'w', encoding='utf-8') as f:
            json.dump(planning_data, f, ensure_ascii=False, indent=2)
        
        print("转换完成！已保存为 1_planning_fixed.json")
        print(f"共转换了 {len(planning_data.get('trajectory_point', []))} 个轨迹点")
        
        # 显示前几个轨迹点的信息以验证
        if 'trajectory_point' in planning_data and planning_data['trajectory_point']:
            print("\n前3个轨迹点信息:")
            for i, point in enumerate(planning_data['trajectory_point'][:3]):
                print(f"  点 {i+1}: v={point.get('v')}, a={point.get('a')}, relative_time={point.get('relative_time')}")
    
    except FileNotFoundError:
        print("错误：找不到文件 1_planning.txt，请确保文件存在且路径正确")
    except Exception as e:
        print(f"转换过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()