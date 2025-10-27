#!/usr/bin/env python3
"""
测试脚本：模拟前端发送选择到后端
用于测试实时选择监听功能
"""
import json
import urllib.request
import urllib.error
from datetime import datetime

def test_selection(objects):
    """发送选择到后端"""
    url = 'http://localhost:8090/api/submit-selection'
    
    # 构建选择数据
    selection_data = []
    for obj in objects:
        selection_data.append({
            'itemCode': obj['code'],
            'displayName': obj['name'],
            'type': obj.get('type', 'room'),
            'sourceFile': obj.get('file', ''),
            'timestamp': datetime.now().isoformat()
        })
    
    print(f"\n📤 发送选择到后端 ({len(selection_data)} 个物体)...")
    print(f"URL: {url}")
    print(f"数据: {json.dumps(selection_data, indent=2, ensure_ascii=False)}\n")
    
    try:
        # 使用 urllib 发送 POST 请求
        data_bytes = json.dumps(selection_data).encode('utf-8')
        req = urllib.request.Request(
            url,
            data=data_bytes,
            headers={'Content-Type': 'application/json'},
            method='POST'
        )
        
        with urllib.request.urlopen(req) as response:
            response_data = json.loads(response.read().decode('utf-8'))
            print(f"✅ 成功! 状态码: {response.status}")
            print(f"响应: {json.dumps(response_data, indent=2, ensure_ascii=False)}")
            print(f"\n💡 提示: 查看运行 api_server.py 的终端，你应该会看到实时输出！")
            
    except urllib.error.HTTPError as e:
        print(f"❌ HTTP 错误! 状态码: {e.code}")
        print(f"响应: {e.read().decode('utf-8')}")
    except urllib.error.URLError:
        print("❌ 连接失败!")
        print("请确保 API 服务器正在运行:")
        print("  python3 scripts/api_server.py --port 8090")
    except Exception as e:
        print(f"❌ 错误: {e}")

if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--help':
        print("用法:")
        print("  python3 test_realtime_selection.py              # 测试单个选择")
        print("  python3 test_realtime_selection.py --multi      # 测试多个选择")
        print("  python3 test_realtime_selection.py --clear      # 测试清空选择")
        sys.exit(0)
    
    if len(sys.argv) > 1 and sys.argv[1] == '--multi':
        # 测试多个物体选择
        print("\n" + "="*60)
        print("测试场景 1: 选择多个房间")
        print("="*60)
        test_selection([
            {'code': '0-7', 'name': 'room_007', 'file': 'output/floor_0/room_007/mesh.ply'},
            {'code': '0-3', 'name': 'room_003', 'file': 'output/floor_0/room_003/mesh.ply'},
            {'code': '0-1', 'name': 'room_001', 'file': 'output/floor_0/room_001/mesh.ply'},
        ])
        
    elif len(sys.argv) > 1 and sys.argv[1] == '--clear':
        # 测试清空选择
        print("\n" + "="*60)
        print("测试场景 2: 清空选择")
        print("="*60)
        test_selection([])
        
    else:
        # 测试单个物体选择
        print("\n" + "="*60)
        print("测试场景: 选择单个房间")
        print("="*60)
        test_selection([
            {'code': '0-7', 'name': 'room_007', 'file': 'output/floor_0/room_007/mesh.ply'}
        ])
    
    print("\n" + "="*60)
    print("✨ 完成!")
    print("="*60 + "\n")
