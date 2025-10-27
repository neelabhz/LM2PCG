#!/usr/bin/env python3
"""快速测试：检测选择并执行操作"""
import sys
import json
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from ai_agent_helper import AIAgentHelper

helper = AIAgentHelper()

# 读取选择
selection_file = Path("/tmp/viewer_selection.json")
if not selection_file.exists():
    print("❌ 没有找到选择文件")
    sys.exit(1)

print("="*60)
print("检测到用户选择！")
print("="*60)

with open(selection_file) as f:
    data = json.load(f)

codes = [item['itemCode'] for item in data]
print(f"\n用户选择的物体: {codes}")

# 关闭 viewer
print("\n关闭 viewer...")
helper._close_viewer()
print("✓ Viewer 已关闭")

# 执行操作
print("\n对选择的物体执行 CLR 操作...")
results = helper.execute_operations_on_selection(codes, ['CLR'])

print("\n" + "="*60)
print("最终结果:")
print("="*60)
print(json.dumps(results, indent=2, ensure_ascii=False))
