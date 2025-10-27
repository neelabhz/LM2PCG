#!/usr/bin/env python3
"""
测试 AI Agent 完整工作流程
"""
import sys
from pathlib import Path

# Add scripts directory to path
sys.path.insert(0, str(Path(__file__).parent))

from ai_agent_helper import AIAgentHelper

print("="*70)
print("🤖 AI Agent 工作流程测试")
print("="*70)
print()
print("这个测试模拟 AI Agent 的完整工作流程：")
print("1. 运行 VIS 命令打开 viewer")
print("2. 等待你在浏览器中选择物体并点击 Confirm")
print("3. 自动关闭 viewer")
print("4. 对你选择的物体执行 CLR 操作")
print()
print("="*70)
print()

# 创建 AI Agent Helper
helper = AIAgentHelper()

# 步骤 1-3: 可视化并等待选择
selected_codes = helper.visualize_and_wait_for_selection(
    mode='room',
    room_codes=['0-7'],
    timeout=300,  # 5 分钟超时
    auto_close=True
)

# 步骤 4: 如果用户选择了物体，执行操作
if selected_codes:
    print("\n" + "="*70)
    print("🎨 现在对选择的物体执行颜色分析 (CLR)...")
    print("="*70)
    
    results = helper.execute_operations_on_selection(
        selected_codes=selected_codes,
        operations=['CLR']  # 可以改成 ['CLR', 'VOL', 'RCN']
    )
    
    print("\n" + "="*70)
    print("📊 最终结果:")
    print("="*70)
    import json
    print(json.dumps(results, indent=2, ensure_ascii=False))
    print()
else:
    print("\n⚠️  没有选择任何物体，跳过后续操作")

print("\n✅ AI Agent 工作流程测试完成！")
