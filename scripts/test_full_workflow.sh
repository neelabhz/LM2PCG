#!/bin/bash
# 完整测试：VIS 0-7 + 实时选择监听

set -e

echo "=========================================="
echo "完整测试：只运行 VIS 0-7，然后测试选择"
echo "=========================================="
echo

# 1. 停止可能存在的服务器
echo "1. 清理环境..."
bash web/pointcloud-viewer/stop_dev.sh 2>/dev/null || true
pkill -f "api_server.py" 2>/dev/null || true
sleep 2
echo "   ✓ 环境已清理"
echo

# 2. 运行 VIS 0-7 (会自动启动 API 服务器)
echo "2. 运行 VIS 0-7 (auto_serve=True)..."
python3 <<'PYTHON'
from scripts.ai_api import Dispatcher
d = Dispatcher()
result = d.op_VIS(
    mode='room',
    name='test_realtime',
    room_codes=['0-7'],
    auto_serve=True,
    port=5173
)
print("✓ VIS 完成！")
print(f"   Viewer URL: {result['viewer_url']}")
print(f"   API Server: http://localhost:8090")
PYTHON
echo

# 3. 等待服务器完全启动
echo "3. 等待服务器完全启动..."
sleep 5

# 4. 检查服务器状态
echo "4. 检查服务器状态..."
if lsof -i :8090 > /dev/null 2>&1; then
    echo "   ✓ API服务器正在运行 (端口 8090)"
else
    echo "   ❌ API服务器未运行！"
    exit 1
fi

if lsof -i :5173 > /dev/null 2>&1; then
    echo "   ✓ Frontend服务器正在运行 (端口 5173)"
else
    echo "   ⚠️  Frontend服务器未运行（可能需要更长时间启动）"
fi
echo

# 5. 查看 API 服务器日志（最近几行）
echo "5. API 服务器日志（最近10行）:"
echo "----------------------------------------"
tail -10 /tmp/api_server.log 2>/dev/null || echo "   （日志文件暂时为空）"
echo "----------------------------------------"
echo

# 6. 发送测试选择
echo "6. 发送测试选择请求..."
python3 scripts/test_realtime_selection.py
echo

# 7. 查看 API 服务器的最新日志（应该包含实时打印）
echo "7. API 服务器最新日志（应该看到实时选择通知）:"
echo "========================================"
tail -30 /tmp/api_server.log 2>/dev/null || echo "   （日志文件为空）"
echo "========================================"
echo

echo "✅ 测试完成！"
echo
echo "📊 总结："
echo "  - VIS 命令自动启动了 API 和 Frontend 服务器"
echo "  - API 服务器: http://localhost:8090"
echo "  - Frontend: http://localhost:5173/?manifest=/manifests/test_realtime.json"
echo "  - 日志位置: /tmp/api_server.log"
echo
echo "🔍 实时监控日志："
echo "  tail -f /tmp/api_server.log"
echo
echo "🛑 停止服务器："
echo "  bash web/pointcloud-viewer/stop_dev.sh"
