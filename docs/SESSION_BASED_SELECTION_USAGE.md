# Session-Based User Selection - 使用示例

## 概述

这个机制允许 AI agent 在需要时暂停执行，等待用户在 web viewer 中选择物体，然后根据用户的选择继续执行。

## Phase 1 基础架构 - 已完成 ✅

### 修改的文件

1. **`ai_api_wrapper.py`**
   - 扩展了 `VisOutput` schema 添加 selection 相关字段
   - `requires_selection`: 是否需要用户选择
   - `session_id`: 选择会话标识符
   - `selection_file`: 选择结果文件路径

2. **`mutli_room_agent2.py`**
   - 新增 `_wait_for_user_selection()` 方法
   - 轮询 `/tmp/viewer_selection_{session_id}.json` 文件
   - 支持超时和错误处理

3. **`scripts/api_server.py`**
   - 修改 `_handle_submit_selection()` 支持两种格式：
     - Legacy: `[...selections...]` (数组)
     - New: `{"session_id": "...", "selections": [...]}`
   - Session-based 选择保存到 `/tmp/viewer_selection_{session_id}.json`

4. **`scripts/test_realtime_selection.py`**
   - 添加 `--session` 参数支持 session ID
   - 更新 `test_selection()` 函数接受 session_id

5. **`test_selection_wait.py`** (新文件)
   - 测试脚本，验证等待机制
   - 支持 mock 模式和实际等待模式

## 使用方法

### 测试 1: Mock 模式（自动测试）

这个模式会自动创建 mock selection 文件，无需手动操作。

```bash
# 确保数据库存在
python3 room_database.py

# 运行 mock 测试
python3 test_selection_wait.py --mock
```

**预期输出：**
```
==================================================================
🧪 测试 Mock Selection (自动创建选择文件)
==================================================================

🚀 开始等待用户选择...
   Session ID: test_mock
   Mock 文件将在 3 秒后自动创建

----------------------------------------------------------------------

⏳ 等待用户在 viewer 中选择物体...
   Session ID: test_mock
   超时设置: 15 秒
   监控文件: /tmp/viewer_selection_test_mock.json
   (用户在 viewer 中选择后点击 'Confirm' 即可继续)

   [Mock] 等待 3 秒后创建 mock selection 文件...
   [Mock] ✅ 已创建 mock 文件: /tmp/viewer_selection_test_mock.json
✅ 收到用户选择: 2 个物体
   1. couch (0-7-12)
   2. table (0-7-15)

✅ 已清理临时文件: /tmp/viewer_selection_test_mock.json
----------------------------------------------------------------------

✅ Mock 测试成功！
   收到 2 个物体

🧹 清理测试文件: /tmp/viewer_selection_test_mock.json
```

### 测试 2: 实际等待模式（多终端协作）

这个模式模拟真实场景，需要手动发送选择。

#### Terminal 1: 启动 API Server

```bash
python3 scripts/api_server.py --port 8090
```

#### Terminal 2: 启动等待测试

```bash
python3 test_selection_wait.py
```

**输出：**
```
==================================================================
🧪 测试 Session-Based 用户选择等待机制
==================================================================

📋 测试配置:
   Session ID: a1b2c3d4
   超时设置: 30 秒
   临时文件: /tmp/viewer_selection_a1b2c3d4.json

🚀 开始等待用户选择...

   提示: 在另一个终端运行以下命令来模拟用户选择:
   python3 scripts/test_realtime_selection.py --session a1b2c3d4

----------------------------------------------------------------------

⏳ 等待用户在 viewer 中选择物体...
   Session ID: a1b2c3d4
   超时设置: 30 秒
   监控文件: /tmp/viewer_selection_a1b2c3d4.json
   (用户在 viewer 中选择后点击 'Confirm' 即可继续)

   ... 仍在等待用户选择 (10/30秒)...
```

#### Terminal 3: 模拟用户选择

**选项 A: 单个物体**
```bash
python3 scripts/test_realtime_selection.py --session a1b2c3d4
```

**选项 B: 多个物体**
```bash
python3 scripts/test_realtime_selection.py --multi --session a1b2c3d4
```

**选项 C: 清空选择**
```bash
python3 scripts/test_realtime_selection.py --clear --session a1b2c3d4
```

#### Terminal 2: 看到结果

```
✅ 收到用户选择: 3 个物体
   1. room_007 (0-7)
   2. room_003 (0-3)
   3. room_001 (0-1)

✅ 已清理临时文件: /tmp/viewer_selection_a1b2c3d4.json
----------------------------------------------------------------------

✅ 测试成功！收到用户选择:

   1. room_007
      Code: 0-7
      Type: room

   2. room_003
      Code: 0-3
      Type: room

   3. room_001
      Code: 0-1
      Type: room
```

#### Terminal 1 (API Server): 看到日志

```
============================================================
🎯 实时检测到用户选择！(Session: a1b2c3d4)
============================================================
✅ 用户选择了 3 个物体：

   1. room_007 (0-7)
   2. room_003 (0-3)
   3. room_001 (0-1)

📦 详细信息：
[
  {
    "itemCode": "0-7",
    "displayName": "room_007",
    "type": "room",
    "sourceFile": "output/floor_0/room_007/mesh.ply",
    "timestamp": "2025-10-29T12:34:56.789Z"
  },
  ...
]
============================================================

💾 选择已保存到: /tmp/viewer_selection_a1b2c3d4.json (session-specific)
✅ 文件写入成功
```

## 工作流程

```
┌─────────────────────────────────────────────────────────────┐
│ 1. Agent 检测到需要用户选择                                 │
│    - 查询歧义 (多个 bedroom)                                │
│    - 不明确指代 (the chair)                                  │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. Agent 生成唯一 session_id                                │
│    session_id = uuid.uuid4()[:8]  # e.g., "a1b2c3d4"        │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. Agent 打开 Viewer (selection mode)                       │
│    URL: http://localhost:5173/?manifest=...                 │
│         &selectionMode=true                                 │
│         &sessionId=a1b2c3d4                                  │
│         &prompt=请选择您想查询的bedroom                      │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. Agent 进入等待循环                                        │
│    _wait_for_user_selection(session_id, timeout=60)         │
│    - 每秒检查一次文件                                        │
│    - 最多等待 60 秒                                          │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. 用户在 Viewer 中操作                                      │
│    - 点击选择物体                                            │
│    - 点击 "Confirm All" 按钮                                │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│ 6. Viewer POST 到 API Server                                │
│    POST /api/submit-selection                               │
│    Body: {                                                  │
│      "session_id": "a1b2c3d4",                              │
│      "selections": [...]                                    │
│    }                                                        │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│ 7. API Server 保存到文件                                    │
│    /tmp/viewer_selection_a1b2c3d4.json                      │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│ 8. Agent 检测到文件                                          │
│    - 读取 JSON                                              │
│    - 解析选择结果                                            │
│    - 删除临时文件                                            │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│ 9. Agent 继续执行                                            │
│    - 使用用户选择的物体                                      │
│    - 调用相应工具 (CLR, VOL, BBD 等)                        │
│    - 返回最终结果                                            │
└─────────────────────────────────────────────────────────────┘
```

## 下一步：Phase 2 & 3

Phase 1 ✅ 已完成基础架构。下一步需要：

### Phase 2: Viewer 集成
- [ ] 修改 `PointCloudView2.tsx` 解析 URL 参数
  - `selectionMode`
  - `sessionId`
  - `prompt`
- [ ] 实现 selection mode UI
  - 显示提示信息
  - 高亮候选物体
- [ ] 修改 POST 请求格式包含 `session_id`

### Phase 3: AI 判断逻辑
- [ ] 实现 `_needs_user_disambiguation()` 方法
- [ ] 集成到 `query()` 主流程
  - 检测歧义查询
  - 决定是否需要等待用户选择
- [ ] 测试各种场景

## 故障排除

### 问题 1: 等待超时
**症状**: Agent 一直等待，最终超时

**可能原因**:
- API server 未运行
- Session ID 不匹配
- 文件权限问题

**解决方法**:
```bash
# 检查 API server
curl http://localhost:8090/health

# 检查文件是否创建
ls -la /tmp/viewer_selection_*.json

# 手动测试写入
echo '[{"test": "data"}]' > /tmp/viewer_selection_test123.json
```

### 问题 2: 文件未被清理
**症状**: `/tmp` 目录下有很多 `viewer_selection_*.json` 文件

**解决方法**:
```bash
# 清理所有选择文件
rm -f /tmp/viewer_selection_*.json
```

### 问题 3: 无法读取选择文件
**症状**: `JSONDecodeError` 或 `PermissionError`

**解决方法**:
```bash
# 检查文件权限
ls -la /tmp/viewer_selection_*.json

# 修复权限
chmod 644 /tmp/viewer_selection_*.json
```

## API 参考

### `_wait_for_user_selection(session_id, timeout)`

**参数:**
- `session_id` (str): 唯一会话标识符
- `timeout` (int): 最大等待时间（秒），默认 60

**返回:**
- `List[Dict]`: 用户选择的物体列表
- `None`: 超时或出错

**示例:**
```python
agent = FinalSpatialAIAgent()
result = agent._wait_for_user_selection("abc123", timeout=30)

if result:
    for item in result:
        print(f"选择: {item['displayName']} ({item['itemCode']})")
else:
    print("用户未完成选择")
```

### POST `/api/submit-selection`

**Request Body (新格式):**
```json
{
  "session_id": "a1b2c3d4",
  "selections": [
    {
      "itemCode": "0-7-12",
      "displayName": "couch",
      "type": "object",
      "sourceFile": "/path/to/file.ply",
      "timestamp": "2025-10-29T12:00:00Z"
    }
  ]
}
```

**Request Body (Legacy 格式):**
```json
[
  {
    "itemCode": "0-7-12",
    "displayName": "couch",
    "type": "object",
    "sourceFile": "/path/to/file.ply",
    "timestamp": "2025-10-29T12:00:00Z"
  }
]
```

**Response:**
```json
{
  "success": true,
  "message": "Selection received",
  "count": 1,
  "session_id": "a1b2c3d4"
}
```
