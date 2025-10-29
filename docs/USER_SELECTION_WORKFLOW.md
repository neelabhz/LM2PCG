# User Selection Workflow Design

## 问题陈述

当前 AI agent 无法在执行过程中暂停并等待用户在 web viewer 中完成物体选择。需要实现一个机制来：
1. 判断查询是否需要用户选择（disambiguation）
2. 在需要时触发 viewer 并等待用户选择
3. 获取用户选择后继续执行

## 两种 Viewer 模式

### 1. View-Only 模式（默认）
- **触发条件**: 用户明确请求可视化，且不存在歧义
- **示例查询**:
  - "show me room 0-7"
  - "visualize the kitchen" (只有一个kitchen)
  - "display object 0-7-12"
- **行为**: 打开 viewer，用户可以查看和旋转，但 agent 不等待用户操作，立即返回响应

### 2. Selection-Required 模式
- **触发条件**: 查询存在歧义，需要用户明确选择
- **示例查询**:
  - "show me the bedroom" (有多个bedroom)
  - "what's the color of the chair?" (房间里有多把chair)
  - "move that sofa" (需要用户指定哪个sofa)
- **行为**: 
  1. 打开 viewer，高亮所有候选物体
  2. Agent 进入等待状态（轮询 selection file）
  3. 用户在 viewer 中选择物体并点击 "Confirm"
  4. Agent 收到选择结果，继续执行后续操作

## 实现架构

### 组件交互流程

```
User Query
    ↓
AI Agent (mutli_room_agent2.py)
    ↓
[判断] 是否需要用户选择？
    ↓
    ├─→ NO: View-Only 模式
    │        ↓
    │   调用 VIS tool (viewOnly=true)
    │        ↓
    │   返回 viewer URL
    │        ↓
    │   继续执行，不等待
    │
    └─→ YES: Selection-Required 模式
             ↓
        调用 VIS tool (selectionMode=true, candidates=[...])
             ↓
        返回 viewer URL + selection session ID
             ↓
        进入等待循环:
             ├─ 每1秒检查 /tmp/viewer_selection_{session_id}.json
             ├─ 最多等待 60 秒
             ├─ 检测到选择 → 解析 JSON → 继续执行
             └─ 超时 → 返回错误提示
```

### 新增数据结构

#### 1. Selection Session
```python
@dataclass
class SelectionSession:
    session_id: str  # 唯一标识符
    mode: str  # "room" | "objects" | "multi-room"
    candidates: List[str]  # 候选物体代码列表
    prompt: str  # 提示用户的说明文字
    timeout: int = 60  # 超时时间（秒）
    created_at: float  # 创建时间戳
```

#### 2. Viewer URL 参数扩展
```
# View-Only 模式
http://localhost:5173/?manifest=/manifests/room_007.json

# Selection 模式
http://localhost:5173/?manifest=/manifests/room_007.json&selectionMode=true&sessionId=abc123&prompt=请选择您要操作的bedroom
```

### API 修改

#### ai_api_wrapper.py 修改
```python
class VisOutput(BaseModel):
    status: str
    mode: Optional[str] = None
    name: Optional[str] = None
    viewer_url: Optional[str] = None
    objects: Optional[List[str]] = None
    requires_selection: bool = False  # NEW
    session_id: Optional[str] = None  # NEW
    selection_file: Optional[str] = None  # NEW
    error: Optional[str] = None

def visualize_point_cloud(
    self, 
    codes: List[str], 
    selection_mode: bool = False,  # NEW parameter
    prompt: Optional[str] = None  # NEW parameter
) -> Optional[VisOutput]:
    """
    Trigger visualization via VIS command.
    
    Args:
        codes: List of object/room codes to visualize
        selection_mode: If True, viewer waits for user selection
        prompt: Message to display in viewer when selection is required
    """
    # Implementation...
```

#### mutli_room_agent2.py 新增方法
```python
def _needs_user_disambiguation(
    self, 
    query: str, 
    found_candidates: List[Dict]
) -> Tuple[bool, Optional[str]]:
    """
    判断是否需要用户在 viewer 中选择物体。
    
    Returns:
        (needs_selection, prompt_message)
    """
    # LLM 判断逻辑
    pass

def _wait_for_user_selection(
    self, 
    session_id: str, 
    timeout: int = 60
) -> Optional[List[Dict]]:
    """
    等待用户在 viewer 中完成选择。
    
    轮询 /tmp/viewer_selection_{session_id}.json
    
    Returns:
        用户选择的物体列表，或 None（超时/取消）
    """
    import time
    selection_file = Path(f"/tmp/viewer_selection_{session_id}.json")
    start_time = time.time()
    
    print(f"⏳ 等待用户选择... (最多 {timeout} 秒)")
    print(f"   Selection file: {selection_file}")
    
    while time.time() - start_time < timeout:
        if selection_file.exists():
            try:
                with open(selection_file, 'r') as f:
                    data = json.load(f)
                
                # 清理临时文件
                selection_file.unlink()
                
                print(f"✅ 收到用户选择: {len(data)} 个物体")
                return data
            except Exception as e:
                print(f"⚠️ 读取选择文件失败: {e}")
        
        time.sleep(1)  # 每秒检查一次
    
    print(f"⏱️ 等待超时 ({timeout} 秒)")
    return None
```

### Viewer (PointCloudView2.tsx) 修改

#### URL 参数解析
```typescript
// 检测 selection mode
const urlParams = new URLSearchParams(window.location.search);
const selectionMode = urlParams.get('selectionMode') === 'true';
const sessionId = urlParams.get('sessionId');
const promptMessage = urlParams.get('prompt');

if (selectionMode && sessionId) {
  // 显示选择提示
  setSelectionPrompt(promptMessage || '请选择相关物体');
  
  // 修改 Confirm 按钮行为
  // POST 到 /api/submit-selection 时包含 session_id
}
```

#### API 提交修改
```typescript
const response = await fetch('http://localhost:8090/api/submit-selection', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    session_id: sessionId,  // NEW
    selections: selectionData
  })
});
```

### api_server.py 修改

```python
def _handle_submit_selection(self):
    """Handle POST /api/submit-selection"""
    # ... 解析 body ...
    
    session_id = data.get('session_id')
    selections = data.get('selections', [])
    
    # 如果有 session_id，保存到特定文件
    if session_id:
        selection_file = Path(f"/tmp/viewer_selection_{session_id}.json")
    else:
        selection_file = Path("/tmp/viewer_selection.json")
    
    with open(selection_file, 'w') as f:
        json.dump(selections, f, indent=2, ensure_ascii=False)
    
    # ... rest of the code ...
```

## 使用示例

### 场景1: 明确查询（不需要选择）
```
User: "show me room 0-7"
Agent: [判断] 房间代码明确，不需要选择
Agent: [调用] VIS 0-7 (viewOnly=true)
Agent: [返回] "Room 0-7 is now displayed in viewer: http://localhost:5173/..."
```

### 场景2: 歧义查询（需要选择）
```
User: "what's the color of the bedroom?"
Agent: [查询] 找到 3 个 bedroom (room 0-5, 1-2, 1-7)
Agent: [判断] 存在歧义，需要用户选择
Agent: [调用] VIS 0-5 1-2 1-7 (selectionMode=true, prompt="请选择您想查询的bedroom")
Agent: [等待] 轮询 /tmp/viewer_selection_abc123.json
User: [在viewer中选择 room 1-2 并点击 Confirm]
Agent: [收到] {"itemCode": "1-2", "type": "room", ...}
Agent: [继续] 现在知道用户选的是 room 1-2，调用 CLR 分析颜色
Agent: [返回] "The bedroom (room 1-2) has a dominant color of..."
```

### 场景3: 多个物体选择
```
User: "compare the chairs in the kitchen"
Agent: [查询] 在 kitchen (room 0-2) 找到 4 把 chair
Agent: [判断] 用户说"chairs"（复数），可能想比较多个
Agent: [调用] VIS 0-2-3 0-2-4 0-2-7 0-2-9 (selectionMode=true, prompt="请选择您想比较的chairs（可多选）")
Agent: [等待] 
User: [选择 chair 0-2-3 和 0-2-4，点击 Confirm]
Agent: [收到] [{"itemCode": "0-2-3"}, {"itemCode": "0-2-4"}]
Agent: [继续] 分别调用 CLR 和 VOL 工具
Agent: [返回] "Chair 0-2-3: red, 0.5m³. Chair 0-2-4: blue, 0.6m³."
```

## 实现优先级

### Phase 1: 基础架构 ✅
- [ ] 修改 `VisOutput` 添加 selection 相关字段
- [ ] 实现 `_wait_for_user_selection()` 方法
- [ ] 修改 `api_server.py` 支持 session_id

### Phase 2: Viewer 集成
- [ ] 修改 `PointCloudView2.tsx` 解析 URL 参数
- [ ] 实现 selection mode UI 提示
- [ ] 修改 POST 请求包含 session_id

### Phase 3: AI 判断逻辑
- [ ] 实现 `_needs_user_disambiguation()` 方法
- [ ] 集成到 `query()` 主流程
- [ ] 测试各种歧义场景

### Phase 4: 优化与增强
- [ ] 添加超时进度提示
- [ ] 支持取消等待
- [ ] 错误处理和重试机制
