# 基于 ZRDDS 的 LLM 多智能体无人机无人车任务调度

仓库包含两个边界明确的运行入口：

- `master_agent/` 是当前总 Agent：接收网页对话，通过 OpenAI 或 DeepSeek 把指令解析为 `uwb_map` 下的结构化任务，然后由确定性调度器选设备并驱动任务状态机。LLM 不负责设备选择和安全决策。
- 根目录的 C++ `StructuredPlanner` 是旧版 ZRDDS 演示入口：它只读取已校验的 `task_plan.json`。`llm_planner.py` 仅是给这个旧 Demo 生成兼容 JSON 的辅助工具，不是第二个总 Agent。

两条链路都不会将自然语言直接发送给设备，也不连接真实飞控或车辆底盘。

## 旧版 ZRDDS Demo 架构

```
可选兼容工具 llm_planner.py -> task_plan.json -> StructuredPlanner --TaskRequest-->
                                                |                          |
                                                +-> CoordinateService -----+
                                                +-> UAV / UGV --CandidateBid--> Coordinator
                                                                                |
                                                      TaskAssignment <----------+
UAV / UGV --VehicleState, ExecutionEvent--> DashboardBridge --> telemetry.json --> Browser
```

- `llm_planner.py`：仅为旧 Demo 导出 JSON。OpenAI 使用 Responses API 与严格 JSON Schema；DeepSeek 使用其兼容的 Chat Completions 和 JSON Object 输出，两者最终都经过相同业务校验。
- `StructuredPlanner`：读取 JSON 文件，在 C++ 侧再次校验字段、范围、能力和依赖关系，再映射到 `TaskRequest`；自然语言不会直接进入 DDS。
- `CoordinateService`：使用固定校园原点把 WGS84 经纬高转换为 ENU 米制坐标，并把 ENU 投影到 640 x 440 沙盘像素。
- `VehicleAgent`：分别模拟 UAV 与 UGV 的能力上报、报价、任务接收和执行状态。
- `Coordinator`：唯一的 `TaskAssignment` Writer。只选择能力匹配且有效的候选报价，排序规则为最低成本、其次最短 ETA、最后 bid ID；前序任务未完成的任务不参与分配。
- `DashboardBridge`：DDS Reader，汇总 Topics 后写出浏览器可读的 `dashboard/telemetry.json`。浏览器不直接访问 DDS。

## DDS 契约与 QoS

IDL 位于 [idl/mission.idl](idl/mission.idl)，QoS 说明位于 [config/qos.yaml](config/qos.yaml)。

| Topic | 载荷 | QoS |
| --- | --- | --- |
| `Mission.TaskRequest` | 任务、优先级、能力、WGS84/ENU 目标、依赖 | Reliable + Transient Local |
| `Mission.VehicleCapability` | 设备种类、能力、速度、续航和可用性 | Reliable + Transient Local |
| `Mission.CandidateBid` | 可行性、ETA、成本、有效期 | Reliable + Volatile |
| `Mission.TaskAssignment` | 获选设备、报价、epoch、dispatcher boot ID | Reliable + Transient Local |
| `Mission.VehicleState` | 位置、执行阶段、电量、进度 | Reliable + Transient Local |
| `Mission.ExecutionEvent` | 审计事件与事件时间 | Reliable + Transient Local |
| `Mission.CoordinateTransform` | WGS84、ENU、沙盘像素及地图版本 | Best Effort + Volatile |

`Transient Local` 只能向同一运行期间的晚加入 Reader 重放 Writer 留存的样本，不能替代跨进程重启的数据库或事件记录服务。

## 坐标约定

- 输入：WGS84，`latitude_deg`、`longitude_deg`、`altitude_m`。
- 任务空间：`park_enu_v1` / `CAMPUS_LOCAL`，东、北、天（ENU），单位米。
- 沙盘：原点左上，`x = 80 + 2.8 * east`，`y = 440 - 2.8 * north`，单位像素。
- 旧 Demo 的 `frame_id` 固定为 `park_enu_v1`，`map_version` 固定为 `campus-map-2026.1`。LLM 不能改写它们，任务、坐标转换和设备状态必须使用同一组值。

## 实体沙盘地图资产

迁移自 UrbanProject 的真实 UAV/UGV 沙盘 ROS 栅格地图位于
[`maps/urban-sandbox-v1/`](maps/urban-sandbox-v1/)。该地图是 `300 x 300`
像素、`0.05 m/pixel` 的 `sandbox_map`，配有原始 `map.pgm`、ROS
`map.yaml`、坐标标定记录和完整性验证脚本。

当前 DDS 演示仍使用 `park_enu_v1` 与 640 x 440 的校园示意图，未自动切换
到该实体地图；两者不能混用。将真实设备或 ROS 导航接入本项目时，必须添加
显式的 `CAMPUS_LOCAL -> sandbox_map` 转换，并同步更新 DDS 的 `frame_id` 和
`map_version`。详细的坐标边界、ROS 使用方法和 CARLA RRD 资产清单见
[`maps/urban-sandbox-v1/README.md`](maps/urban-sandbox-v1/README.md)。

验证迁移文件未被损坏：

```powershell
python .\tests\verify_sandbox_map.py
```

## 运行总 Agent（推荐）

总 Agent 支持 `LLM_PROVIDER=deepseek` 和 `LLM_PROVIDER=openai`。对应 API Key 未配置、超时或接口失败时，会自动降级到本地坐标正则解析，不会中断服务。启动和联调说明见 [`master_agent/README.md`](master_agent/README.md)。

## 运行旧版 ZRDDS Demo

以下步骤在 Windows PowerShell 中执行。API Key 只写入当前 PowerShell 进程的环境变量，不要把真实值写入 `.env.example`、源代码或提交记录。

```powershell
cd D:\ZRDDS\ZRDDS-2.5.0\uav_ugv_scheduler
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
# 二选一：OpenAI
$env:LLM_PROVIDER="openai"
$env:OPENAI_API_KEY="你的 OpenAI API Key"
# 可选：$env:OPENAI_MODEL="gpt-4.1-mini"

# 或者 DeepSeek（如果选择它，就不需要设置 OPENAI_API_KEY）
# $env:LLM_PROVIDER="deepseek"
# $env:DEEPSEEK_API_KEY="你的 DeepSeek API Key"
# 可选：$env:DEEPSEEK_MODEL="deepseek-chat"

.\.venv\Scripts\python.exe .\llm_planner.py "前方发生事故，先让无人机热成像侦察，再让无人车送2.5kg医疗物资"
```

成功后会在仓库根目录生成被 `.gitignore` 排除的 `task_plan.json`。然后构建并运行原有 DDS Demo：

也可以不改环境变量，临时在命令中指定供应商和模型：

```powershell
.\.venv\Scripts\python.exe .\llm_planner.py --provider deepseek --model deepseek-chat "前方发生事故，先侦察再送医疗物资"
```

```powershell
.\scripts\build.ps1 -Configuration Debug
.\scripts\run_demo.ps1
python .\tests\verify_telemetry.py --task-plan .\task_plan.json
.\scripts\serve_dashboard.ps1 -Port 8765
```

打开 `http://localhost:8765` 查看沙盘。`run_demo.ps1` 默认读取仓库根目录的 `task_plan.json`；也可以用 `-TaskPlan .\其他计划.json` 指定文件。脚本会设置临时 `ZRDDS_HOME`，使运行时从 SDK 根目录找到 `zrddslicence.lic`，并在退出后恢复原环境变量。

如果只想离线检查 C++/DDS 链路，不调用 API，可以先复制已提交的安全示例：

```powershell
Copy-Item .\task_plan.example.json .\task_plan.json
.\scripts\run_demo.ps1
```

## JSON 与安全边界

当前 Demo 的设备和调度轮次固定，因此计划必须恰好包含两个任务：一个 `SURVEY/CAMERA_THERMAL` 和一个依赖它的 `DELIVERY/MEDICAL_PAYLOAD`。Python 与 C++ 会同时检查：

- 只允许 `TaskRequest` 映射所需的字段，不接受额外字段；
- DDS 字符串长度、枚举、经纬高、截止时间和最大 12 kg 载重；
- 任务 ID 唯一、事故/坐标系/地图版本一致；
- 勘测没有前序任务，投送必须依赖勘测。

`.gitignore` 已排除 `.env`、`task_plan.json`、虚拟环境和构建产物；`.env.example` 仅列 OpenAI/DeepSeek 的变量名和占位值。程序不会自动读取 `.env`，避免误把密钥当作项目数据。OpenAI Key 和 DeepSeek Key 不能互相使用，选择哪个供应商，就设置哪个供应商自己的 Key。

## 示例与验收

`task_plan.example.json` 中的事件 `INC-2026-001` 包含两个关键任务：

1. `INC-2026-001-SURVEY`：UAV `uav-alpha` 以 `CAMERA_THERMAL` 能力进行热成像勘测。
2. `INC-2026-001-DELIVERY`：UGV `ugv-bravo` 以 `MEDICAL_PAYLOAD` 能力投送物资，前提是勘测任务已完成。

`tests/verify_telemetry.py` 会读取本次使用的 `task_plan.json`，验证任务数量、设备分配、完成状态、关键事件及依赖时间顺序。离线校验与 SDK 请求序列化测试可用 `python -m unittest tests/test_llm_planner.py tests/test_openai_request.py` 运行。它们验证的是 JSON 适配和此次沙盘运行生成的 DDS 桥接快照，而非真实设备的安全或飞行认证。
