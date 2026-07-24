---
name: "ti-project-rules"
description: "Enforces project rules for the TI MSPM0小车 project. Invoke when writing or modifying any code in this project. Ensures AI does not modify code without user permission, always considers better alternatives, and summarizes all changes."
---

# TI Project Rules

本项目（TI MSPM0 小车）的强制协作规则。

## 规则 1：修改代码必须征得用户同意

- **AI 不得直接修改代码**，除非用户明确发出修改指令（如"修改"、"改"、"帮我改"、"直接改"等）。
- 默认行为：分析问题 → 指出原因 → 给出修改建议（含具体代码）→ 等待用户确认。
- 即使用户选中了一段代码指出问题，也需先给出分析和建议，等用户确认后再动手。

## 规则 2：每次回答前思考更优方案

- 即使问题看起来简单直接，也要考虑是否存在更好的解决方案。
- 如果有更好的做法，在给出直接答案的同时一并提出，让用户选择。

## 规则 3：修改后必须总结

每次修改代码后，用简洁的格式总结：

- **改了哪些文件**（含行号链接）
- **改了什么内容**
- **为什么这样改**

## 项目背景补充

- MCU: MSPM0G3507
- 传感器: 8路灰度（加权质心法循迹）、VL53L1X、BNO08X、维特智能IMU
- 核心逻辑: 速度环PI + 循迹环PID，直角转弯状态机（TURN_IDLE → TURN_FORWARD → TURN_SPIN → TURN_RECOVER）
- 转弯计数 m0: 每检测到一个直角转弯 +1，4次 = 1圈
- 圈数控制: `quanshu` 变量设定总圈数，`renwu()` 在达到目标后停车
