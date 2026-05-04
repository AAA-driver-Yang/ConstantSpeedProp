# Constant Speed Prop

Constant Speed Prop is a KSP1 Breaking Ground plugin that adds an automatic constant-speed variable-pitch propeller governor to robotic rotors.

## Features

- Automatic blade pitch control
- Target RPM governor
- Fast response mode
- Anti-stall guard
- Works with Breaking Ground robotic rotors and propeller blades
- Supports counter-rotating and coaxial propeller setups

## Requirements

- Kerbal Space Program 1.12.x
- Breaking Ground DLC
- ModuleManager

## Installation

Copy the `GameData/ConstantSpeedProp` folder into your KSP `GameData` folder.

Final path:

```text
Kerbal Space Program/GameData/ConstantSpeedProp/Plugins/ConstantSpeedProp.dll
Kerbal Space Program/GameData/ConstantSpeedProp/Patches/ConstantSpeedProp.cfg
```
ConstantSpeedProp 使用说明（中文版）
============================================================

一、模组定位
------------------------------------------------------------
ConstantSpeedProp 是一个用于《坎巴拉太空计划 KSP1》Breaking Ground / 破土重生 DLC 机器人转子的定速变距螺旋桨控制模组。

核心目标是：

1. 快速响应桨距变化；
2. 尽量保持设定的目标转速；
3. 避免螺旋桨因桨距过大、迎角过大而失速；
4. 删除旧版中会拖慢响应的起飞保护和油门保持逻辑；
5. 更适合前后共轴、反向旋转、多转子螺旋桨布局。

设计思路已经从旧版的“起飞阶段强限制桨距”改为：

RPM 定速控制为主
→ 桨距快速响应
→ 只在出现明显失速风险时强制减小桨距
→ 不再用起飞速度或主油门状态锁住桨距

因此模组更适合需要快速建立推力、需要前后转子同步响应、且发动机功率依赖转速的飞机。

二、安装与确认
------------------------------------------------------------
A：源码编译
1. 用 Visual Studio 2022 打开源码包中的：

   ConstantSpeedProp.sln

2. 顶部选择：

   Release | Any CPU

3. 点击：

   生成 > 生成解决方案

4. 编译成功后，将生成的 DLL 覆盖到：

   X:\KSP\GameData\ConstantSpeedProp\Plugins\ConstantSpeedProp.dll

5. 覆盖前必须完全退出 KSP。KSP 不能热加载 DLL。

6. 重新进入游戏，右键转子，应该看到版本号：

   CSP Version: vx.x.x  xxxxxxx

B:直接复制ConstantSpeedProp到GameData文件夹

如果没有看到这个版本号，说明游戏仍在加载旧 DLL，需要检查是否存在多个 ConstantSpeedProp.dll。

正常情况下，KSP 的 GameData 里只应该有一个：

   E:\KSP\GameData\ConstantSpeedProp\Plugins\ConstantSpeedProp.dll


三、基本使用流程
------------------------------------------------------------
1. 在 SPH / VAB 中放置 Breaking Ground 机器人转子。
2. 在转子上安装 DLC 螺旋桨桨叶或可变距桨叶。
3. 确保桨叶能正常旋转并能通过“偏转角度”产生推力。
4. 右键转子，打开：

   Constant Speed Prop: On

5. 设置 Target RPM、Min Pitch、Max Pitch、Pitch Rate、P Gain 等参数。
6. 试飞时重点观察：

   Current RPM
   Commanded Pitch
   Pitch Limit
   Guard AoA / Measured AoA
   Blade Speed
   Aero Samples

调试重点不再是起飞保护，而是：

1. 目标转速是否合适；
2. 最大桨距是否过大；
3. 桨距变化速度是否足够；
4. 失速保护是否及时介入。


五、核心参数说明
------------------------------------------------------------

1. Constant Speed Prop
---------------------
定速桨总开关。

推荐：

   On

关闭后，模组不会自动控制桨距。


2. Target RPM
-------------
目标转速。模组会尽量让转子的 Current RPM 接近这个值。

重要说明：

转速不仅影响螺旋桨状态，也影响发动机功率表现。Target RPM 设得过低时，控制器会为了压低转速而增大桨距，容易把桨叶推到大桨距失速区。

推荐范围：
  高亚音速型：280-310 rpm(经测试可达1000km/h)
   小型高速桨：450左右 rpm
   普通飞机：430–450 rpm
   大型桨：350–450 rpm
   重型低速机：300–420 rpm

建议起步值：

   450 rpm

如果出现大桨距失速，优先不要降低 RPM，而应考虑提高 Target RPM 或降低 Max Pitch。

推荐使用节流阀只控制转子的扭矩


3. Throttle Sets RPM
--------------------
是否让主油门控制目标 RPM。

推荐：

   No

设为 No 时，Target RPM 是固定目标，更接近现实定速螺旋桨。

设为 Yes 时，主油门越大，目标 RPM 越接近 Target RPM；主油门越小，目标 RPM 越接近 Idle RPM。

对于 DLC 转子，很多飞机并不是直接由主油门控制转子，因此调试阶段建议保持 No。


4. Idle RPM
-----------
低油门目标转速。

只有在 Throttle Sets RPM = Yes 时才主要生效。

推荐：

   120 rpm


5. Min Pitch
------------
最小桨距角。

v3.5 中，Min Pitch 不只是防止桨距过小，也会影响低速响应和发动机负载。

推荐范围：

   轻型机：2–8 deg
   普通飞机：8–15 deg
   高速 / 大桨距配置：12–20 deg

如果飞机在小桨距时推力不足，可以适当提高 Min Pitch。

如果起飞转速上不来，可以降低 Min Pitch。


6. Max Pitch
------------
最大桨距角。

这是防止大桨距失速的第一道简单限制。

推荐范围：

   安全调试：40–45 deg
   普通使用：45–55 deg
   高速性能：55–60 deg

如果出现螺旋桨失速，优先降低 Max Pitch。

不要为了压低 RPM 盲目提高 Max Pitch。Target RPM 过低时，控制器会把桨距推到 Max Pitch 附近，容易失速。


7. Pitch Rate
-------------
桨距最大变化速度。

v3.5 需要更快响应，Pitch Rate 可以比旧版更高。

推荐范围：

   稳定测试：20–30 deg/s
   普通使用：30–40 deg/s
   高速响应：40–60 deg/s

如果起飞时桨距跟不上，提升 Pitch Rate。

如果桨距来回抽动，降低 Pitch Rate。


8. Min Response Rate
--------------------
v3.5 新增参数。只要 RPM 偏离目标超过 RPM Deadband，桨距至少会以这个速度变化。

它的作用是防止 PID 输出太小，导致桨距慢慢爬，特别适合起飞阶段和前后共轴转子同步。

推荐范围：

   稳定：1–2 deg/s
   普通：2–5 deg/s
   快速响应：5–15 deg/s

如果起飞阶段桨距响应仍然慢，优先提高 Min Response Rate。


9. RPM Deadband
---------------
RPM 死区。

当 Current RPM 与 Target RPM 的差距小于这个值时，控制器不会强行微调，减少小幅振荡。

推荐：

   3–8 rpm

常用：

   5 rpm

如果转子一直细微抖动，可以提高到 8–10 rpm。

如果需要更精确保持转速，可以降低到 2–3 rpm。


10. P Gain
----------
比例增益。决定 RPM 偏差出现时，控制器立刻修正的力度。

推荐范围：

   稳定：0.05–0.07
   普通：0.07–0.10
   快速响应：0.10–0.14

如果桨距响应慢，提高 P Gain。

如果桨距震荡，降低 P Gain。


11. I Gain
----------
积分增益。用于消除长期 RPM 偏差。

推荐范围：

   0.001–0.004

常用：

   0.002

I Gain 不宜太大。过高容易产生积累，导致桨距过冲或恢复慢。


12. D Gain
----------
微分增益。

推荐：

   0

KSP 物理帧波动较大，D Gain 容易放大噪声，一般不建议使用。


四、失速保护参数
------------------------------------------------------------

1. Anti-Stall Guard
-------------------
失速保护总开关。

推荐：

   On

打开后，如果 Guard AoA 超过 Max Blade AoA，控制器会强制减小桨距。


2. Use DLC Aero Data
--------------------
是否读取 DLC / Unity 物理数据作为桨叶状态参考。

推荐：

   Yes

说明：

v3.5 虽然不再依赖旧几何硬限制，但仍可使用 DLC 数据辅助判断桨叶速度和估算迎角。

如果 Aero Samples 正常大于 0，说明采样成功。


3. Abs AoA Guard
----------------
是否使用迎角绝对值判断失速保护。

推荐：

   On

原因：

KSP 中左右镜像、反向旋转、共轴结构可能导致模组估算 AoA 出现负号或分支错误。如果 Abs AoA Guard = Off，Measured AoA 为负数时可能绕过 Max Blade AoA 限制。

打开后：

   abs(Guard AoA) > Max Blade AoA

就会触发保护。


4. Target Blade AoA
-------------------
目标桨叶迎角。

它主要用于软限制桨距增加速度，不再作为几何硬上限。

推荐范围：

   安全：5–6 deg
   普通：6–8 deg
   性能：8–10 deg

常用：

   6 deg


5. Max Blade AoA
----------------
最大允许桨叶迎角。

超过该值后，失速保护会强制减小桨距。

推荐范围：

   安全：9–10 deg
   普通：10–12 deg
   性能：12–14 deg

如果仍然出现失速，降低 Max Blade AoA。

如果保护过早介入，导致推力不足，可以适当提高。


6. AoA Inc Rate Cap
-------------------
当桨叶迎角接近 Target Blade AoA 时，允许继续增大桨距的最大速度。

推荐范围：

   稳定：6–10 deg/s
   普通：10–18 deg/s
   快速响应：18–25 deg/s

如果起飞响应慢，提高它。

如果机动中容易失速，降低它。


7. AoA Recovery Rate
--------------------
当 Guard AoA 超过 Max Blade AoA 时，强制减小桨距的速度。

推荐范围：

   普通：60–90 deg/s
   强保护：90–120 deg/s
   极强保护：120–160 deg/s

如果失速保护不够快，提高它。

如果保护介入时推力突然波动，降低它。


8. Min Blade Speed
------------------
最低有效桨叶速度。低于该速度时，DLC 采样迎角可能不可信。

推荐：

   5 m/s

如果低速时 Aero Samples 经常为 0，可以降低到 3 m/s。

如果低速数据乱跳，可以提高到 8 m/s。



五、常见问题与处理
------------------------------------------------------------

1. 起飞时桨距响应仍然慢
------------------------
优先提高：

   Pitch Rate
   Min Response Rate
   AoA Inc Rate Cap
   P Gain

建议：

   Pitch Rate:        35 → 45
   Min Response Rate: 10 → 15
   AoA Inc Rate Cap:  12 → 18
   P Gain:            0.08 → 0.10


2. 起飞时 RPM 高但推力不足
--------------------------
说明桨距偏小或响应不够积极。

调整：

   Min Pitch 提高
   Max Pitch 提高一点
   Min Response Rate 提高
   Target Blade AoA 提高

建议：

   Min Pitch:        12 → 15
   Max Pitch:        50 → 55
   Target Blade AoA: 6 → 7


3. 飞行中螺旋桨失速
--------------------
通常是 Max Pitch 过大、Target RPM 过低，或者失速保护介入太慢。

优先调整：

   Target RPM 提高
   Max Pitch 降低
   Max Blade AoA 降低
   AoA Recovery Rate 提高

建议：

   Target RPM:        350 → 450
   Max Pitch:         55 → 45 或 50
   Max Blade AoA:     12 → 10
   AoA Recovery Rate: 90 → 120


4. 桨距来回震荡
----------------
说明控制太激进。

调整：

   P Gain 降低
   Pitch Rate 降低
   Min Response Rate 降低
   I Gain 降低

建议：

   P Gain:            0.10 → 0.07
   Pitch Rate:        45 → 30
   Min Response Rate: 15 → 8
   I Gain:            0.003 → 0.001


5. Current RPM 长期高于 Target RPM
----------------------------------
说明桨距吸收功率不足。

调整：

   Max Pitch 提高
   P Gain 提高
   I Gain 稍微提高

但不要盲目把 Max Pitch 拉太高，否则可能失速。

建议每次只加 2–3 度：

   Max Pitch: 45 → 48 → 50


6. Current RPM 长期低于 Target RPM
----------------------------------
说明桨距过大或负载过重。

调整：

   Max Pitch 降低
   Min Pitch 降低
   Target RPM 提高
   I Gain 降低




六、调参优先级
------------------------------------------------------------

如果要提高响应速度：

   1. Min Response Rate
   2. Pitch Rate
   3. AoA Inc Rate Cap
   4. P Gain

如果要防止失速：

   1. Max Pitch
   2. Target RPM
   3. Max Blade AoA
   4. AoA Recovery Rate
   5. Abs AoA Guard

如果要提高巡航效率：

   1. Max Pitch
   2. Target Blade AoA
   3. Target RPM
   4. I Gain

如果要减少震荡：

   1. P Gain
   2. Min Response Rate
   3. Pitch Rate
   4. I Gain
