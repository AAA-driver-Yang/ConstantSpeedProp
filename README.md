ConstantSpeedProp

ConstantSpeedProp 是一个用于《Kerbal Space Program》Breaking Ground 机械转子的恒速螺旋桨控制 Mod。它为安装在机械转子上的 stock 螺旋桨叶片提供自动变距控制，使玩家可以像使用真实恒速螺旋桨一样，设定目标转速后由调速器自动调整桨叶角度。

本 Mod 的核心目标不是简单地固定桨距，而是将每片螺旋桨叶片视为一个可变桨距控制面，直接控制 stock ModuleControlSurface.deployAngle。初始化时，Mod 会读取 KSP 原生的 liftCurve 与 dragCurve，预计算最大推力、最高效率、零升力和比例推力等目标；飞行中则根据转子轴向来流与叶片切向速度合成相对风，并使用类似 Pitch Perfect 的几何反解方法计算基础桨叶角。

在此基础上，ConstantSpeedProp 会根据玩家设定的目标 RPM 对 Pitch Perfect 风格的输出角进行正负修正，从而实现恒速控制。实时转子扭矩不会直接参与桨距求解，而是用于调节桨距过渡速度和平滑程度，使高扭矩状态下响应更快、低扭矩状态下变化更柔和。

主要功能
为 Breaking Ground 机械转子提供恒速螺旋桨调速器
支持设定目标转速，并根据实际 RPM 自动修正桨距
直接控制 stock ModuleControlSurface.deployAngle
基于 KSP 原生 liftCurve / dragCurve 预计算
支持最大推力、最高效率、比例推力、零升力模式
支持正推力与反推力切换
支持油门控制目标转速
支持顺桨功能
支持多转子自定义同步组，使同组转子保持一致桨距
实时扭矩用于控制桨距变化速度与平滑过渡
使用方式

将 Mod 安装到 GameData 后，所有带有 Breaking Ground ModuleRoboticServoRotor 的机械转子都会获得 ConstantSpeedProp 模块。玩家需要将螺旋桨叶片安装在转子上，并确保叶片本身使用 stock 控制面模块。

在飞行中打开转子右键菜单，启用“定速螺旋桨”，设置目标转速、桨距范围、转速修正范围和 Pitch Perfect 模式。起飞或巡航时，Mod 会自动调整桨叶角度，使转子尽量维持目标 RPM。

推荐用途

ConstantSpeedProp 适合用于：

螺旋桨飞机
倾转旋翼机
多发活塞飞机
共轴反转螺旋桨
可变桨距推进器
使用 Breaking Ground 机械转子的自制航空器
需求
Kerbal Space Program 1.x
Breaking Ground / Serenity 扩展包
ModuleManager
注意事项

ConstantSpeedProp 主要面向 stock Breaking Ground 转子与 stock 控制面桨叶设计。不同桨叶安装方向、转子朝向、对称方式和正反转设置可能需要调整最小桨距、最大桨距、推力方向或桨距命令偏移。

未能达到预期效果请尝试调节推力反向
