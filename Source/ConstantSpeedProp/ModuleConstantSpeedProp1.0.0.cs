using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using System.Text.RegularExpressions;
using Expansions.Serenity;
using UnityEngine;

namespace ConstantSpeedProp
{
    /// <summary>
    /// Flight-wide tick source. Keeps the governor alive even when KSP does not wake a PartModule
    /// until its right-click menu has been opened.
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class ConstantSpeedPropFlightRunner : MonoBehaviour
    {
        private void FixedUpdate()
        {
            TickAll("CSPFlightRunnerFixed", TimeWarp.fixedDeltaTime);
        }

        private static void TickAll(string source, float dt)
        {
            try
            {
                Vessel vessel = FlightGlobals.ActiveVessel;
                if (vessel == null || vessel.parts == null) return;

                for (int p = 0; p < vessel.parts.Count; p++)
                {
                    Part part = vessel.parts[p];
                    if (part == null || part.Modules == null) continue;

                    for (int m = 0; m < part.Modules.Count; m++)
                    {
                        ModuleConstantSpeedProp csp = part.Modules[m] as ModuleConstantSpeedProp;
                        if (csp != null) csp.ExternalGovernorTick(source, dt);
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogError("[ConstantSpeedProp] Flight runner failed: " + e);
            }
        }
    }

    /// <summary>
    /// ConstantSpeedProp 1.0.0
    ///
    /// KSP1 / Breaking Ground constant-speed variable-pitch propeller governor.
    ///
    /// This build deliberately removes runtime aerodynamic-field based protection. The pitch command is instead built from:
    ///   1. Pitch Perfect style precomputed lift/drag curve targets: max thrust, max efficiency, zero lift and proportional thrust.
    ///   2. Closed-form deployAngle solve using rotor axial inflow plus blade tangential speed.
    ///   3. Target-RPM closed-loop correction layered on top of the Pitch Perfect deployAngle, clamped by rpmCorrectionLimit.
    ///   4. Realtime rotor torque used only as the transition speed / smoothing response factor.
    ///   5. Optional vessel-local pitch synchronization groups.
    ///
    /// Attach this module to a Breaking Ground ModuleRoboticServoRotor part. It controls directly
    /// attached ModuleControlSurface propeller blades.
    /// </summary>
    public class ModuleConstantSpeedProp : PartModule
    {
        private const string VersionText = "1.0.0";
        private const float MaxRpmLimit = 460f;

        private ModuleRoboticServoRotor rotor;
        private readonly List<ModuleControlSurface> blades = new List<ModuleControlSurface>();
        private readonly List<PropBladeModel> bladeModels = new List<PropBladeModel>();

        private float collectivePitch;
        private bool initializedPitch;
        private float integralError;
        private float previousError;
        private float nextBladeRefreshTime;
        private float lastGovernorFixedTime = -999f;
        private int physicsFrameCounter;
        private int nextOptimizerFrame;
        private PitchPerfectSolution cachedSolution;
        private bool hasCachedSolution;

        private bool rpmFilterInitialized;
        private float filteredRpm;
        private bool outputFilterInitialized;
        private float filteredOutputPitch;
        private bool rpmCorrectionFilterInitialized;
        private float filteredRpmCorrection;
        private bool motionRpmInitialized;
        private Vector3 previousBladeRadialDirection;
        private float measuredSignedRpm;

        private float lastLocalTargetPitch;
        private float lastLocalAppliedPitch;
        private float lastLocalAvailableTorque;
        private double lastLocalUt = -1.0;

        private bool pitchSyncGroupEditorOpen;
        private string pitchSyncGroupEditBuffer = "";
        private Rect pitchSyncGroupWindowRect = new Rect(320f, 160f, 380f, 155f);

        private struct RotorSample
        {
            public bool valid;
            public float rawRpm;
            public float signedRpm;
            public float filteredRpm;
            public float wantedRpm;
            public float omega;
            public float targetOmega;
            public float availableTorque;
            public bool hasAvailableTorque;
            public float airDensity;
            public Vector3 axis;
            public Vector3 vesselVelocity;
        }

        private class PropBladeModel
        {
            public ModuleControlSurface surface;
            public Transform surfaceTransform;
            public Vector3 liftVector0;
            public float maxLiftDot;
            public float maxLDDot;
            public float zeroLiftDot;
            public float[] proportionalDots;
            public float radius;
            public bool valid;
        }

        private struct PitchPerfectSolution
        {
            public bool valid;
            public float basePitch;          // deploy angle solved from the target blade AoA
            public float rpmCorrection;      // closed-loop deploy-angle trim, degrees
            public float targetPitch;
            public float targetDot;          // Pitch Perfect target sin(AoA)
            public float targetAoaDeg;       // asin(targetDot) for display
            public float solvedAoaDeg;       // achieved average AoA after numeric solve
            public float targetPerformance;  // lift/drag or thrust target quality indicator
            public float torqueResponseFactor;
            public float throttleFactor;
            public int samples;
            public string source;
        }

        private struct PitchSyncSnapshot
        {
            public double ut;
            public float referencePitch;
            public int members;
        }

        private static readonly Dictionary<string, PitchSyncSnapshot> PitchSyncSnapshots = new Dictionary<string, PitchSyncSnapshot>();

        // --------------------------- User-facing fields ---------------------------

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "定速螺旋桨")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool governorEnabled = true;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "版本")]
        public string versionDisplay = VersionText;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "目标转速", guiUnits = " rpm")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = MaxRpmLimit, stepIncrement = 10f, affectSymCounterparts = UI_Scene.All)]
        public float targetRpm = 450f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "油门控制转速")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "否", enabledText = "是", affectSymCounterparts = UI_Scene.All)]
        public bool throttleSetsTargetRpm = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "怠速转速", guiUnits = " rpm")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = MaxRpmLimit, stepIncrement = 10f, affectSymCounterparts = UI_Scene.All)]
        public float idleRpm = 120f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "最小桨距", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -30f, maxValue = 80f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float minPitch = 2f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "最大桨距", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -30f, maxValue = 80f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float maxPitch = 55f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "桨距速度", guiUnits = " deg/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 1f, maxValue = 120f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float pitchRateLimit = 35f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "桨距平滑", guiUnits = " s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1f, stepIncrement = 0.05f, affectSymCounterparts = UI_Scene.All)]
        public float pitchOutputFilterTau = 0.10f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "转速滤波", guiUnits = " s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1f, stepIncrement = 0.05f, affectSymCounterparts = UI_Scene.All)]
        public float rpmFilterTau = 0.20f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "转速死区", guiUnits = " rpm")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 80f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float rpmDeadband = 10f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "P 转速修正")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 0.20f, stepIncrement = 0.002f, affectSymCounterparts = UI_Scene.All)]
        public float proportionalGain = 0.030f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "I 转速修正")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 0.05f, stepIncrement = 0.0005f, affectSymCounterparts = UI_Scene.All)]
        public float integralGain = 0.0020f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "D 转速修正")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 0.05f, stepIncrement = 0.0005f, affectSymCounterparts = UI_Scene.All)]
        public float derivativeGain = 0.0000f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "转速调节范围", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 45f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float rpmCorrectionLimit = 12f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "PitchPerfect前馈")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool pitchPerfectFeedForward = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "PP模式")]
        [UI_ChooseOption(scene = UI_Scene.All, options = new string[] { "效率", "比例推力", "最大推力", "零升力" }, affectSymCounterparts = UI_Scene.All)]
        public string pitchPerfectMode = "比例推力";

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "推力比例")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1f, stepIncrement = 0.05f, affectSymCounterparts = UI_Scene.All)]
        public float proportionalThrust = 1.00f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "油门设推力比例")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "否", enabledText = "是", affectSymCounterparts = UI_Scene.All)]
        public bool throttleSetsProportionalThrust = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "推力方向")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "反推", enabledText = "正推", affectSymCounterparts = UI_Scene.All)]
        public bool positiveThrust = true;

        // Legacy field retained so old craft/save files load cleanly. The new PP模式 replaces it.
        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false)]
        public float pitchPerfectThrustBias = 0.20f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "前馈刷新帧")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 1f, maxValue = 30f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float optimizerIntervalFrames = 1f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "扭矩平滑")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool torqueSmoothingEnabled = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "扭矩响应参考", guiUnits = " kNm")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0.1f, maxValue = 100f, stepIncrement = 0.5f, affectSymCounterparts = UI_Scene.All)]
        public float torqueResponseReference = 20f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "低扭矩响应")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0.05f, maxValue = 1f, stepIncrement = 0.05f, affectSymCounterparts = UI_Scene.All)]
        public float minimumTorqueResponse = 0.30f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "油门缩放扭矩")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "否", enabledText = "是", affectSymCounterparts = UI_Scene.All)]
        public bool torqueScalesWithThrottle = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "零油门扭矩系数")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 0.5f, stepIncrement = 0.05f, affectSymCounterparts = UI_Scene.All)]
        public float zeroThrottleTorqueFactor = 0.05f;

        // Hidden legacy fields retained only so old craft/save files load cleanly. They are not used.
        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false)]
        public bool torqueCorrectionEnabled = false;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false)]
        public float torqueCorrectionGain = 0f;

        [KSPField(isPersistant = true, guiActive = false, guiActiveEditor = false)]
        public float modelTorqueScale = 1f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "桨距命令偏移", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -30f, maxValue = 30f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float commandPitchOffset = 0f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "顺桨")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool feather = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "顺桨角", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -90f, maxValue = 90f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float featherPitch = 80f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "桨距同步")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool pitchSyncEnabled = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "同步组名")]
        public string pitchSyncGroup = "";

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "同步死区", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 5f, stepIncrement = 0.1f, affectSymCounterparts = UI_Scene.All)]
        public float pitchSyncDeadband = 0.2f;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "当前转速", guiUnits = " rpm", guiFormat = "F0")]
        public float currentRpmDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "目标转速有效值", guiUnits = " rpm", guiFormat = "F0")]
        public float wantedRpmDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "转速误差", guiUnits = " rpm", guiFormat = "F0")]
        public float rpmErrorDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "当前桨距", guiUnits = " deg", guiFormat = "F1")]
        public float commandedPitchDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "PP解算桨叶角", guiUnits = " deg", guiFormat = "F1")]
        public float pitchPerfectPitchDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "PP目标dot角", guiUnits = " deg", guiFormat = "F1")]
        public float pitchPerfectTargetAoaDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "PP输出dot角", guiUnits = " deg", guiFormat = "F1")]
        public float pitchPerfectSolvedAoaDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "转速修正", guiUnits = " deg", guiFormat = "F1")]
        public float rpmCorrectionDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "可用扭矩", guiUnits = " kNm", guiFormat = "F2")]
        public float availableTorqueDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "扭矩平滑系数", guiFormat = "F2")]
        public float torqueResponseDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "PP目标指标", guiFormat = "F2")]
        public float pitchPerfectEfficiencyDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "PP目标sin", guiFormat = "F2")]
        public float pitchPerfectDotDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "主油门", guiUnits = " %", guiFormat = "F0")]
        public float mainThrottleDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "扭矩油门系数", guiFormat = "F2")]
        public float torqueThrottleFactorDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "桨叶数量")]
        public int bladeCountDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "同步状态")]
        public string pitchSyncStateDisplay = "关";

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "同步成员")]
        public int pitchSyncMemberCountDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "同步参考桨距", guiUnits = " deg", guiFormat = "F1")]
        public float pitchSyncReferencePitchDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "循环来源")]
        public string loopSourceDisplay = "未运行";

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "计算来源")]
        public string pitchSourceDisplay = "无";

        // --------------------------- Actions / Events ---------------------------

        [KSPAction("切换定速螺旋桨")]
        public void ToggleGovernor(KSPActionParam param)
        {
            governorEnabled = !governorEnabled;
        }

        [KSPAction("顺桨")]
        public void FeatherAction(KSPActionParam param)
        {
            feather = true;
        }

        [KSPAction("退出顺桨")]
        public void UnfeatherAction(KSPActionParam param)
        {
            feather = false;
        }

        [KSPAction("切换桨距同步")]
        public void TogglePitchSync(KSPActionParam param)
        {
            pitchSyncEnabled = !pitchSyncEnabled;
        }

        [KSPAction("切换正反推")]
        public void ToggleThrustDirection(KSPActionParam param)
        {
            positiveThrust = !positiveThrust;
            hasCachedSolution = false;
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "切换正反推")]
        public void ToggleThrustDirectionEvent()
        {
            positiveThrust = !positiveThrust;
            hasCachedSolution = false;
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "正推力")]
        public void SetPositiveThrustEvent()
        {
            positiveThrust = true;
            hasCachedSolution = false;
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "反推力")]
        public void SetReverseThrustEvent()
        {
            positiveThrust = false;
            hasCachedSolution = false;
        }

        [KSPEvent(guiActive = true, guiActiveEditor = true, guiName = "设置同步组")]
        public void OpenPitchSyncGroupEditor()
        {
            pitchSyncGroupEditBuffer = pitchSyncGroup ?? "";
            pitchSyncGroupEditorOpen = true;
        }

        public void OnGUI()
        {
            if (!pitchSyncGroupEditorOpen) return;
            if (!HighLogic.LoadedSceneIsFlight && !HighLogic.LoadedSceneIsEditor)
            {
                pitchSyncGroupEditorOpen = false;
                return;
            }

            int windowId = 514600 + Math.Abs(GetInstanceID() % 10000);
            pitchSyncGroupWindowRect = ImGuiWindow(windowId, pitchSyncGroupWindowRect, DrawPitchSyncGroupWindow, "桨距同步组");
        }

        private void DrawPitchSyncGroupWindow(int windowId)
        {
            ImGuiLabel("相同组名的螺旋桨会同步最终桨距；留空表示不加入同步组。");
            ImGuiSetNextControlName("CSP_PitchSyncGroup");
            pitchSyncGroupEditBuffer = ImGuiTextField(pitchSyncGroupEditBuffer ?? "", 32);

            ImGuiBeginHorizontal();
            if (ImGuiButton("应用"))
            {
                pitchSyncGroup = CleanPitchSyncGroupName(pitchSyncGroupEditBuffer);
                pitchSyncGroupEditBuffer = pitchSyncGroup;
                pitchSyncGroupEditorOpen = false;
            }
            if (ImGuiButton("清空"))
            {
                pitchSyncGroup = "";
                pitchSyncGroupEditBuffer = "";
                pitchSyncGroupEditorOpen = false;
            }
            if (ImGuiButton("取消")) pitchSyncGroupEditorOpen = false;
            ImGuiEndHorizontal();
            ImGuiDragWindow();
        }

        // --------------------------- Optional IMGUI bridge ---------------------------
        // KSP 1.12 / Unity 2019 splits IMGUI into UnityEngine.IMGUIModule.dll.  Many
        // plugin projects reference only UnityEngine.dll/CoreModule, which makes direct
        // GUI/GUILayout calls fail at compile time.  Reflection keeps this source compatible
        // with both reference layouts.

        private static Type imGuiType;
        private static Type imGUILayoutType;
        private static Type imGUILayoutOptionType;
        private static Type imWindowFunctionType;
        private static MethodInfo imWindowMethod;
        private static MethodInfo imLabelMethod;
        private static MethodInfo imTextFieldMethod;
        private static MethodInfo imBeginHorizontalMethod;
        private static MethodInfo imEndHorizontalMethod;
        private static MethodInfo imButtonMethod;
        private static MethodInfo imSetNextControlNameMethod;
        private static MethodInfo imDragWindowMethod;

        private static Type FindRuntimeType(string fullName)
        {
            Type type = Type.GetType(fullName + ", UnityEngine.IMGUIModule");
            if (type != null) return type;

            type = Type.GetType(fullName + ", UnityEngine");
            if (type != null) return type;

            Assembly[] assemblies = AppDomain.CurrentDomain.GetAssemblies();
            for (int i = 0; i < assemblies.Length; i++)
            {
                try
                {
                    type = assemblies[i].GetType(fullName, false);
                    if (type != null) return type;
                }
                catch (Exception)
                {
                }
            }
            return null;
        }

        private static Array EmptyGUILayoutOptions()
        {
            return Array.CreateInstance(imGUILayoutOptionType, 0);
        }

        private static bool EnsureImGuiMethods()
        {
            if (imWindowMethod != null) return true;

            imGuiType = FindRuntimeType("UnityEngine.GUI");
            imGUILayoutType = FindRuntimeType("UnityEngine.GUILayout");
            imGUILayoutOptionType = FindRuntimeType("UnityEngine.GUILayoutOption");
            imWindowFunctionType = FindRuntimeType("UnityEngine.GUI+WindowFunction");

            if (imGuiType == null || imGUILayoutType == null || imGUILayoutOptionType == null || imWindowFunctionType == null)
            {
                return false;
            }

            MethodInfo[] layoutMethods = imGUILayoutType.GetMethods(BindingFlags.Public | BindingFlags.Static);
            for (int i = 0; i < layoutMethods.Length; i++)
            {
                MethodInfo method = layoutMethods[i];
                ParameterInfo[] parameters = method.GetParameters();

                if (method.Name == "Window" && parameters.Length == 5 &&
                    parameters[0].ParameterType == typeof(int) &&
                    parameters[1].ParameterType == typeof(Rect) &&
                    parameters[2].ParameterType == imWindowFunctionType &&
                    parameters[3].ParameterType == typeof(string))
                {
                    imWindowMethod = method;
                }
                else if (method.Name == "Label" && parameters.Length == 2 && parameters[0].ParameterType == typeof(string))
                {
                    imLabelMethod = method;
                }
                else if (method.Name == "TextField" && parameters.Length == 3 &&
                         parameters[0].ParameterType == typeof(string) && parameters[1].ParameterType == typeof(int))
                {
                    imTextFieldMethod = method;
                }
                else if (method.Name == "BeginHorizontal" && parameters.Length == 1)
                {
                    imBeginHorizontalMethod = method;
                }
                else if (method.Name == "EndHorizontal" && parameters.Length == 0)
                {
                    imEndHorizontalMethod = method;
                }
                else if (method.Name == "Button" && parameters.Length == 2 && parameters[0].ParameterType == typeof(string))
                {
                    imButtonMethod = method;
                }
            }

            imSetNextControlNameMethod = imGuiType.GetMethod("SetNextControlName", BindingFlags.Public | BindingFlags.Static, null, new Type[] { typeof(string) }, null);
            imDragWindowMethod = imGuiType.GetMethod("DragWindow", BindingFlags.Public | BindingFlags.Static, null, Type.EmptyTypes, null);

            return imWindowMethod != null && imLabelMethod != null && imTextFieldMethod != null &&
                   imBeginHorizontalMethod != null && imEndHorizontalMethod != null && imButtonMethod != null &&
                   imSetNextControlNameMethod != null && imDragWindowMethod != null;
        }

        private Rect ImGuiWindow(int id, Rect rect, Action<int> drawFunction, string title)
        {
            if (!EnsureImGuiMethods())
            {
                pitchSyncGroupEditorOpen = false;
                ScreenMessages.PostScreenMessage("ConstantSpeedProp: 找不到 IMGUI 运行时，无法打开同步组编辑窗口。", 3f, ScreenMessageStyle.UPPER_CENTER);
                return rect;
            }

            try
            {
                Delegate callback = Delegate.CreateDelegate(imWindowFunctionType, drawFunction.Target, drawFunction.Method);
                object result = imWindowMethod.Invoke(null, new object[] { id, rect, callback, title, EmptyGUILayoutOptions() });
                return result is Rect ? (Rect)result : rect;
            }
            catch (Exception e)
            {
                pitchSyncGroupEditorOpen = false;
                Debug.LogError("[ConstantSpeedProp] Pitch sync group editor failed: " + e);
                return rect;
            }
        }

        private static void ImGuiLabel(string text)
        {
            if (!EnsureImGuiMethods()) return;
            imLabelMethod.Invoke(null, new object[] { text, EmptyGUILayoutOptions() });
        }

        private static string ImGuiTextField(string text, int maxLength)
        {
            if (!EnsureImGuiMethods()) return text;
            object result = imTextFieldMethod.Invoke(null, new object[] { text, maxLength, EmptyGUILayoutOptions() });
            return result as string ?? text;
        }

        private static void ImGuiBeginHorizontal()
        {
            if (!EnsureImGuiMethods()) return;
            imBeginHorizontalMethod.Invoke(null, new object[] { EmptyGUILayoutOptions() });
        }

        private static void ImGuiEndHorizontal()
        {
            if (!EnsureImGuiMethods()) return;
            imEndHorizontalMethod.Invoke(null, null);
        }

        private static bool ImGuiButton(string text)
        {
            if (!EnsureImGuiMethods()) return false;
            object result = imButtonMethod.Invoke(null, new object[] { text, EmptyGUILayoutOptions() });
            return result is bool && (bool)result;
        }

        private static void ImGuiSetNextControlName(string name)
        {
            if (!EnsureImGuiMethods()) return;
            imSetNextControlNameMethod.Invoke(null, new object[] { name });
        }

        private static void ImGuiDragWindow()
        {
            if (!EnsureImGuiMethods()) return;
            imDragWindowMethod.Invoke(null, null);
        }

        // --------------------------- KSP lifecycle ---------------------------

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            versionDisplay = VersionText;
            ClampPersistentSettings();
            ResetRuntimeState();
        }

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            versionDisplay = VersionText;
            ClampPersistentSettings();
            ResetRuntimeState();
        }

        public override void OnStartFinished(StartState state)
        {
            base.OnStartFinished(state);
            versionDisplay = VersionText;
            ClampPersistentSettings();
            rotor = part.FindModuleImplementing<ModuleRoboticServoRotor>();
            if (rotor == null)
            {
                Debug.LogError("[ConstantSpeedProp] Module must be added to a Breaking Ground robotic rotor part.");
                enabled = false;
                return;
            }
            RefreshBlades();
            InitializePitchFromBlades();
            ResetRuntimeFiltersOnly();
        }

        public override void OnUpdate()
        {
            base.OnUpdate();
            ClampPersistentSettings();
            // Do not run the governor from render Update. The flight runner and FixedUpdate path
            // are guarded so only one integration step happens per Unity physics frame.
        }

        public override void OnFixedUpdate()
        {
            base.OnFixedUpdate();
            RunGovernorLoop("KSPOnFixedUpdate", TimeWarp.fixedDeltaTime);
        }

        public void FixedUpdate()
        {
            RunGovernorLoop("UnityFixedUpdate", TimeWarp.fixedDeltaTime);
        }

        internal void ExternalGovernorTick(string source, float dt)
        {
            RunGovernorLoop(source, dt);
        }

        private void ResetRuntimeState()
        {
            nextBladeRefreshTime = 0f;
            initializedPitch = false;
            lastGovernorFixedTime = -999f;
            physicsFrameCounter = 0;
            nextOptimizerFrame = 0;
            hasCachedSolution = false;
            ResetRuntimeFiltersOnly();
        }

        private void ResetRuntimeFiltersOnly()
        {
            rpmFilterInitialized = false;
            outputFilterInitialized = false;
            rpmCorrectionFilterInitialized = false;
            filteredRpmCorrection = 0f;
            motionRpmInitialized = false;
            measuredSignedRpm = 0f;
            previousBladeRadialDirection = Vector3.zero;
            integralError = 0f;
            previousError = 0f;
            lastLocalTargetPitch = 0f;
            lastLocalAppliedPitch = 0f;
            lastLocalAvailableTorque = 0f;
            lastLocalUt = -1.0;
        }

        private void ClampPersistentSettings()
        {
            targetRpm = Mathf.Clamp(targetRpm, 0f, MaxRpmLimit);
            idleRpm = Mathf.Clamp(idleRpm, 0f, MaxRpmLimit);
            minPitch = Mathf.Clamp(minPitch, -90f, 90f);
            maxPitch = Mathf.Clamp(maxPitch, -90f, 90f);
            pitchRateLimit = Mathf.Clamp(Mathf.Abs(pitchRateLimit), 1f, 120f);
            pitchOutputFilterTau = Mathf.Clamp(pitchOutputFilterTau, 0f, 1f);
            rpmFilterTau = Mathf.Clamp(rpmFilterTau, 0f, 1f);
            rpmDeadband = Mathf.Max(0f, rpmDeadband);
            pitchPerfectThrustBias = Mathf.Clamp01(pitchPerfectThrustBias);
            proportionalThrust = Mathf.Clamp01(proportionalThrust);
            if (string.IsNullOrEmpty(pitchPerfectMode)) pitchPerfectMode = "比例推力";
            if (pitchPerfectMode != "效率" && pitchPerfectMode != "比例推力" && pitchPerfectMode != "最大推力" && pitchPerfectMode != "零升力") pitchPerfectMode = "比例推力";
            optimizerIntervalFrames = Mathf.Clamp(optimizerIntervalFrames, 1f, 30f);
            torqueResponseReference = Mathf.Clamp(torqueResponseReference, 0.1f, 100f);
            minimumTorqueResponse = Mathf.Clamp(minimumTorqueResponse, 0.05f, 1f);
            zeroThrottleTorqueFactor = Mathf.Clamp(zeroThrottleTorqueFactor, 0f, 0.5f);
            torqueCorrectionEnabled = false;
            torqueCorrectionGain = 0f;
            modelTorqueScale = 1f;
            pitchSyncGroup = CleanPitchSyncGroupName(pitchSyncGroup);
        }

        // --------------------------- Main loop ---------------------------

        private void RunGovernorLoop(string source, float dt)
        {
            if (!HighLogic.LoadedSceneIsFlight || FlightDriver.Pause) return;
            if (dt <= 0f) dt = Mathf.Max(TimeWarp.fixedDeltaTime, 0.02f);

            float fixedTime = Time.fixedTime;
            if (Mathf.Abs(fixedTime - lastGovernorFixedTime) < 0.0001f) return;

            if (!EnsureRuntimeReady()) return;

            lastGovernorFixedTime = fixedTime;
            loopSourceDisplay = source;
            physicsFrameCounter++;
            ClampPersistentSettings();

            RotorSample sample = ReadRotorSample(dt);
            currentRpmDisplay = sample.filteredRpm;
            wantedRpmDisplay = sample.wantedRpm;
            availableTorqueDisplay = sample.hasAvailableTorque ? sample.availableTorque : 0f;
            mainThrottleDisplay = GetMainThrottle01() * 100f;
            torqueThrottleFactorDisplay = GetTorqueThrottleFactor();
            bladeCountDisplay = blades.Count;

            if (!governorEnabled || blades.Count == 0)
            {
                pitchSyncStateDisplay = pitchSyncEnabled ? "等待定速" : "关";
                return;
            }

            if (!initializedPitch) InitializePitchFromBlades();

            float lowerPitch = Mathf.Min(minPitch, maxPitch);
            float upperPitch = Mathf.Max(minPitch, maxPitch);
            float pitchToApply;

            if (feather)
            {
                integralError = 0f;
                previousError = 0f;
                collectivePitch = MoveTowardWithRate(collectivePitch, featherPitch, pitchRateLimit, dt);
                pitchToApply = Mathf.Clamp(collectivePitch, lowerPitch, upperPitch);
                outputFilterInitialized = false;
                pitchSourceDisplay = "顺桨";
                pitchSyncStateDisplay = pitchSyncEnabled ? "顺桨旁路" : "关";
            }
            else
            {
                PitchPerfectSolution solution = GetPitchPerfectSolution(sample, lowerPitch, upperPitch, dt);
                pitchPerfectPitchDisplay = solution.basePitch;
                pitchPerfectTargetAoaDisplay = solution.targetAoaDeg;
                pitchPerfectSolvedAoaDisplay = solution.solvedAoaDeg;
                rpmCorrectionDisplay = solution.rpmCorrection;
                torqueResponseDisplay = solution.torqueResponseFactor;
                pitchPerfectEfficiencyDisplay = solution.targetPerformance;
                pitchPerfectDotDisplay = solution.targetDot;
                pitchSourceDisplay = solution.source;

                float wantedPitch = Mathf.Clamp(solution.targetPitch + commandPitchOffset, lowerPitch, upperPitch);
                float effectivePitchRate = pitchRateLimit * Mathf.Clamp(solution.torqueResponseFactor, minimumTorqueResponse, 1f);
                wantedPitch = MoveTowardWithRate(collectivePitch, wantedPitch, effectivePitchRate, dt);
                collectivePitch = wantedPitch;

                pitchToApply = SmoothOutputPitch(collectivePitch, dt, lowerPitch, upperPitch, solution.torqueResponseFactor);

                lastLocalTargetPitch = pitchToApply;
                lastLocalAvailableTorque = sample.hasAvailableTorque ? Mathf.Abs(sample.availableTorque) : 1f;
                lastLocalUt = Planetarium.GetUniversalTime();
                pitchToApply = ApplyPitchSynchronization(pitchToApply, lowerPitch, upperPitch, lastLocalUt);
            }

            ApplyPitchToBlades(pitchToApply);
            commandedPitchDisplay = pitchToApply;
            lastLocalAppliedPitch = pitchToApply;
        }

        private bool EnsureRuntimeReady()
        {
            if (part == null) return false;
            if (rotor == null) rotor = part.FindModuleImplementing<ModuleRoboticServoRotor>();
            if (rotor == null) return false;

            if (Planetarium.GetUniversalTime() >= nextBladeRefreshTime || blades.Count == 0)
            {
                RefreshBlades();
                nextBladeRefreshTime = (float)Planetarium.GetUniversalTime() + 0.5f;
            }

            if (!initializedPitch && blades.Count > 0) InitializePitchFromBlades();
            return blades.Count > 0 || rotor != null;
        }

        // --------------------------- Sampling ---------------------------

        private RotorSample ReadRotorSample(float dt)
        {
            RotorSample sample = new RotorSample();
            sample.valid = true;
            sample.signedRpm = GetSignedRotorRpm();
            sample.rawRpm = Mathf.Abs(sample.signedRpm);
            sample.filteredRpm = GetFilteredRpm(sample.rawRpm, dt);
            sample.wantedRpm = GetWantedRpm();
            sample.omega = Mathf.Abs(sample.signedRpm) * Mathf.PI * 2f / 60f;
            sample.targetOmega = Mathf.Max(0.001f, sample.wantedRpm * Mathf.PI * 2f / 60f);
            sample.axis = GetRotorAxis();
            sample.vesselVelocity = GetVesselSurfaceVelocity();
            sample.airDensity = GetAirDensity();
            sample.hasAvailableTorque = TryReadRotorTorque(out sample.availableTorque);
            if (!sample.hasAvailableTorque)
            {
                sample.availableTorque = EstimateFallbackAvailableTorque(sample);
            }
            return sample;
        }

        private float GetFilteredRpm(float rawRpm, float dt)
        {
            if (!rpmFilterInitialized)
            {
                filteredRpm = rawRpm;
                rpmFilterInitialized = true;
                return filteredRpm;
            }
            filteredRpm = LowPass(filteredRpm, rawRpm, rpmFilterTau, dt);
            return filteredRpm;
        }

        private float GetWantedRpm()
        {
            float target = Mathf.Clamp(targetRpm, 0f, MaxRpmLimit);
            if (!throttleSetsTargetRpm) return target;
            float throttle = FlightInputHandler.state != null ? FlightInputHandler.state.mainThrottle : 0f;
            float idle = Mathf.Clamp(idleRpm, 0f, MaxRpmLimit);
            return Mathf.Clamp(Mathf.Lerp(idle, target, Mathf.Clamp01(throttle)), 0f, MaxRpmLimit);
        }

        private Vector3 GetRotorAxis()
        {
            if (part != null && part.transform != null)
            {
                Vector3 axis = part.transform.up;
                if (axis.sqrMagnitude > 0.000001f) return axis.normalized;
            }
            return Vector3.up;
        }

        private Vector3 GetVesselSurfaceVelocity()
        {
            try
            {
                if (vessel != null)
                {
                    Vector3d v = vessel.srf_velocity;
                    return new Vector3((float)v.x, (float)v.y, (float)v.z);
                }
            }
            catch (Exception)
            {
            }
            return Vector3.zero;
        }

        private float GetAirDensity()
        {
            try
            {
                if (vessel != null)
                {
                    Type vesselType = vessel.GetType();
                    FieldInfo field = vesselType.GetField("atmDensity", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                    if (field != null)
                    {
                        object raw = field.GetValue(vessel);
                        float value;
                        if (TryConvertToFloat(raw, out value) && value > 0f) return value;
                    }
                    PropertyInfo prop = vesselType.GetProperty("atmDensity", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                    if (prop != null)
                    {
                        object raw = prop.GetValue(vessel, null);
                        float value;
                        if (TryConvertToFloat(raw, out value) && value > 0f) return value;
                    }
                }
            }
            catch (Exception)
            {
            }
            return 1.225f;
        }

        private float GetSignedRotorRpm()
        {
            float measured;
            if (TryMeasureSignedRpmFromBladeMotion(out measured)) return measured;

            try
            {
                return rotor != null ? rotor.currentRPM : 0f;
            }
            catch (Exception)
            {
                return 0f;
            }
        }

        private bool TryMeasureSignedRpmFromBladeMotion(out float rpm)
        {
            rpm = 0f;
            if (part == null || part.transform == null || blades.Count == 0) return false;

            ModuleControlSurface blade = null;
            for (int i = 0; i < blades.Count; i++)
            {
                if (blades[i] != null && blades[i].part != null && blades[i].part.transform != null)
                {
                    blade = blades[i];
                    break;
                }
            }
            if (blade == null) return false;

            Vector3 axis = GetRotorAxis();
            Vector3 radial = blade.part.transform.position - part.transform.position;
            radial = Vector3.ProjectOnPlane(radial, axis);
            if (radial.sqrMagnitude < 0.000001f)
            {
                motionRpmInitialized = false;
                return false;
            }
            radial.Normalize();

            if (!motionRpmInitialized)
            {
                previousBladeRadialDirection = radial;
                motionRpmInitialized = true;
                measuredSignedRpm = 0f;
                return false;
            }

            float dt = Mathf.Max(TimeWarp.fixedDeltaTime, 0.001f);
            float angleDeg = Vector3.SignedAngle(previousBladeRadialDirection, radial, axis);
            previousBladeRadialDirection = radial;

            rpm = angleDeg / 360f * 60f / dt;
            if (float.IsNaN(rpm) || float.IsInfinity(rpm))
            {
                rpm = 0f;
                return false;
            }

            measuredSignedRpm = rpm;
            return true;
        }

        // --------------------------- Pitch Perfect style feed-forward ---------------------------

        private PitchPerfectSolution GetPitchPerfectSolution(RotorSample sample, float lowerPitch, float upperPitch, float dt)
        {
            int interval = Mathf.Clamp(Mathf.RoundToInt(optimizerIntervalFrames), 1, 30);
            if (hasCachedSolution && physicsFrameCounter < nextOptimizerFrame)
            {
                PitchPerfectSolution cached = cachedSolution;
                cached.torqueResponseFactor = GetTorqueResponseFactor(sample);
                cached.rpmCorrection = ComputeRpmCorrection(sample, dt, cached.torqueResponseFactor);
                cached.targetPitch = Mathf.Clamp(cached.basePitch + cached.rpmCorrection, lowerPitch, upperPitch);
                return cached;
            }

            PitchPerfectSolution solution = ComputePitchPerfectSolution(sample, lowerPitch, upperPitch, dt);
            cachedSolution = solution;
            hasCachedSolution = true;
            nextOptimizerFrame = physicsFrameCounter + interval;
            return solution;
        }

        private PitchPerfectSolution ComputePitchPerfectSolution(RotorSample sample, float lowerPitch, float upperPitch, float dt)
        {
            PitchPerfectSolution solution = new PitchPerfectSolution();
            solution.valid = false;
            solution.basePitch = Mathf.Clamp(collectivePitch, lowerPitch, upperPitch);
            solution.targetPitch = solution.basePitch;
            solution.targetDot = 0f;
            solution.targetAoaDeg = 0f;
            solution.solvedAoaDeg = 0f;
            solution.targetPerformance = 0f;
            solution.torqueResponseFactor = GetTorqueResponseFactor(sample);
            solution.source = "无PP样本";

            float basePitch;
            float targetDot;
            float targetAoaDeg;
            float solvedAoaDeg;
            float targetPerformance;
            int ppSamples;
            if (pitchPerfectFeedForward && TryComputePitchPerfectBasePitch(sample, lowerPitch, upperPitch, out basePitch, out targetDot, out targetAoaDeg, out solvedAoaDeg, out targetPerformance, out ppSamples))
            {
                solution.valid = true;
                solution.basePitch = Mathf.Clamp(basePitch, lowerPitch, upperPitch);
                solution.targetDot = targetDot;
                solution.targetAoaDeg = targetAoaDeg;
                solution.solvedAoaDeg = solvedAoaDeg;
                solution.targetPerformance = targetPerformance;
                solution.samples = ppSamples;
                solution.source = "PitchPerfect原式";
            }
            else
            {
                solution.basePitch = Mathf.Clamp(collectivePitch, lowerPitch, upperPitch);
                solution.samples = 0;
                solution.source = "保持当前";
            }

            solution.rpmCorrection = ComputeRpmCorrection(sample, dt, solution.torqueResponseFactor);
            solution.targetPitch = Mathf.Clamp(solution.basePitch + solution.rpmCorrection, lowerPitch, upperPitch);
            return solution;
        }

        private float ComputeRpmCorrection(RotorSample sample, float dt, float torqueResponseFactor)
        {
            float rpmError = sample.filteredRpm - sample.wantedRpm;
            rpmErrorDisplay = rpmError;

            float rawCorrection;
            if (Mathf.Abs(rpmError) <= Mathf.Max(0f, rpmDeadband))
            {
                integralError = Mathf.Lerp(integralError, 0f, Mathf.Clamp01(dt * 1.5f));
                previousError = rpmError;
                rawCorrection = 0f;
            }
            else
            {
                integralError = Mathf.Clamp(integralError + rpmError * dt, -1000f, 1000f);
                float derivative = dt > 0.0001f ? (rpmError - previousError) / dt : 0f;
                previousError = rpmError;

                rawCorrection = proportionalGain * rpmError + integralGain * integralError + derivativeGain * derivative;
                rawCorrection = Mathf.Clamp(rawCorrection, -Mathf.Abs(rpmCorrectionLimit), Mathf.Abs(rpmCorrectionLimit));
            }

            if (!torqueSmoothingEnabled)
            {
                filteredRpmCorrection = rawCorrection;
                rpmCorrectionFilterInitialized = true;
                return rawCorrection;
            }

            float response = Mathf.Clamp(torqueResponseFactor, minimumTorqueResponse, 1f);
            float tau = Mathf.Lerp(0.45f, 0.04f, response);
            if (!rpmCorrectionFilterInitialized)
            {
                filteredRpmCorrection = rawCorrection;
                rpmCorrectionFilterInitialized = true;
                return filteredRpmCorrection;
            }

            filteredRpmCorrection = LowPass(filteredRpmCorrection, rawCorrection, tau, dt);
            return filteredRpmCorrection;
        }

        private float GetTorqueResponseFactor(RotorSample sample)
        {
            if (!torqueSmoothingEnabled) return 1f;

            // sample.availableTorque is already throttle-scaled where possible; multiply by the
            // live throttle factor again only for response weighting so the smoothing visibly slows
            // down at low throttle even if KSP exposes only a static motor torque limit.
            float torque = Mathf.Abs(sample.availableTorque) * (torqueScalesWithThrottle ? GetTorqueThrottleFactor() : 1f);
            float reference = Mathf.Max(0.1f, torqueResponseReference);
            float normalized = Mathf.Clamp01(torque / reference);
            float response = Mathf.Lerp(minimumTorqueResponse, 1f, normalized);
            if (float.IsNaN(response) || float.IsInfinity(response)) response = minimumTorqueResponse;
            return Mathf.Clamp(response, 0.05f, 1f);
        }

        private bool TryComputePitchPerfectBasePitch(RotorSample sample, float lowerPitch, float upperPitch, out float basePitch, out float averageTargetDot, out float averageTargetAoaDeg, out float averageSolvedAoaDeg, out float averageTargetPerformance, out int usedSamples)
        {
            basePitch = collectivePitch;
            averageTargetDot = 0f;
            averageTargetAoaDeg = 0f;
            averageSolvedAoaDeg = 0f;
            averageTargetPerformance = 0f;
            usedSamples = 0;
            if (bladeModels.Count == 0) return false;

            float weightedPitchSum = 0f;
            float weightSum = 0f;
            float dotSum = 0f;
            float targetAoaSum = 0f;
            float solvedAoaSum = 0f;
            float perfSum = 0f;
            float thrustSign = positiveThrust ? 1f : -1f;
            float proportion = throttleSetsProportionalThrust ? GetMainThrottle01() : proportionalThrust;

            for (int i = 0; i < bladeModels.Count; i++)
            {
                PropBladeModel model = bladeModels[i];
                if (model == null || !model.valid || model.surface == null) continue;

                Vector3 inflowDir;
                float inflowSpeed;
                if (!TryGetBladeInflow(model, sample, out inflowDir, out inflowSpeed)) continue;

                Vector3 pitchAxis = GetBladePitchAxis(model);
                Vector3 liftVector0 = model.liftVector0;
                if (pitchAxis.sqrMagnitude < 0.000001f || liftVector0.sqrMagnitude < 0.000001f) continue;
                pitchAxis.Normalize();
                liftVector0.Normalize();

                // Pitch Perfect target is a dot product.  Positive/reverse thrust is handled
                // by changing the sign of that target dot, like the original mod's reverse-thrust control,
                // not by inventing a separate AoA or thrust model.
                float targetDot = GetPitchPerfectTargetDot(model, proportion) * thrustSign;
                targetDot = Mathf.Clamp(targetDot, -1f, 1f);

                float deployAngle;
                float achievedDot;
                if (!SolveDeployAnglePitchPerfect(inflowDir, pitchAxis, liftVector0, targetDot, lowerPitch, upperPitch, collectivePitch, out deployAngle, out achievedDot)) continue;

                float weight = Mathf.Max(0.05f, inflowSpeed * inflowSpeed) * Mathf.Max(0.05f, model.radius);
                weightedPitchSum += deployAngle * weight;
                weightSum += weight;
                dotSum += targetDot;
                targetAoaSum += Mathf.Asin(Mathf.Clamp(targetDot, -1f, 1f)) * Mathf.Rad2Deg;
                solvedAoaSum += Mathf.Asin(Mathf.Clamp(achievedDot, -1f, 1f)) * Mathf.Rad2Deg;
                perfSum += GetPitchPerfectTargetMetric(model, targetDot, sample.axis, part.transform.position);
                usedSamples++;
            }

            if (usedSamples == 0 || weightSum <= 0f) return false;
            basePitch = Mathf.Clamp(weightedPitchSum / weightSum, lowerPitch, upperPitch);
            averageTargetDot = dotSum / usedSamples;
            averageTargetAoaDeg = targetAoaSum / usedSamples;
            averageSolvedAoaDeg = solvedAoaSum / usedSamples;
            averageTargetPerformance = perfSum / usedSamples;
            return true;
        }

        private float GetPitchPerfectTargetDot(PropBladeModel model, float proportion)
        {
            if (model == null) return 0f;
            string mode = pitchPerfectMode ?? "比例推力";
            if (mode == "最大推力") return model.maxLiftDot;
            if (mode == "零升力") return model.zeroLiftDot;
            if (mode == "比例推力") return GetProportionalDot(model, Mathf.Clamp01(proportion));
            return model.maxLDDot;
        }

        private float GetPitchPerfectTargetMetric(PropBladeModel model, float targetDot, Vector3 rotorAxis, Vector3 rotorOrigin)
        {
            if (model == null) return 0f;
            string mode = pitchPerfectMode ?? "比例推力";
            if (mode == "最大推力" || mode == "比例推力") return PitchPerfectThrustFunction(model, targetDot, rotorAxis);
            if (mode == "零升力") return Mathf.Abs(EvaluateBladeLiftCurve(model.surface, targetDot));
            return PitchPerfectEfficiencyFunction(model, targetDot, rotorAxis, rotorOrigin);
        }

        private float GetProportionalDot(PropBladeModel model, float proportion)
        {
            if (model == null || model.proportionalDots == null || model.proportionalDots.Length == 0)
            {
                return Mathf.Lerp(model != null ? model.zeroLiftDot : 0f, model != null ? model.maxLiftDot : 0f, Mathf.Clamp01(proportion));
            }
            if (proportion >= 1f) return model.maxLiftDot;
            if (proportion <= 0f) return model.zeroLiftDot;

            float scaled = proportion * (model.proportionalDots.Length - 1);
            int lowerIndex = Mathf.Clamp(Mathf.FloorToInt(scaled), 0, model.proportionalDots.Length - 2);
            float t = scaled - lowerIndex;
            return Mathf.Lerp(model.proportionalDots[lowerIndex], model.proportionalDots[lowerIndex + 1], t);
        }

        private static bool SolveDeployAnglePitchPerfect(Vector3 inflowDir, Vector3 pitchAxis, Vector3 liftVector0, float targetDot, float lowerPitch, float upperPitch, float currentPitch, out float deployAngle, out float achievedDot)
        {
            deployAngle = currentPitch;
            achievedDot = 0f;
            if (inflowDir.sqrMagnitude < 0.000001f || pitchAxis.sqrMagnitude < 0.000001f || liftVector0.sqrMagnitude < 0.000001f) return false;

            inflowDir.Normalize();
            pitchAxis.Normalize();
            liftVector0.Normalize();
            targetDot = Mathf.Clamp(targetDot, -1f, 1f);
            lowerPitch = Mathf.Min(lowerPitch, upperPitch);
            upperPitch = Mathf.Max(lowerPitch, upperPitch);

            // Direct Pitch Perfect solve: try both mathematical roots returned by
            // PropControlMath.GetDeployAngle and pick the valid stock deployAngle closest
            // to the current blade angle.  Do not reinterpret the result as blade AoA.
            float bestPitch = currentPitch;
            float bestDot = 0f;
            float bestScore = float.PositiveInfinity;
            bool any = false;

            for (int branch = 0; branch < 2; branch++)
            {
                float candidate;
                if (!GetDeployAngle(inflowDir, pitchAxis, liftVector0, targetDot, out candidate, branch == 1)) continue;
                if (float.IsNaN(candidate) || float.IsInfinity(candidate)) continue;
                if (candidate < lowerPitch - 0.01f || candidate > upperPitch + 0.01f) continue;

                Vector3 deployedLift = Quaternion.AngleAxis(candidate, pitchAxis) * liftVector0;
                if (deployedLift.sqrMagnitude < 0.000001f) continue;
                deployedLift.Normalize();

                float dot = Mathf.Clamp(Vector3.Dot(inflowDir, deployedLift), -1f, 1f);
                float score = Mathf.Abs(candidate - currentPitch) + 500f * Mathf.Abs(dot - targetDot);
                if (score < bestScore)
                {
                    bestScore = score;
                    bestPitch = candidate;
                    bestDot = dot;
                    any = true;
                }
            }

            if (!any) return false;
            deployAngle = Mathf.Clamp(bestPitch, lowerPitch, upperPitch);
            achievedDot = bestDot;
            return true;
        }

        private static bool SolveDeployAngleForTargetAoa(Vector3 inflowDir, Vector3 pitchAxis, Vector3 liftVector0, float targetDot, float lowerPitch, float upperPitch, float currentPitch, out float deployAngle, out float achievedDot)
        {
            deployAngle = currentPitch;
            achievedDot = 0f;
            if (inflowDir.sqrMagnitude < 0.000001f || pitchAxis.sqrMagnitude < 0.000001f || liftVector0.sqrMagnitude < 0.000001f) return false;

            inflowDir.Normalize();
            pitchAxis.Normalize();
            liftVector0.Normalize();
            targetDot = Mathf.Clamp(targetDot, -1f, 1f);
            lowerPitch = Mathf.Min(lowerPitch, upperPitch);
            upperPitch = Mathf.Max(lowerPitch, upperPitch);

            Func<float, float> dotAtPitch = delegate(float pitch)
            {
                Vector3 liftVector = Quaternion.AngleAxis(pitch, pitchAxis) * liftVector0;
                if (liftVector.sqrMagnitude < 0.000001f) return 0f;
                liftVector.Normalize();
                return Mathf.Clamp(Vector3.Dot(inflowDir, liftVector), -1f, 1f);
            };

            Func<float, float> scoreAtPitch = delegate(float pitch)
            {
                float dot = dotAtPitch(pitch);
                // Primary objective: match target AoA.  The tiny hysteresis term avoids
                // jumping between equivalent mathematical roots on symmetric blades.
                return Mathf.Abs(dot - targetDot) + 0.0015f * Mathf.Abs(pitch - currentPitch);
            };

            int samples = 121;
            float bestPitch = Mathf.Clamp(currentPitch, lowerPitch, upperPitch);
            float bestScore = float.PositiveInfinity;
            for (int i = 0; i <= samples; i++)
            {
                float t = (float)i / samples;
                float p = Mathf.Lerp(lowerPitch, upperPitch, t);
                float score = scoreAtPitch(p);
                if (score < bestScore)
                {
                    bestScore = score;
                    bestPitch = p;
                }
            }

            float step = Mathf.Max(0.05f, (upperPitch - lowerPitch) / samples);
            for (int iter = 0; iter < 20; iter++)
            {
                bool improved = false;
                float left = Mathf.Clamp(bestPitch - step, lowerPitch, upperPitch);
                float right = Mathf.Clamp(bestPitch + step, lowerPitch, upperPitch);
                float leftScore = scoreAtPitch(left);
                float rightScore = scoreAtPitch(right);
                if (leftScore < bestScore)
                {
                    bestScore = leftScore;
                    bestPitch = left;
                    improved = true;
                }
                if (rightScore < bestScore)
                {
                    bestScore = rightScore;
                    bestPitch = right;
                    improved = true;
                }
                if (!improved) step *= 0.5f;
                if (step < 0.005f) break;
            }

            deployAngle = Mathf.Clamp(bestPitch, lowerPitch, upperPitch);
            achievedDot = dotAtPitch(deployAngle);
            return !(float.IsNaN(deployAngle) || float.IsInfinity(deployAngle));
        }

        private Rigidbody GetPartRigidbody(Part p)
        {
            if (p == null) return null;
            try
            {
                FieldInfo field = p.GetType().GetField("rb", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (field != null) return field.GetValue(p) as Rigidbody;
                PropertyInfo prop = p.GetType().GetProperty("rb", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (prop != null) return prop.GetValue(p, null) as Rigidbody;
            }
            catch (Exception)
            {
            }
            return null;
        }

        private bool TryGetBladeInflow(PropBladeModel model, RotorSample sample, out Vector3 inflowDir, out float inflowSpeed)
        {
            inflowDir = Vector3.zero;
            inflowSpeed = 0f;
            if (model == null || model.surface == null || part == null || part.transform == null) return false;

            Vector3 rotorAxis = sample.axis.sqrMagnitude > 0.000001f ? sample.axis.normalized : GetRotorAxis();
            Vector3 refPoint = GetBladeReferencePoint(model.surface);
            Vector3 radial = Vector3.ProjectOnPlane(refPoint - part.transform.position, rotorAxis);
            if (radial.sqrMagnitude < 0.000001f) return false;

            float radius = Mathf.Max(0.05f, radial.magnitude);
            Vector3 radialDir = radial / radius;
            Vector3 tangentDir = Vector3.Cross(rotorAxis, radialDir);
            if (tangentDir.sqrMagnitude < 0.000001f) return false;
            tangentDir.Normalize();

            float signedOmega = sample.signedRpm * Mathf.PI * 2f / 60f;
            Vector3 bladeTangentialVelocity = tangentDir * (signedOmega * radius);

            // Relative wind model requested by the new design: keep only the rotor-axis component
            // of the vessel airflow, then add the opposite of the blade's tangential velocity.
            Vector3 airVelocityWorld = -sample.vesselVelocity;
            Vector3 axialInflow = Vector3.Project(airVelocityWorld, rotorAxis);
            Vector3 inflow = axialInflow - bladeTangentialVelocity;

            // At very low RPM/speed, provide a tiny tangential inflow so the closed-form solver
            // remains well-conditioned and does not randomly jump branches.
            if (inflow.sqrMagnitude < 0.0025f)
            {
                inflow = -tangentDir * 0.05f + axialInflow;
            }

            if (inflow.sqrMagnitude < 0.000001f) return false;
            inflowSpeed = Mathf.Max(0.05f, inflow.magnitude);
            inflowDir = inflow / inflowSpeed;
            return true;
        }

        /// <summary>
        /// Reimplementation of Pitch Perfect's closed-form deploy-angle solve. The targetDot is
        /// the desired dot product between the inflow direction and the deployed lift vector.
        /// </summary>
        private static bool GetDeployAngle(Vector3 inflowDir, Vector3 pitchAxis, Vector3 liftVector0, float targetDot, out float deployAngle, bool minus)
        {
            deployAngle = float.NaN;
            if (targetDot < -1f || targetDot > 1f) return false;

            float a = Vector3.Dot(inflowDir, pitchAxis) * Vector3.Dot(pitchAxis, liftVector0);
            float b = Vector3.Dot(inflowDir, liftVector0);
            float c = Vector3.Dot(inflowDir, Vector3.Cross(pitchAxis, liftVector0));
            float ab = a - b;
            float da = targetDot - a;
            float ab2 = ab * ab;
            float da2 = da * da;
            float c2 = c * c;
            float radicand = da2 * ab2 - 4f * (ab2 + c2) * (da2 - c2);

            if (radicand < -0.000001f) return false;
            if (radicand < 0f) radicand = 0f;

            float denom = 2f * (ab2 + c2);
            if (Mathf.Abs(denom) < 0.000001f) return false;

            float acosArg = (-da * ab + (minus ? -1f : 1f) * Mathf.Sqrt(radicand)) / denom;
            if (acosArg < -1.0001f || acosArg > 1.0001f) return false;
            acosArg = Mathf.Clamp(acosArg, -1f, 1f);

            deployAngle = Mathf.Acos(acosArg) * Mathf.Rad2Deg;
            if (deployAngle > 90f) deployAngle -= 180f;
            return !(float.IsNaN(deployAngle) || float.IsInfinity(deployAngle));
        }

        private void BuildBladeModels()
        {
            bladeModels.Clear();
            Vector3 rotorAxis = GetRotorAxis();
            Vector3 rotorOrigin = part != null && part.transform != null ? part.transform.position : Vector3.zero;

            for (int i = 0; i < blades.Count; i++)
            {
                ModuleControlSurface surface = blades[i];
                if (surface == null || surface.part == null || surface.transform == null) continue;

                PropBladeModel model = new PropBladeModel();
                model.surface = surface;
                model.surfaceTransform = surface.transform;
                model.valid = false;

                Vector3 liftVector0;
                if (!TryGetLiftVector0(surface, out liftVector0)) continue;
                if (liftVector0.sqrMagnitude < 0.000001f) continue;
                model.liftVector0 = liftVector0.normalized;

                Vector3 action = GetBladeReferencePoint(surface) - rotorOrigin;
                Vector3 radial = Vector3.ProjectOnPlane(action, rotorAxis);
                model.radius = Mathf.Max(0.05f, radial.magnitude);

                InitializePitchPerfectDots(model, rotorAxis, rotorOrigin);
                model.valid = true;
                bladeModels.Add(model);
            }
        }

        private bool TryGetLiftVector0(ModuleControlSurface surface, out Vector3 liftVector0)
        {
            liftVector0 = Vector3.up;
            if (surface == null) return false;

            // Use reflection so this build remains tolerant of minor KSP/Unity signature
            // differences. Pitch Perfect uses SetupCoefficients(Vector3.zero, ..., out
            // liftVector_0, ...); here we invoke the same method and pick the strongest
            // Vector3 output as the undeployed lift vector.
            try
            {
                MethodInfo[] methods = surface.GetType().GetMethods(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                for (int i = 0; i < methods.Length; i++)
                {
                    MethodInfo method = methods[i];
                    if (method == null || method.Name != "SetupCoefficients") continue;
                    ParameterInfo[] parameters = method.GetParameters();
                    if (parameters == null || parameters.Length < 3) continue;
                    if (parameters[0].ParameterType != typeof(Vector3)) continue;

                    object[] args = new object[parameters.Length];
                    args[0] = Vector3.zero;
                    for (int p = 1; p < parameters.Length; p++)
                    {
                        Type t = parameters[p].ParameterType;
                        if (t.IsByRef) t = t.GetElementType();
                        args[p] = t != null && t.IsValueType ? Activator.CreateInstance(t) : null;
                    }

                    method.Invoke(surface, args);

                    // Pitch Perfect calls SetupCoefficients(Vector3.zero, out _, out Vector3 liftVector_0, out _, out _).
                    // Prefer that exact second out Vector3 when the signature is compatible; only then fall back
                    // to the strongest Vector3 for KSP build tolerance.
                    if (args.Length > 2 && args[2] is Vector3)
                    {
                        Vector3 exactLift = (Vector3)args[2];
                        if (exactLift.sqrMagnitude > 0.000001f)
                        {
                            liftVector0 = exactLift.normalized;
                            return true;
                        }
                    }

                    Vector3 best = Vector3.zero;
                    float bestMag = 0f;
                    for (int p = 1; p < args.Length; p++)
                    {
                        if (!(args[p] is Vector3)) continue;
                        Vector3 candidate = (Vector3)args[p];
                        float mag = candidate.sqrMagnitude;
                        if (mag > bestMag)
                        {
                            best = candidate;
                            bestMag = mag;
                        }
                    }

                    if (bestMag > 0.000001f)
                    {
                        liftVector0 = best.normalized;
                        return true;
                    }
                }
            }
            catch (Exception)
            {
            }

            try
            {
                liftVector0 = surface.transform.rotation * Vector3.up;
                return liftVector0.sqrMagnitude > 0.000001f;
            }
            catch (Exception)
            {
            }
            return false;
        }

        private void InitializePitchPerfectDots(PropBladeModel model, Vector3 rotorAxis, Vector3 rotorOrigin)
        {
            float upper = FindLiftCurveUpperBound(model.surface);
            if (upper <= 0.001f) upper = 1f;
            upper = Mathf.Clamp(upper, 0.01f, 1f);

            model.maxLiftDot = MaximizeScalar(delegate(float x) { return PitchPerfectThrustFunction(model, x, rotorAxis); }, 0f, upper, 96);
            model.zeroLiftDot = FindZeroLiftDot(model.surface, 0f, model.maxLiftDot);
            model.maxLDDot = MaximizeScalar(delegate(float x) { return PitchPerfectEfficiencyFunction(model, x, rotorAxis, rotorOrigin); }, 0f, Mathf.Max(0.01f, model.maxLiftDot), 96);
            model.proportionalDots = BuildProportionalDots(model, 10);

            model.maxLiftDot = Mathf.Clamp(model.maxLiftDot, -1f, 1f);
            model.maxLDDot = Mathf.Clamp(model.maxLDDot, -1f, 1f);
            model.zeroLiftDot = Mathf.Clamp(model.zeroLiftDot, -1f, 1f);
        }

        private float[] BuildProportionalDots(PropBladeModel model, int proportionCount)
        {
            proportionCount = Mathf.Max(2, proportionCount);
            float[] dots = new float[proportionCount + 1];
            dots[0] = model.zeroLiftDot;
            dots[proportionCount] = model.maxLiftDot;

            float maxLiftValue = Mathf.Max(0f, PitchPerfectThrustFunction(model, model.maxLiftDot, GetRotorAxis()));
            if (maxLiftValue <= 0.0001f)
            {
                for (int i = 1; i < proportionCount; i++) dots[i] = Mathf.Lerp(model.zeroLiftDot, model.maxLiftDot, (float)i / proportionCount);
                return dots;
            }

            float lower = Mathf.Min(model.zeroLiftDot, model.maxLiftDot);
            float upper = Mathf.Max(model.zeroLiftDot, model.maxLiftDot);
            for (int i = 1; i < proportionCount; i++)
            {
                float target = maxLiftValue * i / proportionCount;
                dots[i] = FindDotForThrustFraction(model, target, lower, upper);
            }
            return dots;
        }

        private float FindDotForThrustFraction(PropBladeModel model, float targetThrust, float lower, float upper)
        {
            Vector3 axis = GetRotorAxis();
            float bestDot = lower;
            float bestError = float.PositiveInfinity;

            int samples = 96;
            for (int i = 0; i <= samples; i++)
            {
                float x = Mathf.Lerp(lower, upper, (float)i / samples);
                float error = Mathf.Abs(PitchPerfectThrustFunction(model, x, axis) - targetThrust);
                if (error < bestError)
                {
                    bestError = error;
                    bestDot = x;
                }
            }

            float step = Mathf.Max(0.0005f, (upper - lower) / samples);
            for (int iter = 0; iter < 18; iter++)
            {
                float left = Mathf.Clamp(bestDot - step, lower, upper);
                float right = Mathf.Clamp(bestDot + step, lower, upper);
                float leftError = Mathf.Abs(PitchPerfectThrustFunction(model, left, axis) - targetThrust);
                float rightError = Mathf.Abs(PitchPerfectThrustFunction(model, right, axis) - targetThrust);
                if (leftError < bestError)
                {
                    bestError = leftError;
                    bestDot = left;
                }
                if (rightError < bestError)
                {
                    bestError = rightError;
                    bestDot = right;
                }
                step *= 0.5f;
            }
            return bestDot;
        }

        private float PitchPerfectThrustFunction(PropBladeModel model, float sinAlpha, Vector3 rotorAxis)
        {
            // Match Pitch Perfect's precompute target: lift curve value times the component
            // of the AoA-rotated lift vector along the undeployed lift vector.
            float lift = EvaluateBladeLiftCurve(model.surface, sinAlpha);
            Vector3 pitchAxis = GetBladePitchAxis(model);
            Vector3 liftVector = Quaternion.AngleAxis(Mathf.Asin(Mathf.Clamp(sinAlpha, -1f, 1f)) * Mathf.Rad2Deg, pitchAxis) * model.liftVector0;
            return lift * Vector3.Project(liftVector, model.liftVector0).magnitude;
        }

        private float PitchPerfectEfficiencyFunction(PropBladeModel model, float sinAlpha, Vector3 rotorAxis, Vector3 rotorOrigin)
        {
            float lift = EvaluateBladeLiftCurve(model.surface, sinAlpha);
            float drag = Mathf.Abs(EvaluateBladeDragCurve(model.surface, sinAlpha));
            Vector3 pitchAxis = GetBladePitchAxis(model);
            Vector3 liftVector = Quaternion.AngleAxis(Mathf.Asin(Mathf.Clamp(sinAlpha, -1f, 1f)) * Mathf.Rad2Deg, pitchAxis) * model.liftVector0;
            Vector3 actionPoint = GetBladeReferencePoint(model.surface) - rotorOrigin;
            Vector3 actionNorm = actionPoint.sqrMagnitude > 0.000001f ? actionPoint.normalized : Vector3.right;
            float thrust = lift * Vector3.Project(liftVector, rotorAxis).magnitude;
            float torque = lift * Vector3.Project(Vector3.Cross(liftVector, actionNorm), rotorAxis).magnitude + drag;
            return thrust / Mathf.Max(0.0001f, torque);
        }

        private static float MaximizeScalar(Func<float, float> func, float lower, float upper, int samples)
        {
            lower = Mathf.Min(lower, upper);
            upper = Mathf.Max(lower, upper);
            samples = Mathf.Max(8, samples);

            float bestX = lower;
            float bestY = float.NegativeInfinity;
            for (int i = 0; i <= samples; i++)
            {
                float t = (float)i / samples;
                float x = Mathf.Lerp(lower, upper, t);
                float y = SafeEval(func, x);
                if (y > bestY)
                {
                    bestY = y;
                    bestX = x;
                }
            }

            float step = (upper - lower) / samples;
            float a = Mathf.Max(lower, bestX - step);
            float b = Mathf.Min(upper, bestX + step);
            for (int iter = 0; iter < 16; iter++)
            {
                float c = Mathf.Lerp(a, b, 1f / 3f);
                float d = Mathf.Lerp(a, b, 2f / 3f);
                if (SafeEval(func, c) < SafeEval(func, d)) a = c;
                else b = d;
            }
            return (a + b) * 0.5f;
        }

        private static float SafeEval(Func<float, float> func, float x)
        {
            try
            {
                float y = func(x);
                if (float.IsNaN(y) || float.IsInfinity(y)) return float.NegativeInfinity;
                return y;
            }
            catch (Exception)
            {
                return float.NegativeInfinity;
            }
        }

        private float FindLiftCurveUpperBound(ModuleControlSurface surface)
        {
            try
            {
                if (surface != null && surface.liftCurve != null && surface.liftCurve.Curve != null)
                {
                    float upperBound = 1f;
                    float maxKeyValue = 0f;
                    for (int i = 0; i < surface.liftCurve.Curve.keys.Length; i++)
                    {
                        float keyValue = surface.liftCurve.Curve.keys[i].value;
                        if (keyValue < maxKeyValue)
                        {
                            upperBound = surface.liftCurve.Curve.keys[i].time;
                            break;
                        }
                        maxKeyValue = keyValue;
                    }
                    return Mathf.Clamp(upperBound, 0.01f, 1f);
                }
            }
            catch (Exception)
            {
            }
            return 1f;
        }

        private float FindZeroLiftDot(ModuleControlSurface surface, float lower, float upper)
        {
            float fLower = EvaluateBladeLiftCurve(surface, lower);
            float fUpper = EvaluateBladeLiftCurve(surface, upper);
            if (Mathf.Abs(fLower) < 0.0001f) return lower;
            if (Mathf.Sign(fLower) == Mathf.Sign(fUpper)) return 0f;

            float a = lower;
            float b = upper;
            for (int i = 0; i < 24; i++)
            {
                float mid = (a + b) * 0.5f;
                float fMid = EvaluateBladeLiftCurve(surface, mid);
                if (Mathf.Sign(fLower) == Mathf.Sign(fMid))
                {
                    a = mid;
                    fLower = fMid;
                }
                else
                {
                    b = mid;
                }
            }
            return (a + b) * 0.5f;
        }

        private Vector3 GetBladePitchAxis(PropBladeModel model)
        {
            try
            {
                if (model != null && model.surfaceTransform != null)
                {
                    Vector3 axis = model.surfaceTransform.rotation * Vector3.right;
                    if (axis.sqrMagnitude > 0.000001f) return axis.normalized;
                }
            }
            catch (Exception)
            {
            }
            return Vector3.right;
        }

        private Vector3 GetBladeReferencePoint(ModuleControlSurface surface)
        {
            if (surface == null || surface.transform == null) return Vector3.zero;
            try
            {
                if (surface.displaceVelocity) return surface.transform.TransformPoint(surface.velocityOffset);
            }
            catch (Exception)
            {
            }
            return surface.transform.position;
        }

        private float EvaluateBladeLiftCurve(ModuleControlSurface blade, float sinAlpha)
        {
            try
            {
                if (blade != null && blade.liftCurve != null) return blade.liftCurve.Evaluate(sinAlpha);
            }
            catch (Exception)
            {
            }
            return sinAlpha;
        }

        private float EvaluateBladeDragCurve(ModuleControlSurface blade, float sinAlpha)
        {
            try
            {
                if (blade != null && blade.dragCurve != null) return Mathf.Abs(blade.dragCurve.Evaluate(sinAlpha));
            }
            catch (Exception)
            {
            }
            return 0.02f + 0.18f * Mathf.Abs(sinAlpha);
        }

        // --------------------------- Rotor torque reading ---------------------------

        private bool TryReadRotorTorque(out float torque)
        {
            torque = 0f;
            if (rotor == null) return false;

            float rawTorque = 0f;
            bool found = false;

            // Prefer live/output-like fields. Limit/max fields are accepted only as a fallback,
            // then scaled by throttle below so the response factor still follows pilot input.
            string[] liveNames = new string[]
            {
                "currentTorque", "outputTorque", "availableTorque", "driveTorque",
                "currentMotorTorque", "motorOutput", "motorTorqueOutput", "servoTorque"
            };
            for (int i = 0; i < liveNames.Length; i++)
            {
                if (TryReadObjectFloatMember(rotor, liveNames[i], out rawTorque))
                {
                    rawTorque = NormalizeTorqueValue(rawTorque);
                    if (Mathf.Abs(rawTorque) > 0.0001f) { found = true; break; }
                }
            }

            if (!found)
            {
                string[] limitNames = new string[]
                {
                    "motorTorque", "torque", "maxMotorOutput", "maxTorque", "torqueLimit", "motorTorqueLimit"
                };
                for (int i = 0; i < limitNames.Length; i++)
                {
                    if (TryReadObjectFloatMember(rotor, limitNames[i], out rawTorque))
                    {
                        rawTorque = NormalizeTorqueValue(rawTorque);
                        if (Mathf.Abs(rawTorque) > 0.0001f) { found = true; break; }
                    }
                }
            }

            if (!found)
            {
                try
                {
                    if (rotor.Fields != null)
                    {
                        BaseField bestField = null;
                        float bestValue = 0f;
                        int bestScore = -100000;
                        foreach (BaseField field in rotor.Fields)
                        {
                            if (field == null) continue;
                            string text = GetFieldSearchText(field);
                            if (!LooksLikeTorqueField(text)) continue;

                            float candidate;
                            if (!TryGetFieldFloat(rotor, field, out candidate)) continue;
                            int score = ScoreTorqueField(text);
                            if (score > bestScore)
                            {
                                bestScore = score;
                                bestField = field;
                                bestValue = candidate;
                            }
                        }
                        if (bestField != null)
                        {
                            rawTorque = NormalizeTorqueValue(bestValue);
                            found = Mathf.Abs(rawTorque) > 0.0001f;
                        }
                    }
                }
                catch (Exception)
                {
                }
            }

            if (!found) return false;

            float throttleFactor = torqueScalesWithThrottle ? GetTorqueThrottleFactor() : 1f;
            torque = Mathf.Max(0f, rawTorque * throttleFactor);
            return torque > 0.0001f;
        }

        private float NormalizeTorqueValue(float value)
        {
            if (float.IsNaN(value) || float.IsInfinity(value)) return 0f;
            value = Mathf.Abs(value);
            // Some rotor UI fields are displayed in N*m while others are kN*m-like. Keep this gentle;
            // the value is now used only as a response/smoothing factor, not as a prop-load model.
            if (value > 1000f) value *= 0.001f;
            return value;
        }

        private float EstimateFallbackAvailableTorque(RotorSample sample)
        {
            float baseTorque = 1f + Mathf.Abs(sample.wantedRpm) * 0.01f;
            return Mathf.Max(0.05f, baseTorque * GetTorqueThrottleFactor());
        }

        private float GetMainThrottle01()
        {
            try
            {
                return FlightInputHandler.state != null ? Mathf.Clamp01(FlightInputHandler.state.mainThrottle) : 1f;
            }
            catch (Exception)
            {
                return 1f;
            }
        }

        private float GetTorqueThrottleFactor()
        {
            float throttle = GetMainThrottle01();
            return Mathf.Lerp(Mathf.Clamp01(zeroThrottleTorqueFactor), 1f, throttle);
        }

        private bool TryReadObjectFloatMember(object obj, string name, out float value)
        {
            value = 0f;
            if (obj == null || string.IsNullOrEmpty(name)) return false;
            try
            {
                Type type = obj.GetType();
                FieldInfo field = type.GetField(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (field != null && TryConvertToFloat(field.GetValue(obj), out value)) return true;

                PropertyInfo prop = type.GetProperty(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (prop != null && TryConvertToFloat(prop.GetValue(obj, null), out value)) return true;
            }
            catch (Exception)
            {
            }
            return false;
        }

        private bool LooksLikeTorqueField(string text)
        {
            if (string.IsNullOrEmpty(text)) return false;
            if (text.Contains("扭矩") || text.Contains("torque")) return true;
            if (text.Contains("motor") && text.Contains("output")) return true;
            return false;
        }

        private int ScoreTorqueField(string text)
        {
            int score = 0;
            if (text.Contains("current") || text.Contains("当前")) score += 40;
            if (text.Contains("output") || text.Contains("输出")) score += 35;
            if (text.Contains("available") || text.Contains("可用")) score += 30;
            if (text.Contains("torque") || text.Contains("扭矩")) score += 20;
            if (text.Contains("target") || text.Contains("目标")) score -= 20;
            if (text.Contains("max") || text.Contains("最大")) score -= 25;
            if (text.Contains("limit") || text.Contains("限制")) score -= 20;
            return score;
        }

        private bool TryGetFieldFloat(PartModule module, BaseField field, out float value)
        {
            value = 0f;
            try
            {
                object raw = field.GetValue(module);
                if (TryConvertToFloat(raw, out value)) return true;
            }
            catch (Exception)
            {
            }

            try
            {
                string guiString = field.GetStringValue(module, true);
                if (TryConvertToFloat(guiString, out value)) return true;
            }
            catch (Exception)
            {
            }
            return false;
        }

        private string GetFieldSearchText(BaseField field)
        {
            string gui = string.Empty;
            string internalName = string.Empty;
            try { gui = field.guiName ?? string.Empty; } catch (Exception) { gui = string.Empty; }
            try
            {
                PropertyInfo prop = field.GetType().GetProperty("name");
                object value = prop != null ? prop.GetValue(field, null) : null;
                internalName = value != null ? value.ToString() : string.Empty;
            }
            catch (Exception)
            {
                internalName = string.Empty;
            }
            return (gui + " " + internalName).ToLowerInvariant();
        }

        private bool TryConvertToFloat(object raw, out float value)
        {
            value = 0f;
            if (raw == null) return false;
            if (raw is float)
            {
                value = (float)raw;
                return true;
            }
            if (raw is double)
            {
                value = (float)(double)raw;
                return true;
            }
            if (raw is int)
            {
                value = (int)raw;
                return true;
            }
            if (raw is long)
            {
                value = (long)raw;
                return true;
            }
            return TryConvertToFloat(raw.ToString(), out value);
        }

        private bool TryConvertToFloat(string text, out float value)
        {
            value = 0f;
            if (string.IsNullOrEmpty(text)) return false;
            string cleaned = text.Trim();
            cleaned = cleaned.Replace("kN·m", "").Replace("kNm", "").Replace("N·m", "").Replace("Nm", "");
            cleaned = cleaned.Replace("rpm", "").Replace("RPM", "").Replace("%", "").Replace("°", "");
            Match match = Regex.Match(cleaned, @"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?");
            if (match.Success) cleaned = match.Value;
            return float.TryParse(cleaned, NumberStyles.Float, CultureInfo.InvariantCulture, out value) ||
                   float.TryParse(cleaned, NumberStyles.Float, CultureInfo.CurrentCulture, out value);
        }

        // --------------------------- Blade application ---------------------------

        private void RefreshBlades()
        {
            blades.Clear();
            if (part == null || part.children == null)
            {
                bladeCountDisplay = 0;
                bladeModels.Clear();
                return;
            }

            foreach (Part child in part.children)
            {
                if (child == null) continue;
                ModuleControlSurface surface = child.FindModuleImplementing<ModuleControlSurface>();
                if (surface != null)
                {
                    if (governorEnabled && !surface.deploy) surface.deploy = true;
                    blades.Add(surface);
                }
            }
            bladeCountDisplay = blades.Count;
            BuildBladeModels();
            hasCachedSolution = false;
        }

        private void InitializePitchFromBlades()
        {
            if (blades.Count == 0)
            {
                collectivePitch = Mathf.Clamp(collectivePitch, Mathf.Min(minPitch, maxPitch), Mathf.Max(minPitch, maxPitch));
                initializedPitch = true;
                return;
            }

            float sum = 0f;
            int count = 0;
            for (int i = 0; i < blades.Count; i++)
            {
                ModuleControlSurface blade = blades[i];
                if (blade == null) continue;
                try
                {
                    sum += blade.deployAngle;
                    count++;
                }
                catch (Exception)
                {
                }
            }

            if (count > 0) collectivePitch = sum / count;
            collectivePitch = Mathf.Clamp(collectivePitch, Mathf.Min(minPitch, maxPitch), Mathf.Max(minPitch, maxPitch));
            initializedPitch = true;
        }

        private void ApplyPitchToBlades(float requestedPitch)
        {
            for (int i = 0; i < blades.Count; i++)
            {
                ModuleControlSurface blade = blades[i];
                if (blade == null) continue;
                if (!blade.deploy) blade.deploy = true;

                float lower = Mathf.Min(minPitch, maxPitch);
                float upper = Mathf.Max(minPitch, maxPitch);
                try
                {
                    Vector2 limits = blade.deployAngleLimits;
                    lower = Mathf.Max(lower, Mathf.Min(limits.x, limits.y));
                    upper = Mathf.Min(upper, Mathf.Max(limits.x, limits.y));
                }
                catch (Exception)
                {
                }

                blade.deployAngle = Mathf.Clamp(requestedPitch, lower, upper);
            }
        }

        private float SmoothOutputPitch(float targetPitch, float dt, float lowerPitch, float upperPitch, float torqueResponseFactor)
        {
            if (!outputFilterInitialized)
            {
                filteredOutputPitch = targetPitch;
                outputFilterInitialized = true;
                return Mathf.Clamp(filteredOutputPitch, lowerPitch, upperPitch);
            }

            float response = Mathf.Clamp(torqueResponseFactor, minimumTorqueResponse, 1f);
            float tau = torqueSmoothingEnabled ? pitchOutputFilterTau / Mathf.Max(0.05f, response) : pitchOutputFilterTau;
            filteredOutputPitch = LowPass(filteredOutputPitch, targetPitch, tau, dt);
            return Mathf.Clamp(filteredOutputPitch, lowerPitch, upperPitch);
        }

        // --------------------------- Synchronization ---------------------------

        private float ApplyPitchSynchronization(float localPitch, float hardLower, float hardUpper, double ut)
        {
            string groupName = CleanPitchSyncGroupName(pitchSyncGroup);
            pitchSyncGroup = groupName;

            if (!pitchSyncEnabled || feather || !HighLogic.LoadedSceneIsFlight)
            {
                pitchSyncStateDisplay = pitchSyncEnabled ? "等待飞行" : "关";
                pitchSyncMemberCountDisplay = 0;
                pitchSyncReferencePitchDisplay = localPitch;
                return localPitch;
            }

            if (string.IsNullOrEmpty(groupName))
            {
                pitchSyncStateDisplay = "未分组";
                pitchSyncMemberCountDisplay = 1;
                pitchSyncReferencePitchDisplay = localPitch;
                return localPitch;
            }

            if (vessel == null || vessel.parts == null)
            {
                pitchSyncStateDisplay = "等待载具";
                pitchSyncMemberCountDisplay = 1;
                pitchSyncReferencePitchDisplay = localPitch;
                return localPitch;
            }

            PitchSyncSnapshot snapshot;
            if (!TryGetPitchSyncSnapshot(groupName, ut, out snapshot))
            {
                snapshot = BuildPitchSyncSnapshot(groupName, localPitch, ut);
                StorePitchSyncSnapshot(groupName, snapshot);
            }

            pitchSyncMemberCountDisplay = snapshot.members;
            pitchSyncReferencePitchDisplay = snapshot.referencePitch;
            if (snapshot.members < 2)
            {
                pitchSyncStateDisplay = "等待配对";
                return localPitch;
            }

            float syncedPitch = Mathf.Clamp(snapshot.referencePitch, hardLower, hardUpper);
            pitchSyncStateDisplay = Mathf.Abs(syncedPitch - localPitch) <= Mathf.Max(0f, pitchSyncDeadband) ? "同步" : "结果同步";
            return syncedPitch;
        }

        private PitchSyncSnapshot BuildPitchSyncSnapshot(string groupName, float localPitch, double ut)
        {
            float weightedSum = localPitch * Mathf.Max(0.1f, Mathf.Abs(lastLocalAvailableTorque));
            float weightSum = Mathf.Max(0.1f, Mathf.Abs(lastLocalAvailableTorque));
            int members = 1;

            for (int pIndex = 0; pIndex < vessel.parts.Count; pIndex++)
            {
                Part vesselPart = vessel.parts[pIndex];
                if (vesselPart == null || vesselPart.Modules == null) continue;

                for (int m = 0; m < vesselPart.Modules.Count; m++)
                {
                    ModuleConstantSpeedProp other = vesselPart.Modules[m] as ModuleConstantSpeedProp;
                    if (other == null || other == this) continue;
                    if (!other.pitchSyncEnabled || other.feather) continue;
                    if (CleanPitchSyncGroupName(other.pitchSyncGroup) != groupName) continue;
                    if (Math.Abs(ut - other.lastLocalUt) > 1.0) continue;

                    float weight = Mathf.Max(0.1f, Mathf.Abs(other.lastLocalAvailableTorque));
                    weightedSum += other.lastLocalTargetPitch * weight;
                    weightSum += weight;
                    members++;
                }
            }

            PitchSyncSnapshot snapshot = new PitchSyncSnapshot();
            snapshot.ut = ut;
            snapshot.referencePitch = weightedSum / Mathf.Max(0.1f, weightSum);
            snapshot.members = members;
            return snapshot;
        }

        private bool TryGetPitchSyncSnapshot(string groupName, double ut, out PitchSyncSnapshot snapshot)
        {
            snapshot = new PitchSyncSnapshot();
            string key = GetPitchSyncKey(groupName);
            if (!PitchSyncSnapshots.TryGetValue(key, out snapshot)) return false;
            return Math.Abs(snapshot.ut - ut) < 0.05;
        }

        private void StorePitchSyncSnapshot(string groupName, PitchSyncSnapshot snapshot)
        {
            PitchSyncSnapshots[GetPitchSyncKey(groupName)] = snapshot;
        }

        private string GetPitchSyncKey(string groupName)
        {
            string vesselKey = vessel != null ? vessel.id.ToString() : "novessel";
            return vesselKey + "|" + groupName;
        }

        private static string CleanPitchSyncGroupName(string raw)
        {
            if (string.IsNullOrEmpty(raw)) return "";
            string cleaned = raw.Trim();
            cleaned = Regex.Replace(cleaned, @"\s+", " ");
            if (cleaned.Length > 32) cleaned = cleaned.Substring(0, 32);
            return cleaned;
        }

        // --------------------------- Math helpers ---------------------------

        private float MoveTowardWithRate(float current, float target, float rateDegPerSec, float dt)
        {
            float maxDelta = Mathf.Abs(rateDegPerSec) * Mathf.Max(0f, dt);
            return Mathf.MoveTowards(current, target, maxDelta);
        }

        private static float LowPass(float current, float target, float tau, float dt)
        {
            if (tau <= 0f) return target;
            float alpha = 1f - Mathf.Exp(-Mathf.Max(0f, dt) / Mathf.Max(0.0001f, tau));
            return Mathf.Lerp(current, target, Mathf.Clamp01(alpha));
        }
    }
}
