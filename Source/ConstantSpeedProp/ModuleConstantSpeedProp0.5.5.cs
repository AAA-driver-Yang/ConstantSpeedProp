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
    /// Flight-wide tick source.  This prevents the historical KSP PAW/right-click wake-up problem:
    /// even when KSP does not call a PartModule physics callback until the PAW is opened, this
    /// addon still ticks every ConstantSpeedProp module on the active vessel.
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
    /// KSP1 / Breaking Ground constant-speed variable-pitch propeller governor.
    ///
    /// v0.4.5-resultsync-customgroup-rpm460-slim:
    /// - Full-source release based on the public v0.4.1 design.
    /// - Adds vessel-local propeller pitch synchronization groups.
    /// - Clamps Target RPM and Idle RPM to 460 rpm both in UI and at runtime.
    ///
    /// The module is intended to be attached to Breaking Ground robotic rotors.
    /// It controls directly attached ModuleControlSurface propeller blades.
    /// </summary>
    public class ModuleConstantSpeedProp : PartModule
    {
        private const string VersionText = "0.5.5";
        private const float MaxRpmLimit = 460f;

        private ModuleRoboticServoRotor rotor;
        private readonly List<ModuleControlSurface> blades = new List<ModuleControlSurface>();

        private float collectivePitch;
        private float integralError;
        private float previousError;
        private bool initializedPitch;
        private float nextBladeRefreshTime;
        private double lastGovernorUniversalTime = -1.0;
        private float lastGovernorFixedTime = -999f;

        private AeroData cachedAeroData;
        private bool hasCachedAeroData;
        private int physicsFrameCounter;
        private int nextAeroSampleFrame;
        private int lastAeroSampleFrame = -1;

        private bool rpmFilterInitialized;
        private float filteredRpm;
        private bool aeroFilterInitialized;
        private float filteredSignedAoa;
        private float filteredGuardAoa;
        private float filteredBladeSpeed;
        private float filteredAverageForwardLift;
        private float filteredMinForwardLift;
        private bool pitchOutputFilterInitialized;
        private float filteredOutputPitch;

        private float lastLocalResultPitch;
        private double lastLocalResultUt = -1.0;

        private float forwardLiftNormalizationSign = 1f;
        private int forwardLiftNegativeStableSamples;
        private bool forwardLiftGuardLatched;
        private int forwardLiftGuardTriggerSamples;
        private int forwardLiftGuardReleaseSamples;

        private float aoaNormalizationSign = 1f;
        private int aoaPositiveStableSamples;
        private int aoaNegativeStableSamples;
        private int highAoaStableSamples;
        private bool lastLocalProtectionActive;
        private float lastLocalProtectionPitch;
        private double lastLocalProtectionUt = -1.0;

        private bool motionRpmInitialized;
        private Vector3 previousBladeRadialDirection;
        private float measuredSignedRpm;

        private struct PitchSyncSnapshot
        {
            public double ut;
            public float referencePitch;
            public int members;
            public bool anyProtectionActive;
        }

        private static readonly Dictionary<string, PitchSyncSnapshot> PitchSyncSnapshots = new Dictionary<string, PitchSyncSnapshot>();

        private bool pitchSyncGroupEditorOpen;
        private string pitchSyncGroupEditBuffer = "";
        private Rect pitchSyncGroupWindowRect = new Rect(320f, 160f, 360f, 150f);

        private struct AeroData
        {
            public bool valid;
            public float signedAverageAoa;
            public float guardAoa;
            public float averageBladeSpeed;
            public bool hasForwardLift;
            public float averageForwardLift;
            public float minForwardLift;
            public int forwardLiftSampleCount;
            public int sampleCount;
            public bool hasDlcPanelAoa;
            public int dlcAoaSampleCount;
            public string source;
        }

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
        public float pitchRateLimit = 25f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "最小响应速度", guiUnits = " deg/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 60f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float minimumCorrectionRate = 0f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "气动采样间隔")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 1f, maxValue = 30f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float aeroSampleIntervalFrames = 10f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "转速滤波", guiUnits = " s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1f, stepIncrement = 0.05f, affectSymCounterparts = UI_Scene.All)]
        public float rpmFilterTau = 0.20f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "气动滤波", guiUnits = " s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1f, stepIncrement = 0.05f, affectSymCounterparts = UI_Scene.All)]
        public float aeroFilterTau = 0.30f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "桨距平滑", guiUnits = " s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1f, stepIncrement = 0.05f, affectSymCounterparts = UI_Scene.All)]
        public float pitchOutputFilterTau = 0.10f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "转速死区", guiUnits = " rpm")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 80f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float rpmDeadband = 15f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "P 响应")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1f, stepIncrement = 0.005f, affectSymCounterparts = UI_Scene.All)]
        public float proportionalGain = 0.04f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "I 积分")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 0.2f, stepIncrement = 0.001f, affectSymCounterparts = UI_Scene.All)]
        public float integralGain = 0.0008f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "D 阻尼")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 0.2f, stepIncrement = 0.001f, affectSymCounterparts = UI_Scene.All)]
        public float derivativeGain = 0.0f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "顺桨")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool feather = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "顺桨角", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -90f, maxValue = 90f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float featherPitch = 80f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "防失速保护")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool antiStallEnabled = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "目标叶片迎角", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 20f, stepIncrement = 0.5f, affectSymCounterparts = UI_Scene.All)]
        public float targetBladeAoa = 8f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "最大叶片迎角", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 2f, maxValue = 45f, stepIncrement = 0.5f, affectSymCounterparts = UI_Scene.All)]
        public float maxBladeAoa = 14f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "迎角限粗速度", guiUnits = " deg/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 60f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float aoaPitchIncreaseLimit = 3f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "失速退桨速度", guiUnits = " deg/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 1f, maxValue = 120f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float aoaRecoveryRate = 20f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "前进升力保护")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool forwardLiftGuardEnabled = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "负前进升力死区", guiUnits = " kN")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 20f, stepIncrement = 0.1f, affectSymCounterparts = UI_Scene.All)]
        public float negativeForwardLiftDeadband = 0.5f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "负升力恢复速度", guiUnits = " deg/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 1f, maxValue = 120f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float negativeLiftRecoveryRate = 45f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "升力保护最低油门", guiUnits = " %")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 100f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float forwardLiftGuardMinThrottlePercent = 0f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "桨距命令偏移", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -30f, maxValue = 30f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float commandPitchOffset = 0f;

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

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "转速误差", guiUnits = " rpm", guiFormat = "F0")]
        public float rpmErrorDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "目标转速上限", guiUnits = " rpm", guiFormat = "F0")]
        public float rpmLimitDisplay = MaxRpmLimit;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "当前桨距", guiUnits = " deg", guiFormat = "F1")]
        public float commandedPitchDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "桨距速度命令", guiUnits = " deg/s", guiFormat = "F1")]
        public float pitchRateCommandDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "桨距上限", guiUnits = " deg", guiFormat = "F1")]
        public float pitchLimitDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "平均迎角", guiUnits = " deg", guiFormat = "F1")]
        public float estimatedAoaDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "保护迎角", guiUnits = " deg", guiFormat = "F1")]
        public float guardAoaDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "前进升力", guiUnits = " kN", guiFormat = "F1")]
        public float forwardLiftDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "升力样本数")]
        public int forwardLiftSampleCountDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "升力保护状态")]
        public string forwardLiftGuardActiveDisplay = "否";

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "主油门", guiUnits = " %", guiFormat = "F0")]
        public float mainThrottleDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "迎角来源")]
        public string aoaSourceDisplay = "无";

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "DLC迎角样本")]
        public int dlcAoaSampleCountDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "DLC迎角字段")]
        public string dlcAoaFieldDisplay = "未找到";

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "DLC迎角读值")]
        public string dlcAoaReadDisplay = "无";

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "气动样本数")]
        public int aeroSampleCountDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "气动采样状态")]
        public string aeroSampleHoldDisplay = "否";

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "循环来源")]
        public string loopSourceDisplay = "未运行";

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "桨叶数量")]
        public int bladeCountDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "同步状态")]
        public string pitchSyncStateDisplay = "关";

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "同步成员")]
        public int pitchSyncMemberCountDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "同步参考桨距", guiUnits = " deg", guiFormat = "F1")]
        public float pitchSyncReferencePitchDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "同步桨距误差", guiUnits = " deg", guiFormat = "F2")]
        public float pitchSyncErrorDisplay;

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
            if (ImGuiButton("取消"))
            {
                pitchSyncGroupEditorOpen = false;
            }
            ImGuiEndHorizontal();
            ImGuiDragWindow();
        }

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

        internal void ExternalGovernorTick(string source, float dt)
        {
            RunGovernorLoop(source, dt);
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            versionDisplay = VersionText;
            ClampPersistentSettings();
            nextBladeRefreshTime = 0f;
            initializedPitch = false;
            ResetRuntimeFilters();
        }

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            versionDisplay = VersionText;
            ClampPersistentSettings();
            nextBladeRefreshTime = 0f;
            initializedPitch = false;
            ResetRuntimeFilters();
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
            ResetRuntimeFilters();
        }

        public override void OnUpdate()
        {
            base.OnUpdate();
            ClampPersistentSettings();
            // Do not run the governor from render Update.
            // Running both Update and FixedUpdate makes pitch integration depend on frame rate
            // and amplifies stale DLC data during take-off.  The flight runner below ticks
            // every module once per physics frame even when the PAW is never opened.
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

        private void ClampPersistentSettings()
        {
            targetRpm = Mathf.Clamp(targetRpm, 0f, MaxRpmLimit);
            idleRpm = Mathf.Clamp(idleRpm, 0f, MaxRpmLimit);
            rpmLimitDisplay = MaxRpmLimit;
            pitchSyncGroup = CleanPitchSyncGroupName(pitchSyncGroup);
        }

        private string LocalizeAoaSource(string source)
        {
            if (string.IsNullOrEmpty(source)) return "无";
            string text = source;
            text = text.Replace("DLC Runtime AoA", "DLC运行迎角");
            text = text.Replace("DLC Display AoA", "DLC显示迎角");
            text = text.Replace("DLC Degree AoA", "DLC度数迎角");
            text = text.Replace("DLC Panel AoA", "DLC迎角");
            text = text.Replace("Filtered", "滤波");
            text = text.Replace("DLC Runtime Forward Lift", "DLC运行前进升力");
            text = text.Replace("DLC Forward Lift", "DLC前进升力");
            text = text.Replace("None", "无");
            return text;
        }

        private void RunGovernorLoop(string source, float dt)
        {
            if (!HighLogic.LoadedSceneIsFlight || FlightDriver.Pause) return;
            if (dt <= 0f) dt = Mathf.Max(TimeWarp.fixedDeltaTime, 0.02f);

            // Exactly one governor integration step per Unity physics frame.
            // KSP can call PartModule.OnFixedUpdate, MonoBehaviour.FixedUpdate and the
            // global KSPAddon runner in the same frame; without this guard the pitch
            // integrator can run two or three times and oscillate.
            float fixedTime = Time.fixedTime;
            if (Mathf.Abs(fixedTime - lastGovernorFixedTime) < 0.0001f) return;

            double ut = Planetarium.GetUniversalTime();
            if (!EnsureRuntimeReady()) return;

            lastGovernorFixedTime = fixedTime;
            lastGovernorUniversalTime = ut;
            loopSourceDisplay = source;
            physicsFrameCounter++;
            ClampPersistentSettings();

            float rawRpm = GetCurrentRotorRpm();
            currentRpmDisplay = GetFilteredRpm(rawRpm, dt);
            bladeCountDisplay = blades.Count;

            if (!governorEnabled || blades.Count == 0)
            {
                pitchSyncStateDisplay = pitchSyncEnabled ? "等待定速" : "关";
                return;
            }

            if (!initializedPitch) InitializePitchFromBlades();

            float lowerPitch = Mathf.Min(minPitch, maxPitch);
            float upperPitch = Mathf.Max(minPitch, maxPitch);
            float wantedRpm = GetWantedRpm();
            AeroData aero = GetAeroDataDecimated(wantedRpm, dt);

            pitchLimitDisplay = upperPitch;
            estimatedAoaDisplay = aero.signedAverageAoa;
            guardAoaDisplay = aero.guardAoa;
            forwardLiftDisplay = aero.hasForwardLift ? aero.minForwardLift : 0f;
            forwardLiftSampleCountDisplay = aero.forwardLiftSampleCount;
            mainThrottleDisplay = GetMainThrottlePercent();
            aoaSourceDisplay = LocalizeAoaSource(aero.source);
            dlcAoaSampleCountDisplay = aero.dlcAoaSampleCount;
            aeroSampleCountDisplay = aero.sampleCount;
            forwardLiftGuardActiveDisplay = IsForwardLiftGuardArmed() ? (forwardLiftGuardLatched ? "保护" : "待命") : "否";

            float pitchToApply;
            float requestedPitchRate = 0f;

            if (feather)
            {
                integralError = 0f;
                previousError = 0f;
                requestedPitchRate = Mathf.Sign(featherPitch - collectivePitch) * Mathf.Abs(pitchRateLimit);
                collectivePitch = MoveTowardWithRate(collectivePitch, featherPitch, pitchRateLimit, dt);
                pitchToApply = collectivePitch;
                pitchOutputFilterInitialized = false;
                filteredAverageForwardLift = 0f;
                filteredMinForwardLift = 0f;
                pitchSyncStateDisplay = pitchSyncEnabled ? "顺桨旁路" : "关";
            }
            else
            {
                float rpmError = currentRpmDisplay - wantedRpm;
                rpmErrorDisplay = rpmError;
                requestedPitchRate = ComputeGovernorPitchRate(rpmError, dt);

                bool protectionActive = false;
                if (antiStallEnabled && (aero.valid || aero.hasForwardLift))
                {
                    protectionActive = ApplyStallGuard(ref requestedPitchRate, aero, wantedRpm);
                }

                requestedPitchRate = LimitPitchRateAndProtectIntegral(requestedPitchRate, lowerPitch, upperPitch);
                collectivePitch = Mathf.Clamp(collectivePitch + requestedPitchRate * dt, lowerPitch, upperPitch);
                pitchToApply = SmoothOutputPitch(collectivePitch, dt, lowerPitch, upperPitch);

                float localResultPitch = pitchToApply;
                pitchToApply = ApplyPitchSynchronization(localResultPitch, lowerPitch, upperPitch, ut);
                lastLocalResultPitch = localResultPitch;
                lastLocalResultUt = ut;
                lastLocalProtectionActive = protectionActive;
                lastLocalProtectionPitch = localResultPitch;
                lastLocalProtectionUt = ut;
            }

            pitchRateCommandDisplay = requestedPitchRate;
            ApplyPitchToBlades(pitchToApply);
            commandedPitchDisplay = pitchToApply;
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

        private void ResetRuntimeFilters()
        {
            hasCachedAeroData = false;
            physicsFrameCounter = 0;
            nextAeroSampleFrame = 0;
            lastAeroSampleFrame = -1;
            rpmFilterInitialized = false;
            aeroFilterInitialized = false;
            pitchOutputFilterInitialized = false;
            aeroSampleHoldDisplay = "否";
            pitchSyncMemberCountDisplay = 0;
            pitchSyncErrorDisplay = 0f;
            lastLocalResultPitch = 0f;
            lastLocalResultUt = -1.0;
            forwardLiftNormalizationSign = 1f;
            forwardLiftNegativeStableSamples = 0;
            forwardLiftGuardLatched = false;
            forwardLiftGuardTriggerSamples = 0;
            forwardLiftGuardReleaseSamples = 0;
            aoaNormalizationSign = 1f;
            aoaPositiveStableSamples = 0;
            aoaNegativeStableSamples = 0;
            highAoaStableSamples = 0;
            lastLocalProtectionActive = false;
            lastLocalProtectionPitch = 0f;
            lastLocalProtectionUt = -1.0;
            lastGovernorFixedTime = -999f;
            motionRpmInitialized = false;
            previousBladeRadialDirection = Vector3.zero;
            measuredSignedRpm = 0f;
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

        private AeroData GetAeroDataDecimated(float wantedRpm, float dt)
        {
            int interval = Mathf.Clamp(Mathf.RoundToInt(aeroSampleIntervalFrames), 1, 30);
            bool shouldSample = !hasCachedAeroData || physicsFrameCounter >= nextAeroSampleFrame;
            if (!shouldSample)
            {
                aeroSampleHoldDisplay = "保持";
                return cachedAeroData;
            }

            AeroData raw = GetAeroData(wantedRpm);
            int elapsedFrames = lastAeroSampleFrame >= 0 ? Mathf.Max(1, physicsFrameCounter - lastAeroSampleFrame) : interval;
            float sampleDt = Mathf.Max(dt, TimeWarp.fixedDeltaTime) * elapsedFrames;
            cachedAeroData = FilterAeroData(raw, sampleDt);
            hasCachedAeroData = true;
            lastAeroSampleFrame = physicsFrameCounter;
            nextAeroSampleFrame = physicsFrameCounter + interval;
            aeroSampleHoldDisplay = "采样";
            return cachedAeroData;
        }

        private AeroData FilterAeroData(AeroData raw, float dt)
        {
            if (!raw.valid && !raw.hasForwardLift) return raw;
            if (!aeroFilterInitialized)
            {
                filteredSignedAoa = raw.signedAverageAoa;
                filteredGuardAoa = raw.guardAoa;
                filteredBladeSpeed = raw.averageBladeSpeed;
                filteredAverageForwardLift = raw.averageForwardLift;
                filteredMinForwardLift = raw.minForwardLift;
                aeroFilterInitialized = true;
                return raw;
            }

            AeroData result = raw;
            if (raw.valid)
            {
                filteredSignedAoa = LowPass(filteredSignedAoa, raw.signedAverageAoa, aeroFilterTau, dt);
                filteredGuardAoa = LowPass(filteredGuardAoa, raw.guardAoa, aeroFilterTau, dt);
                filteredBladeSpeed = LowPass(filteredBladeSpeed, raw.averageBladeSpeed, aeroFilterTau, dt);
                result.signedAverageAoa = filteredSignedAoa;
                result.guardAoa = filteredGuardAoa;
                result.averageBladeSpeed = filteredBladeSpeed;
            }
            if (raw.hasForwardLift)
            {
                filteredAverageForwardLift = LowPass(filteredAverageForwardLift, raw.averageForwardLift, aeroFilterTau, dt);
                filteredMinForwardLift = LowPass(filteredMinForwardLift, raw.minForwardLift, aeroFilterTau, dt);
                result.averageForwardLift = filteredAverageForwardLift;
                result.minForwardLift = filteredMinForwardLift;
            }
            if (!string.IsNullOrEmpty(result.source) && !result.source.Contains("Filtered")) result.source = result.source + " Filtered";
            return result;
        }

        private float SmoothOutputPitch(float targetPitch, float dt, float lowerPitch, float upperPitch)
        {
            if (!pitchOutputFilterInitialized)
            {
                filteredOutputPitch = targetPitch;
                pitchOutputFilterInitialized = true;
                return Mathf.Clamp(filteredOutputPitch, lowerPitch, upperPitch);
            }
            filteredOutputPitch = LowPass(filteredOutputPitch, targetPitch, pitchOutputFilterTau, dt);
            return Mathf.Clamp(filteredOutputPitch, lowerPitch, upperPitch);
        }

        private float ComputeGovernorPitchRate(float rpmError, float dt)
        {
            if (Mathf.Abs(rpmError) <= Mathf.Max(0f, rpmDeadband))
            {
                integralError = Mathf.Lerp(integralError, 0f, Mathf.Clamp01(dt * 1.5f));
                previousError = rpmError;
                return 0f;
            }

            integralError = Mathf.Clamp(integralError + rpmError * dt, -500f, 500f);
            float derivative = (rpmError - previousError) / dt;
            previousError = rpmError;
            float rate = proportionalGain * rpmError + integralGain * integralError + derivativeGain * derivative;

            float floor = Mathf.Abs(minimumCorrectionRate);
            float floorStartError = Mathf.Max(Mathf.Max(0f, rpmDeadband) * 3f, 30f);
            if (floor > 0f && Mathf.Abs(rpmError) >= floorStartError)
            {
                if (rate > 0f) rate = Mathf.Max(rate, floor);
                else if (rate < 0f) rate = Mathf.Min(rate, -floor);
                else rate = Mathf.Sign(rpmError) * floor;
            }
            return rate;
        }

        private float LimitPitchRateAndProtectIntegral(float requestedPitchRate, float lowerPitch, float upperPitch)
        {
            float limit = Mathf.Abs(pitchRateLimit);
            float rate = Mathf.Clamp(requestedPitchRate, -limit, limit);

            if (collectivePitch >= upperPitch - 0.1f && rate > 0f)
            {
                integralError = Mathf.Min(0f, integralError);
                return 0f;
            }
            if (collectivePitch <= lowerPitch + 0.1f && rate < 0f)
            {
                integralError = Mathf.Max(0f, integralError);
                return 0f;
            }
            return rate;
        }

        private float GetMainThrottlePercent()
        {
            float throttle = FlightInputHandler.state != null ? FlightInputHandler.state.mainThrottle : 0f;
            return Mathf.Clamp01(throttle) * 100f;
        }

        private bool IsForwardLiftGuardArmed()
        {
            if (!forwardLiftGuardEnabled) return false;
            float threshold = Mathf.Clamp(forwardLiftGuardMinThrottlePercent, 0f, 100f);
            return GetMainThrottlePercent() >= threshold;
        }

        private bool ApplyStallGuard(ref float requestedPitchRate, AeroData aero, float wantedRpm)
        {
            bool protectionActive = false;

            if (IsForwardLiftGuardArmed() && aero.hasForwardLift)
            {
                protectionActive |= ApplyForwardLiftSoftGuard(ref requestedPitchRate, aero, wantedRpm);
            }
            else
            {
                forwardLiftGuardLatched = false;
                forwardLiftGuardTriggerSamples = 0;
                forwardLiftGuardReleaseSamples = 0;
            }

            if (!aero.valid)
            {
                highAoaStableSamples = 0;
                return protectionActive;
            }

            float guardAoa = aero.guardAoa;
            float targetAoa = Mathf.Min(targetBladeAoa, maxBladeAoa - 0.5f);

            if (guardAoa >= maxBladeAoa)
            {
                highAoaStableSamples++;

                // One bad DLC sample must not instantly kick the propeller into a large
                // fine-pitch command.  The first frame only prevents further coarse pitch;
                // sustained high AoA then commands recovery toward fine pitch.
                if (highAoaStableSamples < 2)
                {
                    requestedPitchRate = Mathf.Min(requestedPitchRate, 0f);
                    integralError = Mathf.Min(integralError, 0f);
                    return true;
                }

                float severity = Mathf.Clamp01((guardAoa - maxBladeAoa) / 10f + 0.20f);
                float recoveryRate = Mathf.Min(Mathf.Abs(aoaRecoveryRate) * severity, Mathf.Abs(pitchRateLimit));
                requestedPitchRate = Mathf.Min(requestedPitchRate, -recoveryRate);
                integralError = 0f;
                return true;
            }

            highAoaStableSamples = 0;

            if (guardAoa >= targetAoa && requestedPitchRate > aoaPitchIncreaseLimit)
            {
                requestedPitchRate = Mathf.Min(requestedPitchRate, Mathf.Max(0f, aoaPitchIncreaseLimit));
                integralError = Mathf.Min(integralError, 0f);
                return true;
            }

            return protectionActive;
        }

        private bool ApplyForwardLiftSoftGuard(ref float requestedPitchRate, AeroData aero, float wantedRpm)
        {
            float deadband = Mathf.Max(0f, negativeForwardLiftDeadband);
            float minLift = aero.minForwardLift;

            // Hysteresis: only latch after stable negative thrust, and release only after the
            // filtered value has clearly recovered.  This prevents torque/RPM/load changes
            // from making the guard and RPM governor fight each other every sample.
            if (minLift < -deadband)
            {
                forwardLiftGuardTriggerSamples++;
                forwardLiftGuardReleaseSamples = 0;
                if (forwardLiftGuardTriggerSamples >= 2) forwardLiftGuardLatched = true;
            }
            else if (minLift > -deadband * 0.35f)
            {
                forwardLiftGuardReleaseSamples++;
                forwardLiftGuardTriggerSamples = 0;
                if (forwardLiftGuardReleaseSamples >= 4) forwardLiftGuardLatched = false;
            }
            else
            {
                forwardLiftGuardTriggerSamples = 0;
                forwardLiftGuardReleaseSamples = 0;
            }

            if (!forwardLiftGuardLatched) return false;

            float rpmError = currentRpmDisplay - wantedRpm;
            float rpmMargin = Mathf.Max(10f, rpmDeadband);

            // If the rotor is already below target RPM, do not force a coarse-pitch recovery:
            // that is exactly the case that creates a limit-cycle between negative-lift guard
            // and the RPM governor.  In that condition the guard only blocks further fine pitch.
            if (rpmError < -rpmMargin)
            {
                if (requestedPitchRate < 0f)
                {
                    requestedPitchRate = 0f;
                    integralError = Mathf.Max(0f, integralError);
                    return true;
                }
                return false;
            }

            float deficit = -minLift - deadband;
            float severity = Mathf.Clamp01(deficit / 10f + 0.15f);
            float recoveryRate = Mathf.Min(Mathf.Abs(negativeLiftRecoveryRate) * severity, Mathf.Abs(pitchRateLimit) * 0.65f);

            // Soft recovery, not a hard override.  This keeps the RPM loop dominant while
            // still preventing sustained negative forward lift.
            if (requestedPitchRate < recoveryRate)
            {
                requestedPitchRate = Mathf.Lerp(requestedPitchRate, recoveryRate, 0.35f);
                integralError = 0f;
                return true;
            }

            return true;
        }

        private float GetWantedRpm()
        {
            float target = Mathf.Clamp(targetRpm, 0f, MaxRpmLimit);
            if (!throttleSetsTargetRpm) return target;
            float throttle = FlightInputHandler.state != null ? FlightInputHandler.state.mainThrottle : 0f;
            float idle = Mathf.Clamp(idleRpm, 0f, MaxRpmLimit);
            return Mathf.Clamp(Mathf.Lerp(idle, target, Mathf.Clamp01(throttle)), 0f, MaxRpmLimit);
        }

        private float GetCurrentRotorRpm()
        {
            return Mathf.Abs(GetSignedRotorRpm());
        }

        private float GetSignedRotorRpm()
        {
            // Do not rely exclusively on ModuleRoboticServoRotor.currentRPM.
            // On some KSP/BG builds that value behaves like a PAW/display value and
            // refreshes reliably only after the rotor window has been opened.
            // Measuring the actual blade motion keeps take-off behaviour identical
            // whether or not the user right-clicks the rotor.
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

            Vector3 axis = part.transform.up;
            if (axis.sqrMagnitude < 0.000001f) return false;
            axis.Normalize();

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

            // At 460 rpm and the default 0.02 s physics step the blade moves about
            // 55 degrees per frame, well below SignedAngle's +/-180 degree ambiguity.
            rpm = angleDeg / 360f * 60f / dt;
            if (float.IsNaN(rpm) || float.IsInfinity(rpm))
            {
                rpm = 0f;
                return false;
            }

            measuredSignedRpm = rpm;
            return true;
        }

        private float GetForwardLiftNormalizationSign(float rawAverageLift)
        {
            // Breaking Ground reports the blade PAW "forward lift" in the blade/rotor
            // local convention.  On a counter-rotating rotor that is producing normal
            // thrust, the DLC value can be permanently negative.  Normalize it to the
            // rotor's signed RPM direction before the negative-lift guard sees it.
            float signedRpm = GetSignedRotorRpm();
            if (signedRpm < -1f)
            {
                forwardLiftNormalizationSign = -1f;
                forwardLiftNegativeStableSamples = 0;
                return forwardLiftNormalizationSign;
            }

            if (signedRpm > 1f && rawAverageLift > Mathf.Max(0.2f, negativeForwardLiftDeadband))
            {
                forwardLiftNormalizationSign = 1f;
                forwardLiftNegativeStableSamples = 0;
                return forwardLiftNormalizationSign;
            }

            // Some KSP/Serenity builds expose currentRPM as an absolute value.  In that
            // case detect a reverse-sign DLC forward-lift convention only after it is
            // stable for several samples while the prop is actually running.
            if (GetCurrentRotorRpm() > 30f && rawAverageLift < -Mathf.Max(0.2f, negativeForwardLiftDeadband))
            {
                forwardLiftNegativeStableSamples++;
                if (forwardLiftNegativeStableSamples >= 2)
                {
                    forwardLiftNormalizationSign = -1f;
                    return forwardLiftNormalizationSign;
                }
            }
            else if (rawAverageLift > Mathf.Max(0.2f, negativeForwardLiftDeadband))
            {
                forwardLiftNegativeStableSamples = 0;
                if (forwardLiftNormalizationSign > 0f) forwardLiftNormalizationSign = 1f;
            }

            return forwardLiftNormalizationSign;
        }

        private AeroData GetAeroData(float wantedRpm)
        {
            AeroData result = new AeroData
            {
                valid = false,
                signedAverageAoa = 0f,
                guardAoa = 0f,
                averageBladeSpeed = 0f,
                hasForwardLift = false,
                averageForwardLift = 0f,
                minForwardLift = 0f,
                forwardLiftSampleCount = 0,
                sampleCount = 0,
                hasDlcPanelAoa = false,
                dlcAoaSampleCount = 0,
                source = "None"
            };

            if (!antiStallEnabled && !forwardLiftGuardEnabled) return result;

            float aoaSum = 0f;
            List<float> aoaValues = new List<float>();
            int aoaSamples = 0;
            float rawLiftSum = 0f;
            float rawMinLift = float.MaxValue;
            float rawMaxLift = float.MinValue;
            int rawLiftSamples = 0;
            dlcAoaFieldDisplay = "未找到";
            dlcAoaReadDisplay = "无";

            for (int i = 0; i < blades.Count; i++)
            {
                ModuleControlSurface blade = blades[i];
                if (blade == null || blade.part == null) continue;

                float forwardLift;
                if (TryReadForwardLiftKn(blade, out forwardLift))
                {
                    rawLiftSum += forwardLift;
                    rawMinLift = Mathf.Min(rawMinLift, forwardLift);
                    rawMaxLift = Mathf.Max(rawMaxLift, forwardLift);
                    rawLiftSamples++;
                }

                float dlcAoa;
                if (TryReadDlcAoaDeg(blade, out dlcAoa))
                {
                    aoaSum += dlcAoa;
                    aoaValues.Add(dlcAoa);
                    aoaSamples++;
                }
            }

            List<float> effectiveAoaMagnitudes = null;
            if (aoaSamples > 0)
            {
                // Some BG/KSP blade PAW fields are stale on individual blade parts unless
                // their PAW has been opened.  In that state a rotor can report one correct
                // DLC AoA sample around 50 deg and two stale samples around 0-5 deg.
                // Treat AoA as a per-rotor protection signal: keep the high, physically
                // relevant DLC blade samples and ignore low stale dropouts.
                effectiveAoaMagnitudes = BuildEffectiveDlcAoaMagnitudes(aoaValues);
                aoaSum = 0f;
                for (int i = 0; i < effectiveAoaMagnitudes.Count; i++) aoaSum += effectiveAoaMagnitudes[i];
            }

            if (rawLiftSamples > 0)
            {
                float rawAverageLift = rawLiftSum / rawLiftSamples;
                float liftSign = GetForwardLiftNormalizationSign(rawAverageLift);

                result.hasForwardLift = true;
                result.averageForwardLift = rawAverageLift * liftSign;
                result.minForwardLift = liftSign >= 0f ? rawMinLift : rawMaxLift * liftSign;
                result.forwardLiftSampleCount = rawLiftSamples;
            }

            if (aoaSamples > 0 && effectiveAoaMagnitudes != null && effectiveAoaMagnitudes.Count > 0)
            {
                result.valid = true;
                result.signedAverageAoa = aoaSum / effectiveAoaMagnitudes.Count;
                result.guardAoa = GetRobustGuardAoa(effectiveAoaMagnitudes);
                result.hasDlcPanelAoa = true;
                result.dlcAoaSampleCount = aoaSamples;
                result.sampleCount = effectiveAoaMagnitudes.Count;
                result.source = "DLC Display AoA";
                dlcAoaReadDisplay += " 有效:" + effectiveAoaMagnitudes.Count.ToString(CultureInfo.InvariantCulture) + "/" + aoaSamples.ToString(CultureInfo.InvariantCulture);
            }
            else if (rawLiftSamples > 0)
            {
                result.sampleCount = rawLiftSamples;
                result.source = "DLC Runtime Forward Lift";
            }

            return result;
        }

        private bool TryReadForwardLiftKn(ModuleControlSurface blade, out float liftKn)
        {
            liftKn = 0f;
            if (blade == null || blade.part == null) return false;

            // The DLC blade PAW fields are not always stored on ModuleControlSurface itself;
            // on some Breaking Ground builds they live on another module on the blade part.
            // Therefore we scan the blade part modules, but only accept exact DLC-style
            // display fields and never scan the rotor or the CSP module.
            try
            {
                PartModule bestModule = null;
                BaseField bestField = null;
                float bestValue = 0f;
                int bestScore = -1;

                for (int m = 0; m < blade.part.Modules.Count; m++)
                {
                    PartModule module = blade.part.Modules[m];
                    if (module == null || module.Fields == null) continue;
                    if (module == this) continue;

                    foreach (BaseField field in module.Fields)
                    {
                        if (field == null) continue;
                        if (!LooksLikeForwardLiftField(field)) continue;

                        float candidate;
                        if (!TryGetFieldFloatRuntimeFirst(module, field, out candidate)) continue;
                        if (float.IsNaN(candidate) || float.IsInfinity(candidate)) continue;
                        if (Mathf.Abs(candidate) > 1000f) candidate *= 0.001f;
                        if (Mathf.Abs(candidate) > 200f) continue;

                        int score = ScoreDlcFieldCandidate(module, field, false);
                        if (score > bestScore)
                        {
                            bestScore = score;
                            bestField = field;
                            bestModule = module;
                            bestValue = candidate;
                        }
                    }
                }

                if (bestField != null && bestModule != null)
                {
                    liftKn = bestValue;
                    return true;
                }
            }
            catch (Exception)
            {
                liftKn = 0f;
            }
            return false;
        }

        private bool TryReadDlcAoaDeg(ModuleControlSurface blade, out float aoaDeg)
        {
            aoaDeg = 0f;
            if (blade == null || blade.part == null) return false;

            try
            {
                PartModule bestModule = null;
                BaseField bestField = null;
                float bestValue = 0f;
                int bestScore = -100000;
                string bestReadSource = "";
                string bestRawText = "";
                string bestGuiText = "";

                for (int m = 0; m < blade.part.Modules.Count; m++)
                {
                    PartModule module = blade.part.Modules[m];
                    if (module == null || module.Fields == null) continue;
                    if (module == this) continue;

                    foreach (BaseField field in module.Fields)
                    {
                        if (field == null) continue;

                        int score = ScoreDlcFieldCandidate(module, field, true);
                        if (score <= 0) continue;

                        float candidate;
                        string readSource;
                        string rawText;
                        string guiText;
                        if (!TryGetDlcAoaFloatDisplayFirst(module, field, out candidate, out readSource, out rawText, out guiText)) continue;
                        if (float.IsNaN(candidate) || float.IsInfinity(candidate)) continue;

                        // DLC PAW AoA is degrees.  Do not perform radian auto-conversion.
                        // Values around 50-70 deg are valid and must trigger stall recovery.
                        float candidateMagnitude = Mathf.Abs(candidate);
                        if (candidateMagnitude > 179f) continue;

                        // Important: a blade part can expose more than one field named "迎角".
                        // One is the ordinary control-surface/aero AoA, often around 0-5 deg;
                        // the DLC propeller PAW AoA can be 40-70 deg on the same part.  If we
                        // rank only by field name, the first "迎角" field wins and the stall
                        // guard reads a stuck small value.  Combine the field-name score with
                        // the actual readable AoA magnitude so the DLC propeller reading wins.
                        int readScore = ScoreDlcAoaReadCandidate(candidate, readSource);
                        int totalScore = score * 10 + readScore;

                        // Strongly prefer a non-zero display value.  In BG/KSP the raw value
                        // behind the PAW AoA field can stay at 0 while the visible PAW string
                        // shows the real DLC AoA.
                        if (candidateMagnitude > 0.05f) totalScore += 25;
                        if (readSource.StartsWith("GUI")) totalScore += 40;
                        if (candidateMagnitude >= Mathf.Max(8f, targetBladeAoa)) totalScore += 60;
                        if (candidateMagnitude >= Mathf.Max(10f, maxBladeAoa)) totalScore += 80;

                        if (totalScore > bestScore)
                        {
                            bestScore = totalScore;
                            bestField = field;
                            bestModule = module;
                            bestValue = candidate;
                            bestReadSource = readSource;
                            bestRawText = rawText;
                            bestGuiText = guiText;
                        }
                    }
                }

                if (bestField != null && bestModule != null)
                {
                    aoaDeg = bestValue;
                    dlcAoaFieldDisplay = SafeModuleFieldName(bestModule, bestField);
                    dlcAoaReadDisplay = bestReadSource + "=" + aoaDeg.ToString("F2", CultureInfo.InvariantCulture);
                    if (!string.IsNullOrEmpty(bestGuiText)) dlcAoaReadDisplay += " gui:" + TruncateForPaw(bestGuiText, 24);
                    if (!string.IsNullOrEmpty(bestRawText)) dlcAoaReadDisplay += " raw:" + TruncateForPaw(bestRawText, 18);
                    return true;
                }
            }
            catch (Exception e)
            {
                aoaDeg = 0f;
                dlcAoaReadDisplay = "异常:" + e.GetType().Name;
            }
            return false;
        }

        private int ScoreDlcFieldCandidate(PartModule module, BaseField field, bool isAoa)
        {
            string moduleName = module != null ? module.GetType().Name.ToLowerInvariant() : string.Empty;
            string gui = string.Empty;
            string internalName = string.Empty;
            try { gui = field.guiName ?? string.Empty; } catch (Exception) { gui = string.Empty; }
            try
            {
                PropertyInfo prop = field.GetType().GetProperty("name");
                object value = prop != null ? prop.GetValue(field, null) : null;
                internalName = value != null ? value.ToString() : string.Empty;
            }
            catch (Exception) { internalName = string.Empty; }

            string text = (gui + " " + internalName).ToLowerInvariant();
            int score = 0;

            // Prefer the blade control/aero modules over generic display/helper modules, but
            // do not require ModuleControlSurface because several BG builds expose PAW aero
            // values through a sibling module on the blade part.
            if (moduleName.Contains("controlsurface")) score += 35;
            if (moduleName.Contains("lifting") || moduleName.Contains("aero")) score += 20;
            if (moduleName.Contains("deploy")) score += 5;

            if (isAoa)
            {
                if (gui == "迎角" || gui == "攻角") score += 100;
                if (text.Contains("angle of attack")) score += 80;
                if (Regex.IsMatch(text, @"\baoa\b")) score += 70;
                if (text.Contains("迎角") || text.Contains("攻角")) score += 60;

                // These are CSP/UI/config fields, not DLC blade AoA fields.
                if (text.Contains("目标") || text.Contains("最大") || text.Contains("平均") || text.Contains("保护") || text.Contains("target") || text.Contains("max") || text.Contains("average") || text.Contains("guard")) score -= 200;
            }
            else
            {
                if (gui == "前进升力" || gui == "前向升力" || gui == "推进升力") score += 100;
                if (text.Contains("forward") && text.Contains("lift")) score += 80;
                if (text.Contains("前进升力") || text.Contains("前向升力") || text.Contains("推进升力")) score += 60;
            }

            return score;
        }

        private bool LooksLikeForwardLiftField(BaseField field)
        {
            string text = GetFieldSearchText(field);
            if (text.Contains("前进升力") || text.Contains("前向升力") || text.Contains("推进升力")) return true;
            if (text.Contains("forward") && text.Contains("lift")) return true;
            return false;
        }

        private bool LooksLikeDlcAoaField(BaseField field)
        {
            return ScoreDlcFieldCandidate(null, field, true) > 0;
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
                value = 0f;
            }
            return false;
        }

        private bool TryGetFieldFloatRuntimeFirst(PartModule module, BaseField field, out float value)
        {
            value = 0f;

            // Raw value is updated by the simulation even when the PAW is closed.  GUI string
            // is only a fallback because several KSP/BG fields are formatted lazily when the
            // right-click menu is open, which was the source of the "open PAW to fix it" bug.
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
                value = 0f;
            }
            return false;
        }

        private bool TryGetDlcAoaFloatDisplayFirst(PartModule module, BaseField field, out float value, out string source, out string rawText, out string guiText)
        {
            value = 0f;
            source = "None";
            rawText = string.Empty;
            guiText = string.Empty;

            // A DLC blade AoA BaseField can expose several numbers:
            // - GUI(false) may be the visible PAW AoA on one blade, but stale on another.
            // - GUI(true) can differ by sign/absolute formatting.
            // - RAW can be 0 on some builds, but valid on others.
            // Do not return the first readable number.  Evaluate all available readings and
            // pick the strongest plausible AoA value.  This fixes the common state where one
            // blade reports ~50 deg while sibling blades report stale ~0-5 deg and hide stall.
            float bestValue = 0f;
            string bestSource = "None";
            string bestGuiText = string.Empty;
            string bestRawText = string.Empty;
            int bestScore = -100000;

            float candidate;
            string text;
            if (TryGetFieldGuiFloat(module, field, false, out candidate, out text))
            {
                int score = ScoreDlcAoaReadCandidate(candidate, "GUI(false)");
                if (score > bestScore)
                {
                    bestScore = score;
                    bestValue = candidate;
                    bestSource = "GUI(false)";
                    bestGuiText = text;
                }
            }

            if (TryGetFieldGuiFloat(module, field, true, out candidate, out text))
            {
                int score = ScoreDlcAoaReadCandidate(candidate, "GUI(true)");
                if (score > bestScore)
                {
                    bestScore = score;
                    bestValue = candidate;
                    bestSource = "GUI(true)";
                    bestGuiText = text;
                }
            }

            try
            {
                object raw = field.GetValue(module);
                rawText = raw != null ? raw.ToString() : string.Empty;
                if (TryConvertToFloat(raw, out candidate))
                {
                    int score = ScoreDlcAoaReadCandidate(candidate, "RAW");
                    if (score > bestScore)
                    {
                        bestScore = score;
                        bestValue = candidate;
                        bestSource = "RAW";
                        bestRawText = rawText;
                    }
                }
            }
            catch (Exception)
            {
            }

            if (bestScore <= -10000) return false;
            value = bestValue;
            source = bestSource;
            if (string.IsNullOrEmpty(guiText)) guiText = bestGuiText;
            if (string.IsNullOrEmpty(rawText)) rawText = bestRawText;
            return true;
        }

        private int ScoreDlcAoaReadCandidate(float candidate, string source)
        {
            if (float.IsNaN(candidate) || float.IsInfinity(candidate)) return -100000;
            float magnitude = Mathf.Abs(candidate);
            if (magnitude > 179f) return -100000;

            int score = 0;
            if (source != null && source.StartsWith("GUI")) score += 25;
            if (source == "RAW") score += 5;
            if (magnitude < 0.01f) score -= 30;
            if (magnitude > 0.05f) score += 10;

            // Prefer the value that can actually protect the propeller.  A 3-5 deg stale
            // GUI string must not beat a simultaneously available 45-60 deg DLC AoA value.
            score += Mathf.RoundToInt(Mathf.Clamp(magnitude * 2f, 0f, 140f));
            if (magnitude >= Mathf.Max(6f, targetBladeAoa)) score += 25;
            if (magnitude >= Mathf.Max(8f, maxBladeAoa)) score += 25;
            return score;
        }

        private bool TryGetFieldGuiFloat(PartModule module, BaseField field, bool absolute, out float value, out string guiText)
        {
            value = 0f;
            guiText = string.Empty;
            try
            {
                guiText = field.GetStringValue(module, absolute);
                if (string.IsNullOrEmpty(guiText)) return false;

                // Some fields display placeholders while closed; do not accept text without
                // an actual number.  Unit symbols such as "°" or localized suffixes are OK.
                if (!Regex.IsMatch(guiText, @"[-+]?\d")) return false;
                return TryConvertToFloat(guiText, out value);
            }
            catch (Exception)
            {
                value = 0f;
                guiText = string.Empty;
                return false;
            }
        }

        private string SafeModuleFieldName(PartModule module, BaseField field)
        {
            string moduleName = module != null ? module.GetType().Name : "null";
            string gui = string.Empty;
            string internalName = string.Empty;
            try { gui = field != null && field.guiName != null ? field.guiName : string.Empty; } catch (Exception) { gui = string.Empty; }
            try
            {
                PropertyInfo prop = field != null ? field.GetType().GetProperty("name") : null;
                object value = prop != null ? prop.GetValue(field, null) : null;
                internalName = value != null ? value.ToString() : string.Empty;
            }
            catch (Exception) { internalName = string.Empty; }
            string name = moduleName + ":" + (string.IsNullOrEmpty(gui) ? internalName : gui);
            return TruncateForPaw(name, 42);
        }

        private string TruncateForPaw(string text, int maxLen)
        {
            if (string.IsNullOrEmpty(text)) return string.Empty;
            if (text.Length <= maxLen) return text;
            return text.Substring(0, Mathf.Max(0, maxLen - 1)) + "…";
        }

        private bool TryGetFieldFloatPreferGuiString(PartModule module, BaseField field, out float value)
        {
            value = 0f;
            string rawText;
            string guiText;
            string source;
            return TryGetDlcAoaFloatDisplayFirst(module, field, out value, out source, out rawText, out guiText);
        }

        private void UpdateAoaNormalizationSign(float rawAverageAoa)
        {
            if (Mathf.Abs(rawAverageAoa) < 0.5f || GetCurrentRotorRpm() < 30f)
            {
                return;
            }

            // Counter-rotating/mirrored BG blades can expose the same physical positive
            // blade AoA with the opposite sign.  Normalize the sign only after it is stable;
            // this preserves signed-positive stall protection while avoiding a user-facing
            // reverse-AoA option.
            if (rawAverageAoa > 0.5f)
            {
                aoaPositiveStableSamples++;
                aoaNegativeStableSamples = 0;
                if (aoaPositiveStableSamples >= 3) aoaNormalizationSign = 1f;
            }
            else if (rawAverageAoa < -0.5f)
            {
                aoaNegativeStableSamples++;
                aoaPositiveStableSamples = 0;
                if (aoaNegativeStableSamples >= 3) aoaNormalizationSign = -1f;
            }
        }

        private List<float> BuildEffectiveDlcAoaMagnitudes(List<float> rawAoaValues)
        {
            List<float> magnitudes = new List<float>();
            if (rawAoaValues == null) return magnitudes;

            float maxMagnitude = 0f;
            for (int i = 0; i < rawAoaValues.Count; i++)
            {
                float magnitude = Mathf.Abs(rawAoaValues[i]);
                if (float.IsNaN(magnitude) || float.IsInfinity(magnitude) || magnitude > 179f) continue;
                magnitudes.Add(magnitude);
                if (magnitude > maxMagnitude) maxMagnitude = magnitude;
            }

            if (magnitudes.Count <= 1) return magnitudes;

            // If at least one blade reports a high AoA, low sibling readings are almost
            // certainly stale PAW/BaseField dropouts.  Do not average them into the guard.
            float highAoaGate = Mathf.Max(8f, Mathf.Min(maxBladeAoa, targetBladeAoa + 2f));
            if (maxMagnitude < highAoaGate) return magnitudes;

            float keepThreshold = Mathf.Max(maxMagnitude * 0.45f, Mathf.Max(4f, targetBladeAoa * 0.75f));
            List<float> kept = new List<float>();
            for (int i = 0; i < magnitudes.Count; i++)
            {
                if (magnitudes[i] >= keepThreshold) kept.Add(magnitudes[i]);
            }

            return kept.Count > 0 ? kept : magnitudes;
        }

        private float GetRobustGuardAoa(List<float> aoaValues)
        {
            if (aoaValues == null || aoaValues.Count == 0) return 0f;

            // aoaValues are already effective magnitudes.  Use the maximum for protection.
            // The previous second-high rule failed when two sibling blade fields were stale
            // low values and only one blade exposed the real DLC AoA.
            float maxMagnitude = 0f;
            for (int i = 0; i < aoaValues.Count; i++)
            {
                float magnitude = Mathf.Abs(aoaValues[i]);
                if (float.IsNaN(magnitude) || float.IsInfinity(magnitude) || magnitude > 179f) continue;
                if (magnitude > maxMagnitude) maxMagnitude = magnitude;
            }
            return maxMagnitude;
        }

        private bool TryConvertToFloat(object raw, out float value)
        {
            value = 0f;
            if (raw == null) return false;
            try
            {
                if (raw is float) { value = (float)raw; return true; }
                if (raw is double) { value = (float)(double)raw; return true; }
                if (raw is int) { value = (int)raw; return true; }
                if (raw is long) { value = (long)raw; return true; }
                string text = raw.ToString();
                if (string.IsNullOrEmpty(text)) return false;
                Match match = Regex.Match(text, @"[-+]?\d+(?:[\.,]\d+)?");
                if (!match.Success) return false;
                string number = match.Value.Replace(',', '.');
                return float.TryParse(number, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
            }
            catch (Exception)
            {
                value = 0f;
                return false;
            }
        }
        private void InitializePitchFromBlades()
        {
            RefreshBlades();
            if (blades.Count > 0)
            {
                float sum = 0f;
                for (int i = 0; i < blades.Count; i++)
                {
                    sum += blades[i].deployAngle;
                }
                collectivePitch = sum / blades.Count;
            }
            else
            {
                collectivePitch = Mathf.Clamp(minPitch, Mathf.Min(minPitch, maxPitch), Mathf.Max(minPitch, maxPitch));
            }
            commandedPitchDisplay = collectivePitch;
            filteredOutputPitch = collectivePitch;
            pitchOutputFilterInitialized = true;
            initializedPitch = true;
        }

        private void RefreshBlades()
        {
            blades.Clear();
            if (part == null || part.children == null)
            {
                bladeCountDisplay = 0;
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
        }

        private void ApplyPitchToBlades(float requestedPitch)
        {
            foreach (ModuleControlSurface blade in blades)
            {
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

                blade.deployAngle = Mathf.Clamp(requestedPitch + commandPitchOffset, lower, upper);
            }
        }

        private static string CleanPitchSyncGroupName(string name)
        {
            if (string.IsNullOrEmpty(name)) return string.Empty;

            string cleaned = name.Trim();
            if (cleaned.Length == 0) return string.Empty;

            cleaned = cleaned.Replace('\r', ' ').Replace('\n', ' ').Replace('\t', ' ');
            while (cleaned.Contains("  ")) cleaned = cleaned.Replace("  ", " ");
            if (cleaned.Length > 32) cleaned = cleaned.Substring(0, 32);
            return cleaned;
        }

        private bool IsSamePitchSyncGroup(ModuleConstantSpeedProp other, string groupName)
        {
            if (other == null || other == this) return false;
            if (!other.pitchSyncEnabled || other.feather) return false;
            string otherGroup = CleanPitchSyncGroupName(other.pitchSyncGroup);
            if (string.IsNullOrEmpty(otherGroup)) return false;
            return string.Equals(otherGroup, groupName, StringComparison.OrdinalIgnoreCase);
        }

        private bool TryGetCachedLocalResult(double currentUt, out float pitch)
        {
            pitch = lastLocalResultPitch;
            if (lastLocalResultUt < 0.0) return false;
            if (currentUt - lastLocalResultUt > 2.0) return false;
            return true;
        }

        private bool TryGetCachedProtectedResult(double currentUt, out float pitch)
        {
            pitch = lastLocalProtectionPitch;
            if (!lastLocalProtectionActive) return false;
            if (lastLocalProtectionUt < 0.0) return false;
            if (currentUt - lastLocalProtectionUt > 2.0) return false;
            return true;
        }

        private string GetPitchSyncSnapshotKey(string groupName)
        {
            Vessel v = vessel != null ? vessel : (part != null ? part.vessel : null);
            string vesselKey = v != null ? v.id.ToString() : "no-vessel";
            return vesselKey + "|" + groupName.ToLowerInvariant();
        }

        private bool TryGetPitchSyncSnapshot(string groupName, double ut, out PitchSyncSnapshot snapshot)
        {
            string key = GetPitchSyncSnapshotKey(groupName);
            if (PitchSyncSnapshots.TryGetValue(key, out snapshot) && Math.Abs(snapshot.ut - ut) <= 0.000001)
            {
                return true;
            }
            snapshot = new PitchSyncSnapshot();
            return false;
        }

        private void StorePitchSyncSnapshot(string groupName, PitchSyncSnapshot snapshot)
        {
            PitchSyncSnapshots[GetPitchSyncSnapshotKey(groupName)] = snapshot;

            if (PitchSyncSnapshots.Count <= 64) return;

            List<string> staleKeys = new List<string>();
            foreach (KeyValuePair<string, PitchSyncSnapshot> item in PitchSyncSnapshots)
            {
                if (snapshot.ut - item.Value.ut > 5.0) staleKeys.Add(item.Key);
            }
            for (int i = 0; i < staleKeys.Count; i++) PitchSyncSnapshots.Remove(staleKeys[i]);
        }

        private float ApplyPitchSynchronization(float localPitch, float lowerPitch, float upperPitch, double ut)
        {
            string groupName = CleanPitchSyncGroupName(pitchSyncGroup);
            pitchSyncGroup = groupName;

            if (!pitchSyncEnabled || feather || !HighLogic.LoadedSceneIsFlight)
            {
                pitchSyncStateDisplay = pitchSyncEnabled ? "等待飞行" : "关";
                pitchSyncMemberCountDisplay = 0;
                pitchSyncReferencePitchDisplay = localPitch;
                pitchSyncErrorDisplay = 0f;
                return localPitch;
            }

            if (string.IsNullOrEmpty(groupName))
            {
                pitchSyncStateDisplay = "未分组";
                pitchSyncMemberCountDisplay = 1;
                pitchSyncReferencePitchDisplay = localPitch;
                pitchSyncErrorDisplay = 0f;
                return localPitch;
            }

            if (vessel == null || vessel.parts == null)
            {
                pitchSyncStateDisplay = "等待载具";
                pitchSyncMemberCountDisplay = 1;
                pitchSyncReferencePitchDisplay = localPitch;
                pitchSyncErrorDisplay = 0f;
                return localPitch;
            }

            PitchSyncSnapshot snapshot;
            if (!TryGetPitchSyncSnapshot(groupName, ut, out snapshot))
            {
                float sum = 0f;
                int members = 0;
                bool anyProtectionActive = false;
                float protectedPitchMin = upperPitch;

                float previousPitch;
                if (TryGetCachedLocalResult(ut, out previousPitch))
                {
                    sum += Mathf.Clamp(previousPitch, lowerPitch, upperPitch);
                    members++;

                    float protectedPitch;
                    if (TryGetCachedProtectedResult(ut, out protectedPitch))
                    {
                        anyProtectionActive = true;
                        protectedPitchMin = Mathf.Min(protectedPitchMin, Mathf.Clamp(protectedPitch, lowerPitch, upperPitch));
                    }
                }

                for (int pIndex = 0; pIndex < vessel.parts.Count; pIndex++)
                {
                    Part vesselPart = vessel.parts[pIndex];
                    if (vesselPart == null || vesselPart.Modules == null) continue;

                    for (int mIndex = 0; mIndex < vesselPart.Modules.Count; mIndex++)
                    {
                        ModuleConstantSpeedProp other = vesselPart.Modules[mIndex] as ModuleConstantSpeedProp;
                        if (!IsSamePitchSyncGroup(other, groupName)) continue;

                        float otherPitch;
                        if (!other.TryGetCachedLocalResult(ut, out otherPitch)) continue;

                        sum += Mathf.Clamp(otherPitch, lowerPitch, upperPitch);
                        members++;

                        float otherProtectedPitch;
                        if (other.TryGetCachedProtectedResult(ut, out otherProtectedPitch))
                        {
                            anyProtectionActive = true;
                            protectedPitchMin = Mathf.Min(protectedPitchMin, Mathf.Clamp(otherProtectedPitch, lowerPitch, upperPitch));
                        }
                    }
                }

                if (members >= 2)
                {
                    float averagePitch = Mathf.Clamp(sum / members, lowerPitch, upperPitch);
                    snapshot = new PitchSyncSnapshot
                    {
                        ut = ut,
                        referencePitch = anyProtectionActive ? Mathf.Min(averagePitch, protectedPitchMin) : averagePitch,
                        members = members,
                        anyProtectionActive = anyProtectionActive
                    };
                }
                else
                {
                    snapshot = new PitchSyncSnapshot
                    {
                        ut = ut,
                        referencePitch = Mathf.Clamp(localPitch, lowerPitch, upperPitch),
                        members = members,
                        anyProtectionActive = false
                    };
                }
                StorePitchSyncSnapshot(groupName, snapshot);
            }

            pitchSyncMemberCountDisplay = snapshot.members;

            if (snapshot.members < 2)
            {
                pitchSyncStateDisplay = "等待配对";
                pitchSyncReferencePitchDisplay = localPitch;
                pitchSyncErrorDisplay = 0f;
                return localPitch;
            }

            float syncedPitch = Mathf.Clamp(snapshot.referencePitch, lowerPitch, upperPitch);
            pitchSyncReferencePitchDisplay = syncedPitch;
            pitchSyncErrorDisplay = syncedPitch - localPitch;
            if (snapshot.anyProtectionActive)
            {
                pitchSyncStateDisplay = "保护同步";
            }
            else
            {
                pitchSyncStateDisplay = Mathf.Abs(pitchSyncErrorDisplay) <= Mathf.Max(0f, pitchSyncDeadband) ? "同步" : "结果同步";
            }
            return syncedPitch;
        }

        private static float MoveTowardWithRate(float current, float target, float maxRate, float dt)
        {
            float maxDelta = Mathf.Abs(maxRate) * dt;
            return Mathf.MoveTowards(current, target, maxDelta);
        }

        private static float LowPass(float previous, float current, float tau, float dt)
        {
            if (tau <= 0f) return current;
            float alpha = 1f - Mathf.Exp(-Mathf.Max(0f, dt) / Mathf.Max(0.001f, tau));
            return Mathf.Lerp(previous, current, Mathf.Clamp01(alpha));
        }
    }
}
