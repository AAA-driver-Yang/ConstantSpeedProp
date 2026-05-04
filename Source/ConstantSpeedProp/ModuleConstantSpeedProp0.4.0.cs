using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text.RegularExpressions;
using Expansions.Serenity;
using UnityEngine;

namespace ConstantSpeedProp
{
    /// <summary>
    /// KSP1 / Breaking Ground fast constant-speed propeller governor.
    ///
    /// 0.4.4 CN UI: compact main controls with foldable advanced/debug controls. Forward Lift Guard removed.
    ///
    /// 0.4.0 design goal:
    /// - Keep RPM as the primary control objective because rotor RPM determines available shaft power.
    /// - Remove takeoff pitch schedule and low-throttle pitch hold protections.
    /// - Prefer KSP/DLC's own blade-panel AoA value when available, because vector-derived AoA can flip sign on mirrored/clockwise props.
    /// - Keep signed positive-AoA stall recovery through AoA guard only.
    ///
    /// This module attaches to a ModuleRoboticServoRotor and controls directly attached ModuleControlSurface blades.
    /// </summary>
    public class ModuleConstantSpeedProp : PartModule
    {
        private const string VersionText = "0.4.4 中文高级折叠-无前向升力保护";

        private ModuleRoboticServoRotor rotor;
        private readonly List<ModuleControlSurface> blades = new List<ModuleControlSurface>();

        private float collectivePitch;
        private float integralError;
        private float previousError;
        private bool initializedPitch;
        private float nextBladeRefreshTime;
        private double lastGovernorUniversalTime = -1.0;
        private float lastGovernorRealtime = -1f;
        private bool lastShowAdvancedOptionsState;

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
        private bool pitchOutputFilterInitialized;
        private float filteredOutputPitch;

        private struct AeroData
        {
            public bool valid;
            public float signedAverageAoa;
            public float guardAoa;
            public float averageBladeSpeed;
            public int sampleCount;
            public bool hasDlcPanelAoa;
            public int dlcAoaSampleCount;
            public string source;
        }

        private struct BladeAeroSample
        {
            public bool valid;
            public float signedAoa;
            public float guardAoa;
            public float bladeSpeed;
        }

        private Vector3 RotorAxis
        {
            get
            {
                if (rotor == null)
                    return Vector3.zero;

                try
                {
                    return rotor.transform.TransformVector(rotor.GetMainAxis()).normalized;
                }
                catch (Exception)
                {
                    return part != null ? part.transform.up.normalized : Vector3.zero;
                }
            }
        }

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "定速螺旋桨")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool governorEnabled = true;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "版本")]
        public string versionDisplay = VersionText;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "高级选项")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "收起", enabledText = "展开", affectSymCounterparts = UI_Scene.All)]
        public bool showAdvancedOptions = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "目标转速", guiUnits = " rpm")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1000f, stepIncrement = 10f, affectSymCounterparts = UI_Scene.All)]
        public float targetRpm = 450f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "油门控制转速")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "否", enabledText = "是", affectSymCounterparts = UI_Scene.All)]
        public bool throttleSetsTargetRpm = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "怠速转速", guiUnits = " rpm")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1000f, stepIncrement = 10f, affectSymCounterparts = UI_Scene.All)]
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

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "使用DLC气动数据")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "否", enabledText = "是", affectSymCounterparts = UI_Scene.All)]
        public bool useDlcAeroData = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "使用DLC迎角")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "否", enabledText = "是", affectSymCounterparts = UI_Scene.All)]
        public bool useDlcPanelAoa = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "反转气流方向")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool invertAeroFlow = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "反转迎角符号")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "关", enabledText = "开", affectSymCounterparts = UI_Scene.All)]
        public bool invertMeasuredAoaSign = false;

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

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "最小叶片测速", guiUnits = " m/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 60f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float minMeasuredBladeSpeed = 10f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "桨距命令偏移", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -30f, maxValue = 30f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float commandPitchOffset = 0f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "后备叶片半径", guiUnits = " m")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0.1f, maxValue = 10f, stepIncrement = 0.05f, affectSymCounterparts = UI_Scene.All)]
        public float effectiveBladeRadius = 1.25f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "后备最小来流", guiUnits = " m/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 20f, stepIncrement = 0.5f, affectSymCounterparts = UI_Scene.All)]
        public float minimumInflow = 0f;

        [KSPField(isPersistant = false, guiActive = true, guiName = "当前转速", guiUnits = " rpm", guiFormat = "F0")]
        public float currentRpmDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "转速误差", guiUnits = " rpm", guiFormat = "F0")]
        public float rpmErrorDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "当前桨距", guiUnits = " deg", guiFormat = "F1")]
        public float commandedPitchDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "桨距速度命令", guiUnits = " deg/s", guiFormat = "F1")]
        public float pitchRateCommandDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "桨距上限", guiUnits = " deg", guiFormat = "F1")]
        public float pitchLimitDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "平均迎角", guiUnits = " deg", guiFormat = "F1")]
        public float estimatedAoaDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "保护迎角", guiUnits = " deg", guiFormat = "F1")]
        public float guardAoaDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "主油门", guiUnits = " %", guiFormat = "F0")]
        public float mainThrottleDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "迎角来源")]
        public string aoaSourceDisplay = "无";

        [KSPField(isPersistant = false, guiActive = true, guiName = "DLC迎角样本")]
        public int dlcAoaSampleCountDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "叶片速度", guiUnits = " m/s", guiFormat = "F1")]
        public float measuredBladeSpeedDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "轴向空速", guiUnits = " m/s", guiFormat = "F1")]
        public float axialAirspeedDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "气动样本数")]
        public int aeroSampleCountDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "气动采样状态")]
        public string aeroSampleHoldDisplay = "否";

        [KSPField(isPersistant = false, guiActive = true, guiName = "循环来源")]
        public string loopSourceDisplay = "未运行";

        [KSPField(isPersistant = false, guiActive = true, guiName = "桨叶数量")]
        public int bladeCountDisplay;

        private static readonly string[] compactVisibleFields =
        {
            // Main controls
            "governorEnabled",
            "targetRpm",
            "throttleSetsTargetRpm",
            "idleRpm",
            "minPitch",
            "maxPitch",
            "pitchRateLimit",
            "pitchOutputFilterTau",
            "rpmDeadband",
            "proportionalGain",
            "integralGain",
            "feather",
            "featherPitch",

            // Stall protection controls
            "antiStallEnabled",
            "targetBladeAoa",
            "maxBladeAoa",
            "aoaPitchIncreaseLimit",
            "aoaRecoveryRate",

            // Sampling and smoothing controls
            "aeroSampleIntervalFrames",
            "rpmFilterTau",
            "aeroFilterTau",

            // Compact flight readout
            "versionDisplay",
            "showAdvancedOptions",
            "currentRpmDisplay",
            "rpmErrorDisplay",
            "commandedPitchDisplay",
            "pitchRateCommandDisplay",
            "guardAoaDisplay",
            "aoaSourceDisplay",
            "aeroSampleHoldDisplay",
            "bladeCountDisplay"
        };

        private static readonly string[] foldedAdvancedFields =
        {
            "minimumCorrectionRate",
            "derivativeGain",
            "useDlcAeroData",
            "useDlcPanelAoa",
            "invertAeroFlow",
            "invertMeasuredAoaSign",
            "minMeasuredBladeSpeed",
            "commandPitchOffset",
            "effectiveBladeRadius",
            "minimumInflow",
            "pitchLimitDisplay",
            "estimatedAoaDisplay",
            "mainThrottleDisplay",
            "dlcAoaSampleCountDisplay",
            "measuredBladeSpeedDisplay",
            "axialAirspeedDisplay",
            "aeroSampleCountDisplay",
            "loopSourceDisplay"
        };

        private void SetFieldVisible(string fieldName, bool visible)
        {
            BaseField field = null;
            try
            {
                field = Fields[fieldName];
            }
            catch (Exception)
            {
                field = null;
            }

            if (field == null)
                return;

            field.guiActive = visible;
            field.guiActiveEditor = visible;
        }

        private void ForceFieldVisibility()
        {
            foreach (string fieldName in foldedAdvancedFields)
                SetFieldVisible(fieldName, showAdvancedOptions);

            foreach (string fieldName in compactVisibleFields)
                SetFieldVisible(fieldName, true);

            lastShowAdvancedOptionsState = showAdvancedOptions;
        }

        private void UpdateFoldedFieldVisibilityIfNeeded()
        {
            if (lastShowAdvancedOptionsState != showAdvancedOptions)
                ForceFieldVisibility();
        }

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

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            versionDisplay = VersionText;
            nextBladeRefreshTime = 0f;
            initializedPitch = false;
            ResetRuntimeFilters();
        }

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            versionDisplay = VersionText;
            ForceFieldVisibility();
            nextBladeRefreshTime = 0f;
            initializedPitch = false;
            ResetRuntimeFilters();
        }

        public override void OnStartFinished(StartState state)
        {
            base.OnStartFinished(state);
            versionDisplay = VersionText;
            ForceFieldVisibility();

            rotor = part.FindModuleImplementing<ModuleRoboticServoRotor>();
            if (rotor == null)
            {
                Debug.LogError("[ConstantSpeedProp] Module must be added to a Breaking Ground rotor part.");
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
            UpdateFoldedFieldVisibilityIfNeeded();

            // v3.8 watchdog: some KSP/robotics setups do not reliably execute the physics callback
            // until the PAW has been opened. If neither fixed callback has run recently, run once
            // from OnUpdate so the governor wakes up without the player right-clicking the rotor.
            if (!HighLogic.LoadedSceneIsFlight || FlightDriver.Pause)
                return;

            double ut = Planetarium.GetUniversalTime();
            double maxDelay = Math.Max(0.05, TimeWarp.fixedDeltaTime * 1.5);
            if (lastGovernorUniversalTime < 0.0 || ut - lastGovernorUniversalTime > maxDelay)
                RunGovernorLoop("OnUpdateWatchdog", Mathf.Max(Time.deltaTime, TimeWarp.fixedDeltaTime));
        }

        public override void OnFixedUpdate()
        {
            base.OnFixedUpdate();
            RunGovernorLoop("KSPOnFixedUpdate", TimeWarp.fixedDeltaTime);
        }

        public void FixedUpdate()
        {
            // Keep the old Unity callback too. Some KSP installs/mod stacks are more reliable with this path.
            RunGovernorLoop("UnityFixedUpdate", TimeWarp.fixedDeltaTime);
        }

        private string LocalizeAoaSource(string source)
        {
            if (string.IsNullOrEmpty(source))
                return "无";

            string text = source;
            text = text.Replace("DLC Panel AoA", "DLC迎角");
            text = text.Replace("Vector AoA", "矢量迎角");
            text = text.Replace("Fallback", "后备估算");
            text = text.Replace("Filtered", "滤波");
            text = text.Replace("None", "无");
            return text;
        }

        private void RunGovernorLoop(string source, float dt)
        {
            if (!HighLogic.LoadedSceneIsFlight || FlightDriver.Pause)
                return;

            if (dt <= 0f)
                dt = Mathf.Max(TimeWarp.fixedDeltaTime, 0.02f);

            double ut = Planetarium.GetUniversalTime();
            // Avoid running twice in the same physics instant when both Unity FixedUpdate and KSP OnFixedUpdate fire.
            if (lastGovernorUniversalTime >= 0.0 && Math.Abs(ut - lastGovernorUniversalTime) < 0.000001)
                return;

            if (!EnsureRuntimeReady())
                return;

            lastGovernorUniversalTime = ut;
            lastGovernorRealtime = Time.realtimeSinceStartup;
            loopSourceDisplay = source;
            physicsFrameCounter++;

            float rawRpm = GetCurrentRotorRpm();
            currentRpmDisplay = GetFilteredRpm(rawRpm, dt);
            bladeCountDisplay = blades.Count;

            if (!governorEnabled || blades.Count == 0)
                return;

            if (!initializedPitch)
                InitializePitchFromBlades();

            float lowerPitch = Mathf.Min(minPitch, maxPitch);
            float upperPitch = Mathf.Max(minPitch, maxPitch);
            float wantedRpm = GetWantedRpm();
            AeroData aero = GetAeroDataDecimated(wantedRpm, dt);

            pitchLimitDisplay = upperPitch;
            estimatedAoaDisplay = aero.signedAverageAoa;
            guardAoaDisplay = aero.guardAoa;
            measuredBladeSpeedDisplay = aero.averageBladeSpeed;
            mainThrottleDisplay = GetMainThrottlePercent();
            aoaSourceDisplay = LocalizeAoaSource(aero.source);
            dlcAoaSampleCountDisplay = aero.dlcAoaSampleCount;
            aeroSampleCountDisplay = aero.sampleCount;

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
            }
            else
            {
                float rpmError = currentRpmDisplay - wantedRpm;
                rpmErrorDisplay = rpmError;
                requestedPitchRate = ComputeGovernorPitchRate(rpmError, dt);

                if (antiStallEnabled && aero.valid)
                {
                    ApplyStallGuard(ref requestedPitchRate, aero);
                }

                requestedPitchRate = Mathf.Clamp(requestedPitchRate, -Mathf.Abs(pitchRateLimit), Mathf.Abs(pitchRateLimit));

                // Anti-windup at physical pitch stops. No takeoff cap. No throttle hold. Only min/max pitch.
                if (collectivePitch >= upperPitch - 0.1f && requestedPitchRate > 0f)
                {
                    requestedPitchRate = 0f;
                    integralError = Mathf.Min(0f, integralError);
                }
                if (collectivePitch <= lowerPitch + 0.1f && requestedPitchRate < 0f)
                {
                    requestedPitchRate = 0f;
                    integralError = Mathf.Max(0f, integralError);
                }

                collectivePitch += requestedPitchRate * dt;
                collectivePitch = Mathf.Clamp(collectivePitch, lowerPitch, upperPitch);
                pitchToApply = SmoothOutputPitch(collectivePitch, dt, lowerPitch, upperPitch);
                collectivePitch = pitchToApply;
            }

            pitchRateCommandDisplay = requestedPitchRate;
            ApplyPitchToBlades(pitchToApply);
            commandedPitchDisplay = pitchToApply;
        }

        private bool EnsureRuntimeReady()
        {
            if (part == null)
                return false;

            if (rotor == null)
                rotor = part.FindModuleImplementing<ModuleRoboticServoRotor>();

            if (rotor == null)
                return false;

            if (Planetarium.GetUniversalTime() >= nextBladeRefreshTime || blades.Count == 0)
            {
                RefreshBlades();
                nextBladeRefreshTime = (float)Planetarium.GetUniversalTime() + 0.5f;
            }

            if (!initializedPitch && blades.Count > 0)
                InitializePitchFromBlades();

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
            if (!raw.valid)
                return raw;

            if (!aeroFilterInitialized)
            {
                filteredSignedAoa = raw.signedAverageAoa;
                filteredGuardAoa = raw.guardAoa;
                filteredBladeSpeed = raw.averageBladeSpeed;
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

            if (!string.IsNullOrEmpty(result.source) && !result.source.Contains("Filtered"))
                result.source = result.source + " Filtered";

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
                // Inside the deadband, avoid forcing two nearly identical rotors into different behavior.
                // Slowly bleed the integral toward zero so the controller remains sync-friendly.
                integralError = Mathf.Lerp(integralError, 0f, Mathf.Clamp01(dt * 1.5f));
                previousError = rpmError;
                return 0f;
            }

            integralError = Mathf.Clamp(integralError + rpmError * dt, -500f, 500f);
            float derivative = (rpmError - previousError) / dt;
            previousError = rpmError;

            float rate =
                proportionalGain * rpmError +
                integralGain * integralError +
                derivativeGain * derivative;

            // Immediate response floor: once RPM is outside the deadband, pitch must visibly move.
            // This is what replaces the old delayed takeoff schedule/low-throttle hold behavior.
            float floor = Mathf.Abs(minimumCorrectionRate);
            float floorStartError = Mathf.Max(Mathf.Max(0f, rpmDeadband) * 3f, 30f);
            if (floor > 0f && Mathf.Abs(rpmError) >= floorStartError)
            {
                if (rate > 0f)
                    rate = Mathf.Max(rate, floor);
                else if (rate < 0f)
                    rate = Mathf.Min(rate, -floor);
                else
                    rate = Mathf.Sign(rpmError) * floor;
            }

            return rate;
        }

        private float GetMainThrottlePercent()
        {
            float throttle = FlightInputHandler.state != null ? FlightInputHandler.state.mainThrottle : 0f;
            return Mathf.Clamp01(throttle) * 100f;
        }

        private void ApplyStallGuard(ref float requestedPitchRate, AeroData aero)
        {

            float guardAoa = aero.guardAoa;
            float targetAoa = Mathf.Min(targetBladeAoa, maxBladeAoa - 0.5f);

            if (guardAoa >= maxBladeAoa)
            {
                // Signed high-positive AoA means coarse-pitch stall risk. Move toward fine pitch.
                float severity = Mathf.Clamp01((guardAoa - maxBladeAoa) / 8f + 0.35f);
                requestedPitchRate = Mathf.Min(requestedPitchRate, -Mathf.Abs(aoaRecoveryRate) * severity);
                integralError = 0f;
                return;
            }

            if (guardAoa >= targetAoa && requestedPitchRate > aoaPitchIncreaseLimit)
            {
                // Near positive-AoA stall region: keep RPM authority, but avoid instant coarse-pitch jumps.
                requestedPitchRate = Mathf.Min(requestedPitchRate, Mathf.Max(0f, aoaPitchIncreaseLimit));
                integralError = Mathf.Min(integralError, 0f);
            }
        }

        private float GetWantedRpm()
        {
            if (!throttleSetsTargetRpm)
                return Mathf.Max(0f, targetRpm);

            float throttle = FlightInputHandler.state != null ? FlightInputHandler.state.mainThrottle : 0f;
            return Mathf.Lerp(Mathf.Max(0f, idleRpm), Mathf.Max(0f, targetRpm), Mathf.Clamp01(throttle));
        }

        private float GetCurrentRotorRpm()
        {
            float rpm = 0f;
            try
            {
                if (rotor != null)
                    rpm = Mathf.Abs(rotor.currentRPM);
            }
            catch (Exception)
            {
                rpm = 0f;
            }

            // Some Breaking Ground robotics setups leave currentRPM at zero until the PAW/UI has refreshed.
            // In that case estimate RPM from blade point velocity so the governor can start immediately.
            if (rpm > 0.25f)
                return rpm;

            if (TryEstimateRotorRpmFromBladeMotion(out float estimatedRpm))
                return estimatedRpm;

            return rpm;
        }

        private bool TryEstimateRotorRpmFromBladeMotion(out float rpm)
        {
            rpm = 0f;
            if (rotor == null || part == null || part.Rigidbody == null || blades.Count == 0)
                return false;

            try
            {
                Vector3 axis = RotorAxis;
                if (axis.sqrMagnitude < 0.1f)
                    return false;

                Vector3 origin = rotor.transform.position;
                Vector3 hubVelocity = part.Rigidbody.GetPointVelocity(origin);
                float rpmSum = 0f;
                int samples = 0;

                foreach (ModuleControlSurface blade in blades)
                {
                    if (blade == null || blade.part == null || blade.part.Rigidbody == null)
                        continue;

                    Vector3 refPoint = GetSurfaceReferencePoint(blade);
                    Vector3 radiusVector = Vector3.ProjectOnPlane(refPoint - origin, axis);
                    float radius = radiusVector.magnitude;
                    if (radius < 0.05f)
                        continue;

                    Vector3 relativeVelocity = blade.part.Rigidbody.GetPointVelocity(refPoint) - hubVelocity;
                    float tangentialSpeed = Vector3.ProjectOnPlane(relativeVelocity, axis).magnitude;
                    if (tangentialSpeed < 0.05f)
                        continue;

                    float omega = tangentialSpeed / radius;
                    rpmSum += omega * 60f / (2f * Mathf.PI);
                    samples++;
                }

                if (samples <= 0)
                    return false;

                rpm = rpmSum / samples;
                return rpm > 0.25f;
            }
            catch (Exception)
            {
                rpm = 0f;
                return false;
            }
        }

        private AeroData GetAeroData(float wantedRpm)
        {
            AeroData result = new AeroData
            {
                valid = false,
                signedAverageAoa = 0f,
                guardAoa = 0f,
                averageBladeSpeed = 0f,
                sampleCount = 0,
                hasDlcPanelAoa = false,
                dlcAoaSampleCount = 0,
                source = "None"
            };

            if (!antiStallEnabled)
                return result;

            if (useDlcAeroData)
            {
                float signedSum = 0f;
                float maxGuard = -999f;
                float speedSum = 0f;
                int samples = 0;

                float dlcAoaSum = 0f;
                float dlcAoaMax = -999f;
                int dlcAoaSamples = 0;

                foreach (ModuleControlSurface blade in blades)
                {

                    if (useDlcPanelAoa && TryReadDlcPanelAoaDeg(blade, out float dlcAoa))
                    {
                        dlcAoaSum += dlcAoa;
                        dlcAoaMax = Mathf.Max(dlcAoaMax, dlcAoa);
                        dlcAoaSamples++;
                    }

                    if (TryMeasureBladeAero(blade, out BladeAeroSample sample))
                    {
                        signedSum += sample.signedAoa;
                        // 0.3.9: no absolute AoA guard. Signed AoA is used only for positive high-AoA stall recovery.
                        maxGuard = Mathf.Max(maxGuard, sample.guardAoa);
                        speedSum += sample.bladeSpeed;
                        samples++;
                    }
                }

                if (dlcAoaSamples > 0 || samples > 0)
                {
                    // Prefer KSP/DLC's own panel AoA when available. The vector estimator can select
                    // the wrong signed-angle branch on mirrored clockwise/counter-clockwise blades.
                    if (dlcAoaSamples > 0)
                    {
                        result.valid = true;
                        result.signedAverageAoa = dlcAoaSum / dlcAoaSamples;
                        result.guardAoa = dlcAoaMax;
                        result.hasDlcPanelAoa = true;
                        result.dlcAoaSampleCount = dlcAoaSamples;
                        result.source = "DLC Panel AoA";
                    }
                    else
                    {
                        result.valid = samples > 0;
                        result.signedAverageAoa = samples > 0 ? signedSum / samples : 0f;
                        result.guardAoa = samples > 0 ? maxGuard : 0f;
                        result.source = "Vector AoA";
                    }

                    result.averageBladeSpeed = samples > 0 ? speedSum / samples : 0f;
                    result.sampleCount = samples;
                    return result;
                }
            }

            // Fallback only when real blade samples fail. It is intentionally used only as a stall guard estimate,
            // never as a takeoff pitch cap.
            float axialSpeed = EstimateAxialAirspeedFallback();
            axialAirspeedDisplay = axialSpeed;

            float helixAngle = EstimateHelixAngleDegreesFallback(wantedRpm, axialSpeed);
            float signedAoa = collectivePitch - helixAngle;
            result.valid = true;
            result.signedAverageAoa = signedAoa;
            result.guardAoa = signedAoa;
            result.averageBladeSpeed = 0f;
            result.sampleCount = 0;
            result.hasDlcPanelAoa = false;
            result.dlcAoaSampleCount = 0;
            result.source = "Fallback";
            return result;
        }

        private bool TryMeasureBladeAero(ModuleControlSurface blade, out BladeAeroSample sample)
        {
            sample = new BladeAeroSample
            {
                valid = false,
                signedAoa = 0f,
                guardAoa = 0f,
                bladeSpeed = 0f
            };

            if (blade == null || !blade.deploy || blade.part == null || part == null || rotor == null)
                return false;

            if (part.Rigidbody == null || blade.part.Rigidbody == null)
                return false;

            try
            {
                Vector3 axis = RotorAxis;
                if (axis.sqrMagnitude < 0.1f)
                    return false;

                Vector3 rotorOrigin = rotor.transform.position;
                Vector3 ownPartVelocity = part.Rigidbody.GetPointVelocity(rotorOrigin);
                Vector3 inflowVelocity = Vector3.Project(ownPartVelocity + Krakensbane.GetFrameVelocityV3f(), axis);
                Vector3 referencePoint = GetSurfaceReferencePoint(blade);
                Vector3 bladeVelocity = blade.part.Rigidbody.GetPointVelocity(referencePoint);

                // Measured blade relative flow. This is kept for stall guard only; it no longer creates a hard pitch cap.
                Vector3 measuredFlow = Vector3.ProjectOnPlane(bladeVelocity - ownPartVelocity, axis) + inflowVelocity;
                if (invertAeroFlow)
                    measuredFlow = -measuredFlow;

                float speed = measuredFlow.magnitude;
                if (speed < Mathf.Max(0.1f, minMeasuredBladeSpeed))
                    return false;

                Vector3 flowDir = measuredFlow.normalized;
                Vector3 pitchAxis = blade.transform.rotation * Vector3.right;
                blade.SetupCoefficients(Vector3.zero, out _, out Vector3 liftVectorZero, out _, out _);
                if (liftVectorZero.sqrMagnitude < 0.1f || pitchAxis.sqrMagnitude < 0.1f)
                    return false;

                liftVectorZero.Normalize();
                pitchAxis.Normalize();

                Vector3 currentLiftVector = Quaternion.AngleAxis(blade.deployAngle, pitchAxis) * liftVectorZero;
                float currentDot = Mathf.Clamp(Vector3.Dot(flowDir, currentLiftVector), -0.999f, 0.999f);
                float signedAoa = Mathf.Asin(currentDot) * Mathf.Rad2Deg;
                if (invertMeasuredAoaSign)
                    signedAoa = -signedAoa;

                sample.valid = true;
                sample.signedAoa = signedAoa;
                sample.guardAoa = signedAoa;
                sample.bladeSpeed = speed;
                return true;
            }
            catch (Exception)
            {
                return false;
            }
        }

        private bool TryReadDlcPanelAoaDeg(ModuleControlSurface blade, out float aoaDeg)
        {
            aoaDeg = 0f;
            if (blade == null || blade.part == null)
                return false;

            try
            {
                foreach (PartModule module in blade.part.Modules)
                {
                    if (module == null || module.Fields == null)
                        continue;

                    foreach (BaseField field in module.Fields)
                    {
                        if (field == null)
                            continue;

                        if (!LooksLikeDlcAoaField(field))
                            continue;

                        if (TryGetFieldFloat(module, field, out aoaDeg))
                            return true;
                    }
                }
            }
            catch (Exception)
            {
                aoaDeg = 0f;
            }

            return false;
        }

        private bool LooksLikeDlcAoaField(BaseField field)
        {
            string text = GetFieldSearchText(field);

            // Chinese KSP/DLC panel uses 迎角. Avoid generic 角度 because deploy angle/deflection also contains it.
            if (text.Contains("迎角") || text.Contains("攻角"))
                return true;

            if (text.Contains("angle of attack"))
                return true;

            // AoA is specific enough, but avoid broad matching of arbitrary angle fields.
            if (Regex.IsMatch(text, @"\baoa\b"))
                return true;

            return false;
        }

        private string GetFieldSearchText(BaseField field)
        {
            string gui = string.Empty;
            string internalName = string.Empty;

            try { gui = field.guiName ?? string.Empty; } catch (Exception) { gui = string.Empty; }
            try
            {
                object value = field.GetType().GetProperty("name") != null
                    ? field.GetType().GetProperty("name").GetValue(field, null)
                    : null;
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
                if (TryConvertToFloat(raw, out value))
                    return true;
            }
            catch (Exception)
            {
                // Fall back to GUI string below.
            }

            try
            {
                string guiString = field.GetStringValue(module, true);
                if (TryConvertToFloat(guiString, out value))
                    return true;
            }
            catch (Exception)
            {
                value = 0f;
            }

            return false;
        }

        private bool TryConvertToFloat(object raw, out float value)
        {
            value = 0f;
            if (raw == null)
                return false;

            try
            {
                if (raw is float f)
                {
                    value = f;
                    return true;
                }
                if (raw is double d)
                {
                    value = (float)d;
                    return true;
                }
                if (raw is int i)
                {
                    value = i;
                    return true;
                }
                if (raw is long l)
                {
                    value = l;
                    return true;
                }

                string text = raw.ToString();
                if (string.IsNullOrEmpty(text))
                    return false;

                Match match = Regex.Match(text, @"[-+]?\d+(?:[\.,]\d+)?");
                if (!match.Success)
                    return false;

                string number = match.Value.Replace(',', '.');
                return float.TryParse(number, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
            }
            catch (Exception)
            {
                value = 0f;
                return false;
            }
        }

        private Vector3 GetSurfaceReferencePoint(ModuleControlSurface blade)
        {
            try
            {
                return blade.displaceVelocity ? blade.transform.TransformPoint(blade.velocityOffset) : blade.transform.position;
            }
            catch (Exception)
            {
                return blade.transform.position;
            }
        }

        private float EstimateAxialAirspeedFallback()
        {
            if (vessel == null)
                return Mathf.Max(0f, minimumInflow);

            try
            {
                Vector3d axis = new Vector3d(RotorAxis.x, RotorAxis.y, RotorAxis.z);
                if (axis.sqrMagnitude < 0.01)
                    axis = new Vector3d(part.transform.up.x, part.transform.up.y, part.transform.up.z);

                axis.Normalize();
                double axial = Math.Abs(Vector3d.Dot(vessel.srf_velocity, axis));
                float fallback = (float)(vessel.srfSpeed * 0.25);
                return Mathf.Max(Mathf.Max((float)axial, fallback), minimumInflow);
            }
            catch (Exception)
            {
                return Mathf.Max((float)(vessel.srfSpeed * 0.25), minimumInflow);
            }
        }

        private float EstimateHelixAngleDegreesFallback(float wantedRpm, float axialSpeed)
        {
            float rpmForLimit = Mathf.Max(currentRpmDisplay, Mathf.Max(wantedRpm * 0.85f, 60f));
            float omega = rpmForLimit * Mathf.PI * 2f / 60f;
            float tangentialSpeed = omega * Mathf.Max(0.05f, effectiveBladeRadius);
            return Mathf.Atan2(Mathf.Max(0f, axialSpeed), Mathf.Max(0.1f, tangentialSpeed)) * Mathf.Rad2Deg;
        }

        private void InitializePitchFromBlades()
        {
            RefreshBlades();
            if (blades.Count > 0)
            {
                collectivePitch = blades.Average(b => b.deployAngle);
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
                if (child == null)
                    continue;

                ModuleControlSurface surface = child.FindModuleImplementing<ModuleControlSurface>();
                if (surface != null)
                {
                    // v3.7: Do this during refresh as well as during apply, so the blades are ready
                    // even before their PAW has ever been opened.
                    if (governorEnabled && !surface.deploy)
                        surface.deploy = true;

                    blades.Add(surface);
                }
            }

            bladeCountDisplay = blades.Count;
        }

        private void ApplyPitchToBlades(float requestedPitch)
        {
            foreach (ModuleControlSurface blade in blades)
            {
                if (blade == null)
                    continue;

                if (!blade.deploy)
                    blade.deploy = true;

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
                    // Some modded control surfaces may not expose standard deploy limits. Use governor limits instead.
                }

                blade.deployAngle = Mathf.Clamp(requestedPitch + commandPitchOffset, lower, upper);
            }
        }

        private static float MoveTowardWithRate(float current, float target, float maxRate, float dt)
        {
            float maxDelta = Mathf.Abs(maxRate) * dt;
            return Mathf.MoveTowards(current, target, maxDelta);
        }

        private static float LowPass(float previous, float current, float tau, float dt)
        {
            if (tau <= 0f)
                return current;

            float alpha = 1f - Mathf.Exp(-Mathf.Max(0f, dt) / Mathf.Max(0.001f, tau));
            return Mathf.Lerp(previous, current, Mathf.Clamp01(alpha));
        }
    }
}
