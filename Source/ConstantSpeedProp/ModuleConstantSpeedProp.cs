using System;
using System.Collections.Generic;
using System.Linq;
using Expansions.Serenity;
using UnityEngine;

namespace ConstantSpeedProp
{
    /// <summary>
    /// KSP1 / Breaking Ground fast constant-speed propeller governor.
    ///
    /// v3.5 design goal:
    /// - Remove takeoff pitch schedule protection completely.
    /// - Remove low-throttle/low-power hold protection completely.
    /// - Make pitch response immediate and symmetric for identical rotors.
    /// - Keep RPM as the primary control objective because rotor RPM determines available shaft power.
    /// - Keep a stall guard that can override RPM control only when measured/guard AoA exceeds the configured limit.
    ///
    /// This module attaches to a ModuleRoboticServoRotor and controls directly attached ModuleControlSurface blades.
    /// </summary>
    public class ModuleConstantSpeedProp : PartModule
    {
        private const string VersionText = "v3.5 FastRPM NoTakeoffHold";

        private ModuleRoboticServoRotor rotor;
        private readonly List<ModuleControlSurface> blades = new List<ModuleControlSurface>();

        private float collectivePitch;
        private float integralError;
        private float previousError;
        private bool initializedPitch;
        private float nextBladeRefreshTime;

        private struct AeroData
        {
            public bool valid;
            public float signedAverageAoa;
            public float guardAoa;
            public float averageBladeSpeed;
            public int sampleCount;
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

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Constant Speed Prop")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "Off", enabledText = "On", affectSymCounterparts = UI_Scene.All)]
        public bool governorEnabled = true;

        [KSPField(isPersistant = false, guiActive = true, guiActiveEditor = true, guiName = "CSP Version")]
        public string versionDisplay = VersionText;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Target RPM", guiUnits = " rpm")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1000f, stepIncrement = 10f, affectSymCounterparts = UI_Scene.All)]
        public float targetRpm = 450f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Throttle Sets RPM")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "No", enabledText = "Yes", affectSymCounterparts = UI_Scene.All)]
        public bool throttleSetsTargetRpm = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Idle RPM", guiUnits = " rpm")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1000f, stepIncrement = 10f, affectSymCounterparts = UI_Scene.All)]
        public float idleRpm = 120f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Min Pitch", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -30f, maxValue = 80f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float minPitch = 2f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Max Pitch", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -30f, maxValue = 80f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float maxPitch = 55f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Pitch Rate", guiUnits = " deg/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 1f, maxValue = 120f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float pitchRateLimit = 35f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Min Response Rate", guiUnits = " deg/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 60f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float minimumCorrectionRate = 10f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "RPM Deadband", guiUnits = " rpm")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 80f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float rpmDeadband = 5f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "P Gain")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 1f, stepIncrement = 0.005f, affectSymCounterparts = UI_Scene.All)]
        public float proportionalGain = 0.08f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "I Gain")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 0.2f, stepIncrement = 0.001f, affectSymCounterparts = UI_Scene.All)]
        public float integralGain = 0.002f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "D Gain")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 0.2f, stepIncrement = 0.001f, affectSymCounterparts = UI_Scene.All)]
        public float derivativeGain = 0.0f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Feather")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "Off", enabledText = "On", affectSymCounterparts = UI_Scene.All)]
        public bool feather = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Feather Pitch", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -90f, maxValue = 90f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float featherPitch = 80f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Anti-Stall Guard")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "Off", enabledText = "On", affectSymCounterparts = UI_Scene.All)]
        public bool antiStallEnabled = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Use DLC Aero Data")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "No", enabledText = "Yes", affectSymCounterparts = UI_Scene.All)]
        public bool useDlcAeroData = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Abs AoA Guard")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "Off", enabledText = "On", affectSymCounterparts = UI_Scene.All)]
        public bool useAbsoluteAoaLimit = true;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Invert Aero Flow")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "Off", enabledText = "On", affectSymCounterparts = UI_Scene.All)]
        public bool invertAeroFlow = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Invert AoA Sign")]
        [UI_Toggle(scene = UI_Scene.All, disabledText = "Off", enabledText = "On", affectSymCounterparts = UI_Scene.All)]
        public bool invertMeasuredAoaSign = false;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Target Blade AoA", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 20f, stepIncrement = 0.5f, affectSymCounterparts = UI_Scene.All)]
        public float targetBladeAoa = 6f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Max Blade AoA", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 2f, maxValue = 45f, stepIncrement = 0.5f, affectSymCounterparts = UI_Scene.All)]
        public float maxBladeAoa = 12f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "AoA Inc Rate Cap", guiUnits = " deg/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 60f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float aoaPitchIncreaseLimit = 12f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "AoA Recovery Rate", guiUnits = " deg/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 1f, maxValue = 120f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float aoaRecoveryRate = 90f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Min Blade Speed", guiUnits = " m/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 60f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float minMeasuredBladeSpeed = 5f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Command Offset", guiUnits = " deg")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = -30f, maxValue = 30f, stepIncrement = 1f, affectSymCounterparts = UI_Scene.All)]
        public float commandPitchOffset = 0f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Blade Radius Fallback", guiUnits = " m")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0.1f, maxValue = 10f, stepIncrement = 0.05f, affectSymCounterparts = UI_Scene.All)]
        public float effectiveBladeRadius = 1.25f;

        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Min Inflow Fallback", guiUnits = " m/s")]
        [UI_FloatRange(scene = UI_Scene.All, minValue = 0f, maxValue = 20f, stepIncrement = 0.5f, affectSymCounterparts = UI_Scene.All)]
        public float minimumInflow = 0f;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Current RPM", guiUnits = " rpm", guiFormat = "F0")]
        public float currentRpmDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "RPM Error", guiUnits = " rpm", guiFormat = "F0")]
        public float rpmErrorDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Commanded Pitch", guiUnits = " deg", guiFormat = "F1")]
        public float commandedPitchDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Pitch Rate Cmd", guiUnits = " deg/s", guiFormat = "F1")]
        public float pitchRateCommandDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Pitch Limit", guiUnits = " deg", guiFormat = "F1")]
        public float pitchLimitDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Estimated AoA", guiUnits = " deg", guiFormat = "F1")]
        public float estimatedAoaDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Guard AoA", guiUnits = " deg", guiFormat = "F1")]
        public float guardAoaDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Blade Speed", guiUnits = " m/s", guiFormat = "F1")]
        public float measuredBladeSpeedDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Axial Airspeed", guiUnits = " m/s", guiFormat = "F1")]
        public float axialAirspeedDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Aero Samples")]
        public int aeroSampleCountDisplay;

        [KSPField(isPersistant = false, guiActive = true, guiName = "Blade Count")]
        public int bladeCountDisplay;

        private void ForceFieldVisibility()
        {
            string[] visibleFields =
            {
                "versionDisplay",
                "minimumCorrectionRate",
                "rpmDeadband",
                "antiStallEnabled",
                "useDlcAeroData",
                "useAbsoluteAoaLimit",
                "invertAeroFlow",
                "invertMeasuredAoaSign",
                "targetBladeAoa",
                "maxBladeAoa",
                "aoaPitchIncreaseLimit",
                "aoaRecoveryRate",
                "minMeasuredBladeSpeed",
                "commandPitchOffset",
                "effectiveBladeRadius",
                "minimumInflow",
                "currentRpmDisplay",
                "rpmErrorDisplay",
                "commandedPitchDisplay",
                "pitchRateCommandDisplay",
                "pitchLimitDisplay",
                "estimatedAoaDisplay",
                "guardAoaDisplay",
                "measuredBladeSpeedDisplay",
                "axialAirspeedDisplay",
                "aeroSampleCountDisplay",
                "bladeCountDisplay"
            };

            foreach (string fieldName in visibleFields)
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
                    continue;

                field.guiActive = true;
                field.guiActiveEditor = true;
            }
        }

        [KSPAction("Toggle Constant Speed Prop")]
        public void ToggleGovernor(KSPActionParam param)
        {
            governorEnabled = !governorEnabled;
        }

        [KSPAction("Feather Propeller")]
        public void FeatherAction(KSPActionParam param)
        {
            feather = true;
        }

        [KSPAction("Unfeather Propeller")]
        public void UnfeatherAction(KSPActionParam param)
        {
            feather = false;
        }

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            versionDisplay = VersionText;
            ForceFieldVisibility();
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
        }

        public void FixedUpdate()
        {
            if (!HighLogic.LoadedSceneIsFlight || FlightDriver.Pause || rotor == null)
                return;

            if (Planetarium.GetUniversalTime() >= nextBladeRefreshTime)
            {
                RefreshBlades();
                nextBladeRefreshTime = (float)Planetarium.GetUniversalTime() + 1f;
            }

            currentRpmDisplay = Mathf.Abs(rotor.currentRPM);
            bladeCountDisplay = blades.Count;

            if (!governorEnabled || blades.Count == 0)
                return;

            float dt = TimeWarp.fixedDeltaTime;
            if (dt <= 0f)
                return;

            if (!initializedPitch)
                InitializePitchFromBlades();

            float lowerPitch = Mathf.Min(minPitch, maxPitch);
            float upperPitch = Mathf.Max(minPitch, maxPitch);
            float wantedRpm = GetWantedRpm();
            AeroData aero = GetAeroData(wantedRpm);

            pitchLimitDisplay = upperPitch;
            estimatedAoaDisplay = aero.signedAverageAoa;
            guardAoaDisplay = aero.guardAoa;
            measuredBladeSpeedDisplay = aero.averageBladeSpeed;
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
            }
            else
            {
                float rpmError = currentRpmDisplay - wantedRpm;
                rpmErrorDisplay = rpmError;
                requestedPitchRate = ComputeGovernorPitchRate(rpmError, dt);

                if (antiStallEnabled && aero.valid)
                {
                    ApplyStallGuard(ref requestedPitchRate, aero.guardAoa);
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
                pitchToApply = collectivePitch;
            }

            pitchRateCommandDisplay = requestedPitchRate;
            ApplyPitchToBlades(pitchToApply);
            commandedPitchDisplay = pitchToApply;
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
            if (floor > 0f)
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

        private void ApplyStallGuard(ref float requestedPitchRate, float guardAoa)
        {
            float targetAoa = Mathf.Min(targetBladeAoa, maxBladeAoa - 0.5f);

            if (guardAoa >= maxBladeAoa)
            {
                // Stall protection has priority over RPM control. If pitch is already too high for the measured flow,
                // move toward fine pitch immediately, even if RPM is above target.
                float severity = Mathf.Clamp01((guardAoa - maxBladeAoa) / 8f + 0.35f);
                requestedPitchRate = Mathf.Min(requestedPitchRate, -Mathf.Abs(aoaRecoveryRate) * severity);
                integralError = 0f;
                return;
            }

            if (guardAoa >= targetAoa && requestedPitchRate > aoaPitchIncreaseLimit)
            {
                // Near stall region: keep RPM authority, but do not let overspeed spikes command an instant coarse-pitch jump.
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

        private AeroData GetAeroData(float wantedRpm)
        {
            AeroData result = new AeroData
            {
                valid = false,
                signedAverageAoa = 0f,
                guardAoa = 0f,
                averageBladeSpeed = 0f,
                sampleCount = 0,
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

                foreach (ModuleControlSurface blade in blades)
                {
                    if (TryMeasureBladeAero(blade, out BladeAeroSample sample))
                    {
                        signedSum += sample.signedAoa;
                        maxGuard = Mathf.Max(maxGuard, sample.guardAoa);
                        speedSum += sample.bladeSpeed;
                        samples++;
                    }
                }

                if (samples > 0)
                {
                    result.valid = true;
                    result.signedAverageAoa = signedSum / samples;
                    result.guardAoa = maxGuard;
                    result.averageBladeSpeed = speedSum / samples;
                    result.sampleCount = samples;
                    result.source = "DLC Measured";
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
            result.guardAoa = useAbsoluteAoaLimit ? Mathf.Abs(signedAoa) : signedAoa;
            result.averageBladeSpeed = 0f;
            result.sampleCount = 0;
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
                sample.guardAoa = useAbsoluteAoaLimit ? Mathf.Abs(signedAoa) : signedAoa;
                sample.bladeSpeed = speed;
                return true;
            }
            catch (Exception)
            {
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
            initializedPitch = true;
        }

        private void RefreshBlades()
        {
            blades.Clear();
            foreach (Part child in part.children)
            {
                ModuleControlSurface surface = child.FindModuleImplementing<ModuleControlSurface>();
                if (surface != null)
                    blades.Add(surface);
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
    }
}
