package frc.robot.rebuilt.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Configuration for {@link SmartTurretController} — the 2-state torque-current-FOC turret loop.
 *
 * <p>Ported from the source Rebuilt2026 {@code SmartTurretConfig}. Slot feedforward (kS, kV, kA)
 * is in <b>Amps</b> for TorqueCurrentFOC; the MotionMagicExpo plant model (expoKV/expoKA) is in
 * <b>Volts</b> regardless of control mode; positions are in <b>mechanism rotations</b>
 * (post-gear). Defaults match the source builder. Public fields follow the FRC5010Claude
 * {@code Settings} convention.
 */
public class SmartTurretConfig {

  /** Telemetry/visualization name. */
  public String name = "Turret";
  /**
   * Robot-frame mount pose for 3D visualization, or {@code null} to disable.
   * Use {@link org.frc5010.common.mechanisms.MechanismVisuals3d#YAW_PLANE} as the rotation so
   * the turret sweeps in the horizontal plane (0° = robot forward, CCW positive).
   */
  public Pose3d visualPose3d = null;
  /** Length of the turret arm drawn in the 3D view, meters. */
  public double visualArmLengthM = 0.15;

  /** CAN ID of the turret TalonFX. */
  public int canId = 18;
  /** CAN bus the turret lives on (CANivore on the real robot). */
  public String canBus = "canivore";
  /** Rotor → mechanism gear reduction. */
  public double gearRatio = 30.0;

  // Motion constraints (mechanism rot/s, rot/s²).
  public double maxVelocityMechRotPerSec = 3.0;
  public double maxAccelMechRotPerSecSq = 2.78;

  // Seeking PID — Slot0 (MotionMagicExpoTorqueCurrentFOC).
  public double seekingKP = 225;
  public double seekingKI = 0;
  public double seekingKD = 50;

  // Tracking PID — Slot1 (PositionTorqueCurrentFOC).
  public double trackingKP = 225;
  public double trackingKI = 0;
  public double trackingKD = 50;

  // Feedforward in Amps (TorqueCurrentFOC units).
  public double kS = 10.0;
  public double kV = 0.0;
  public double kA = 5.0;

  /** MotionMagicExpo kV in V/(mech rot/s); ≤0 = auto = 12 ÷ maxVelocity. */
  public double expoKV = -1;
  /** MotionMagicExpo kA in V/(mech rot/s²); ≤0 = auto = 12 ÷ maxAccel. */
  public double expoKA = -1;

  // State-transition thresholds (mechanism rotations).
  public double seekingThresholdRotations = 10.0 / 360.0;
  public double hysteresisBufferRotations = 3.0 / 360.0;

  // Soft limits (mechanism rotations).
  public double lowerLimitRotations = -150.0 / 360.0;
  public double upperLimitRotations = 150.0 / 360.0;

  /** Peak torque current (Amps). */
  public double peakTorqueCurrentAmps = 240.0;
  /** Feedforward safety padding near the limits (mechanism rotations). */
  public double feedforwardPaddingRotations = 10.0 / 360.0;
  /** Tracking-mode kS deadband (mechanism rotations) — zero kS inside to settle without chatter. */
  public double trackingDeadbandRotations = 0.25 / 360.0;

  // Simulation plant.
  /** Motor model for the sim plant (KrakenX44 ≈ KrakenX60 model). */
  public DCMotor motorModel = DCMotor.getKrakenX60(1);
  /** Rotating assembly moment of inertia about the turret axis (kg·m²). */
  public double moiKgM2 = 0.05;
  /** Mechanism position the turret powers on at (rotations). */
  public double startingPositionRot = 0.0;

  /** Resolved MotionMagicExpo kV (auto-computed when {@link #expoKV} ≤ 0). */
  public double resolvedExpoKV() {
    return expoKV > 0 ? expoKV : 12.0 / maxVelocityMechRotPerSec;
  }

  /** Resolved MotionMagicExpo kA (auto-computed when {@link #expoKA} ≤ 0). */
  public double resolvedExpoKA() {
    return expoKA > 0 ? expoKA : 12.0 / maxAccelMechRotPerSecSq;
  }
}
