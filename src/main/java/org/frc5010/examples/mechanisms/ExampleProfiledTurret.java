package org.frc5010.examples.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.ControlStyle;
import org.frc5010.common.mechanisms.Pivot;

/**
 * Profiled-PID variant of {@link ExampleTurret}: same physical turret (Kraken X60,
 * 40:1, 0.5 kg·m²) but with {@code ControlStyle.PROFILED_PID} — trapezoid profile +
 * onboard MotionMagic, kS/kV feedforward, no gravity term (CAN 33).
 *
 * <p>Gains in mechanism rotations: kV theoretical = 12 V ÷ 2.5 rot/s free speed = 4.8.
 */
public class ExampleProfiledTurret extends Pivot {

  /** CAN ID of the turret TalonFX. */
  public static final int CAN_ID = 33;
  /** CAN ID of the turret's absolute CANcoder. */
  public static final int CANCODER_ID = 43;

  public ExampleProfiledTurret() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleProfiledTurret";
    s.controlStyle = ControlStyle.PROFILED_PID;
    s.canId = CAN_ID;
    // Absolute CANcoder mounted 1:1 on the turret, fused onboard — the turret knows
    // its true angle at power-on without re-zeroing. (Best paired with onboard
    // control: MotionMagic consumes the fused sensor at 1 kHz on the motor itself.)
    s.cancoderId = CANCODER_ID;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {10, 4}; // 40:1
    s.moi = KilogramSquareMeters.of(0.5);
    s.minAngle = Degrees.of(-180);
    s.maxAngle = Degrees.of(180);
    s.startingAngle = Degrees.of(0);
    // 3D view: YAW_PLANE = turret spinning about the vertical axis (see ExampleTurret).
    s.visualPose3d = new edu.wpi.first.math.geometry.Pose3d(-0.3, 0, 0.55,
        org.frc5010.common.mechanisms.MechanismVisuals3d.YAW_PLANE);
    s.maxVelocity = DegreesPerSecond.of(360);
    s.maxAcceleration = DegreesPerSecondPerSecond.of(720);
    s.kP = 40;   // volts per turret rotation of error
    s.kV = 4.6;  // volts per turret rotation/s (theoretical 4.8)
    return s;
  }
}
