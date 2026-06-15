package org.frc5010.examples.mechanisms;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.ControlStyle;
import org.frc5010.common.mechanisms.Flywheel;

/**
 * Velocity-PID variant of {@link ExampleShooter}: same physical wheel (Kraken X60
 * direct drive, 4 in / 1.5 kg) but with {@code ControlStyle.PROFILED_PID} — onboard
 * VelocityVoltage with kV feedforward (CAN 34).
 *
 * <p>For velocity PID the kV feedforward does most of the work
 * (theoretical kV = 12 V ÷ 100 rot/s free speed = 0.12); kP only trims the residual.
 */
public class ExampleProfiledShooter extends Flywheel {

  /** CAN ID of the shooter TalonFX. */
  public static final int CAN_ID = 34;

  public ExampleProfiledShooter() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleProfiledShooter";
    s.controlStyle = ControlStyle.PROFILED_PID;
    s.canId = CAN_ID;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {1.0}; // direct drive
    s.diameter = Inches.of(4);
    s.mass = Kilograms.of(1.5);
    s.visualPose3d = new edu.wpi.first.math.geometry.Pose3d(-0.1, -0.25, 0.6,
        edu.wpi.first.math.geometry.Rotation3d.kZero); // wheel in the side-view plane, 3D view
    s.kP = 0.15;  // volts per rot/s of error
    s.kV = 0.115; // volts per rot/s (theoretical 0.12)
    return s;
  }
}
