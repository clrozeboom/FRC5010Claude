package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.ControlStyle;
import org.frc5010.common.mechanisms.Elevator;

/**
 * Profiled-PID variant of {@link ExampleElevator}: same physical elevator (Kraken X60,
 * 12:1, 22T sprocket, 6 kg carriage) but with {@code ControlStyle.PROFILED_PID} —
 * trapezoid profile + onboard MotionMagic on the TalonFX (CAN 31).
 *
 * <p>Gains are in mechanism (drum) rotations: kP volts/rotation, kV volts per
 * rotation/s (theoretical kV = 12 V ÷ 8.33 rot/s free speed ≈ 1.44). Pick this style
 * over LQR when you want hand-tunable gains and don't trust the mass/gearing model.
 */
public class ExampleProfiledElevator extends Elevator {

  /** CAN ID of the elevator TalonFX. */
  public static final int CAN_ID = 31;

  public ExampleProfiledElevator() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleProfiledElevator";
    s.controlStyle = ControlStyle.PROFILED_PID;
    s.canId = CAN_ID;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {4, 3}; // 12:1
    s.drumCircumference = Inches.of(0.25 * 22); // 22T #25-chain sprocket
    s.carriageMass = Kilograms.of(6.0);
    s.minHeight = Meters.of(0);
    s.maxHeight = Meters.of(1.5);
    s.startingHeight = Meters.of(0.1);
    s.maxVelocity = MetersPerSecond.of(0.9);
    s.maxAcceleration = MetersPerSecondPerSecond.of(2.0);
    s.kG = Volts.of(0.19); // onboard Elevator_Static gravity compensation
    s.visualPosition = new edu.wpi.first.math.geometry.Translation2d(0.55, 0.0); // spot on the RobotMechanisms overlay
    s.visualPose3d = new edu.wpi.first.math.geometry.Pose3d(0.25, -0.25, 0,
        edu.wpi.first.math.geometry.Rotation3d.kZero); // front-right corner, 3D view
    s.kP = 6;    // volts per drum rotation of error
    s.kV = 1.4;  // volts per drum rotation/s (theoretical 1.44)
    return s;
  }
}
