package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.ControlStyle;
import org.frc5010.common.mechanisms.Arm;

/**
 * Profiled-PID variant of {@link ExampleArm}: same physical arm (Kraken X60, 50:1,
 * 0.6 m / 4 kg) but with {@code ControlStyle.PROFILED_PID} — trapezoid profile +
 * onboard MotionMagic with Arm_Cosine gravity compensation (CAN 32).
 *
 * <p>Gains in mechanism rotations: kP volts/rotation, kV volts per rotation/s
 * (theoretical kV = 12 V ÷ 2 rot/s free speed = 6.0).
 */
public class ExampleProfiledArm extends Arm {

  /** CAN ID of the arm TalonFX. */
  public static final int CAN_ID = 32;

  public ExampleProfiledArm() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleProfiledArm";
    s.controlStyle = ControlStyle.PROFILED_PID;
    s.canId = CAN_ID;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {10, 5}; // 50:1
    s.length = Meters.of(0.6);
    s.mass = Kilograms.of(4.0);
    s.minAngle = Degrees.of(-30);
    s.maxAngle = Degrees.of(210);
    s.startingAngle = Degrees.of(0);
    s.maxVelocity = DegreesPerSecond.of(180);
    s.maxAcceleration = DegreesPerSecondPerSecond.of(360);
    s.kG = Volts.of(0.40); // onboard Arm_Cosine gravity compensation
    s.visualPosition = new edu.wpi.first.math.geometry.Translation2d(1.7, 1.8); // spot on the RobotMechanisms overlay
    s.visualPose3d = new edu.wpi.first.math.geometry.Pose3d(0.1, -0.15, 0.4,
        edu.wpi.first.math.geometry.Rotation3d.kZero); // swings fore-aft, 3D view
    s.kP = 40;   // volts per arm rotation of error
    s.kV = 5.8;  // volts per arm rotation/s (theoretical 6.0)
    return s;
  }
}
