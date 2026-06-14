package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.Arm;

/**
 * Example LQR arm: one Kraken X60 on a TalonFX (CAN 22), 50:1 gearbox, 0.6 m / 4 kg arm
 * sweeping from −30° to 210° (0° = horizontal).
 *
 * <p>Robot-specific numbers live here; all control logic is in the common
 * {@link Arm}. Copy this class and replace the constants for your robot.
 * kG (0.40 V, voltage to hold horizontal) was computed from the arm mass/length,
 * gearing, and Kraken motor constants — on a real robot, characterize it with
 * {@code sysId()} instead.
 */
public class ExampleArm extends Arm {

  /** CAN ID of the arm TalonFX. */
  public static final int CAN_ID = 22;

  public ExampleArm() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleArm";
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
    s.kG = Volts.of(0.40); // m·g·(L/2) / (gearing · kT / R) for the plant above
    s.visualPose3d = new edu.wpi.first.math.geometry.Pose3d(0.1, 0.15, 0.4,
        edu.wpi.first.math.geometry.Rotation3d.kZero); // swings fore-aft, 3D view
    return s;
  }
}
