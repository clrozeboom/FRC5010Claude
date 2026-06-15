package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import org.frc5010.common.mechanisms.Arm;

/**
 * The arm that extends and retracts the intake.
 *
 * <p>This is a real TalonFX mechanism — one Kraken X60 on a 50:1 gearbox — built by
 * extending the library {@link Arm} class and filling in this robot's physical numbers.
 * The control logic lives in {@link Arm}; this file is just the configuration.
 *
 * <p>It is left on the default {@code ControlStyle.LQR}: the library builds an
 * optimal controller automatically from the mass, length, gearing, and motor model,
 * so there are <em>no PID gains to hand-tune</em>. (The two flywheels, by contrast, use
 * onboard PROFILED_PID so you can see both control styles — see {@link IntakeRoller}.)
 */
public class IntakeArm extends Arm {

  /** CAN ID of the arm motor. Sim-only here; on a real robot use your wired ID. */
  public static final int CAN_ID = 41;

  /** Angle the arm holds when the intake is deployed (out, ready to collect). */
  public static final Angle DEPLOY_ANGLE = Degrees.of(110);
  /** Angle the arm holds when the intake is retracted (stowed inside the frame). */
  public static final Angle RETRACT_ANGLE = Degrees.of(0);
  /** How close to a target angle counts as "there", for status checks. */
  public static final Angle ANGLE_TOLERANCE = Degrees.of(5);

  public IntakeArm() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "IntakeArm";
    s.canId = CAN_ID;
    // controlStyle defaults to LQR — nothing to set, nothing to tune.
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {10, 5}; // 50:1
    s.length = Meters.of(0.5);
    s.mass = Kilograms.of(3.0);
    s.minAngle = Degrees.of(-10); // a little below stowed, as a soft limit
    s.maxAngle = Degrees.of(120); // a little past deployed, as a soft limit
    s.startingAngle = RETRACT_ANGLE; // starts stowed at power-on
    s.maxVelocity = DegreesPerSecond.of(220);
    s.maxAcceleration = DegreesPerSecondPerSecond.of(440);
    s.kG = Volts.of(0.35); // volts to hold the arm level (gravity feedforward)
    // Where to draw the arm in the 3D view (robot frame: x fwd, y left, z up).
    s.visualPose3d = new Pose3d(0.15, 0.0, 0.25, Rotation3d.kZero);
    return s;
  }
}
