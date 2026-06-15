package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5010.common.mechanisms.ControlStyle;
import org.frc5010.common.mechanisms.Flywheel;

/**
 * The onboard shooter flywheel that launches held Fuel.
 *
 * <p>A second real TalonFX flywheel (Kraken X60 on a 4-inch wheel). The robot scores by
 * spinning this wheel up to speed and then releasing one Fuel, which flies out in the
 * direction the robot is facing (see {@link FuelHandler}).
 *
 * <p>Like {@link IntakeRoller}, this uses onboard {@code ControlStyle.PROFILED_PID} so the
 * lesson covers PID as well as the arm's LQR.
 */
public class Shooter extends Flywheel {

  /** CAN ID of the shooter motor. */
  public static final int CAN_ID = 43;

  /** Speed the wheel must reach before it can launch Fuel accurately. */
  public static final AngularVelocity SHOOT_RPM = RPM.of(3000);
  /** How close to {@link #SHOOT_RPM} counts as "ready to fire". */
  public static final AngularVelocity RPM_TOLERANCE = RPM.of(150);

  public Shooter() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "Shooter";
    s.canId = CAN_ID;
    s.controlStyle = ControlStyle.PROFILED_PID; // onboard PID
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {1.0}; // direct drive
    s.diameter = Inches.of(4);
    s.mass = Kilograms.of(1.5);
    s.kP = 0.15;  // volts per rotation/second of error
    s.kV = 0.115; // volts per rotation/second (theoretical ≈ 0.12)
    s.visualPose3d = new Pose3d(-0.15, 0.0, 0.6, Rotation3d.kZero);
    return s;
  }
}
