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
 * The roller wheel on the end of the arm that pulls Fuel in while the intake is deployed.
 *
 * <p>A real TalonFX flywheel (a Kraken X60 directly driving a small wheel), built by
 * extending the library {@link Flywheel} class.
 *
 * <p>Unlike the LQR arm, this uses {@code ControlStyle.PROFILED_PID}: the speed loop runs
 * <em>onboard the motor controller</em> using a P gain plus a velocity feedforward
 * ({@code kV}). For a velocity controller the feedforward does almost all the work — a good
 * starting value is {@code kV ≈ 12 V ÷ free speed in rotations/second}, and {@code kP} only
 * trims what's left. This is the classic PID-style approach, shown here so you can compare it
 * with the arm's LQR.
 */
public class IntakeRoller extends Flywheel {

  /** CAN ID of the roller motor. */
  public static final int CAN_ID = 42;

  /** Wheel speed while collecting. Modest — it only needs to drag Fuel over the bumper. */
  public static final AngularVelocity INTAKE_RPM = RPM.of(2000);

  public IntakeRoller() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "IntakeRoller";
    s.canId = CAN_ID;
    s.controlStyle = ControlStyle.PROFILED_PID; // onboard PID instead of LQR
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {1.0}; // direct drive
    s.diameter = Inches.of(2);
    s.mass = Kilograms.of(0.4);
    s.kP = 0.15;  // volts per rotation/second of error
    s.kV = 0.115; // volts per rotation/second (theoretical ≈ 0.12)
    s.visualPose3d = new Pose3d(0.6, 0.0, 0.45, Rotation3d.kZero);
    return s;
  }
}
