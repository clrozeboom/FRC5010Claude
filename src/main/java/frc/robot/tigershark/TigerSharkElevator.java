package frc.robot.tigershark;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import org.frc5010.common.mechanisms.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;

/**
 * Tiger Shark LQR elevator: 2 Kraken X60s on a TalonFX (CAN 9 & 10), 6:1 gearbox driving a
 * 1.1 in radius drum, 5 lb carriage, 83.475 - 6.725 in of travel.
 *
 * <p>Robot-specific numbers live here; all control logic is in the common {@link Elevator}. 
 */
public class TigerSharkElevator extends Elevator {

  /** CAN ID of the elevator TalonFX. */
  public static final int CAN_ID = 9;
  /** CAN ID of the second motor on the gearbox, following the lead. */
  public static final int FOLLOWER_CAN_ID = 10;
  /** Scoring height in inches. */
  public static final Distance SCORING_HEIGHT = Inches.of(30.0);

  public TigerSharkElevator() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "TigerSharkElevator";
    s.canId = CAN_ID;
    // Two-motor gearbox: a follower on CAN 10, drawn in the 3D view as an offset mirror
    // of the elevator 0.5 m to the +Y side (the far rail of the carriage). On a real
    // two-motor gearbox also set motorModel = DCMotor.getKrakenX60(2) and re-characterize
    // kG; this demo keeps the single-motor plant so its tuning matches the other examples.
    s.followerCanId = FOLLOWER_CAN_ID;
    s.followerOpposed = true;
    s.followerVisualOffset = new edu.wpi.first.math.geometry.Translation3d(0, -0.5, 0);
    s.motorModel = DCMotor.getKrakenX60(2);
    s.gearReductionStages = new double[] {6}; // 12:1
    s.drumCircumference = Inches.of(1.1 * 2 * Math.PI); // 22T #25-chain sprocket
    s.carriageMass = Pounds.of(5.0);
    s.minHeight = Inches.of(6.725);
    s.maxHeight = Inches.of(83.475);
    s.startingHeight = Inches.of(6.725);
    // Free speed at the drum is ~1.16 m/s (100 rps / 12 × 0.1397 m); stay below it.
    s.maxVelocity = MetersPerSecond.of(0.9);
    s.maxAcceleration = MetersPerSecondPerSecond.of(2.0);
    s.kG = Volts.of(0.3); // m·g·r / (gearing · kT / R) for the plant above
    s.visualPose3d = new Pose3d(
                new Translation3d(Inches.of(5.75).in(Meters), Inches.of(4.75).in(Meters), Inches.of(3).in(Meters)),
                new Rotation3d());
    return s;
  }

  public Boolean isMoving() {
    return false;
  }

  public Boolean isNearTop() {
    return false;
  }

  public Boolean isNearBottom() {
    return false;
  }
}
