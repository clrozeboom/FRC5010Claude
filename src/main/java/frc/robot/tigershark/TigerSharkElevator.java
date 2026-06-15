package frc.robot.tigershark;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.frc5010.common.mechanisms.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;

/**
 * TigerShark LQR elevator: 2 Kraken X60s on a TalonFX (CAN 9 lead, 10 follower), 6:1
 * gearbox driving a 1.1 in radius drum, 2.27 kg carriage, ~1.95 m of travel.
 *
 * <p>Robot-specific numbers live here; all control logic is in the common {@link Elevator}.
 */
public class TigerSharkElevator extends Elevator {

  /** CAN ID of the lead elevator TalonFX. */
  public static final int CAN_ID = 9;
  /** CAN ID of the second motor on the gearbox, following the lead. */
  public static final int FOLLOWER_CAN_ID = 10;
  /** Game-piece scoring height. */
  public static final Distance SCORING_HEIGHT = Meters.of(0.75);

  /** Velocity below which we treat the elevator as idle (for LED state). */
  private static final double MOVING_THRESHOLD_MPS = 0.02;

  public TigerSharkElevator() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "TigerSharkElevator";
    s.canId = CAN_ID;
    // Two-motor gearbox: a follower on CAN 10, mounted on the opposite end of the gearbox
    // shaft so it spins opposite-handed; followerOpposed flips its voltage to match.
    // Drawn in the 3D view as a mirror of the elevator 0.5 m to the -Y side (far rail).
    s.followerCanId = FOLLOWER_CAN_ID;
    s.followerOpposed = true;
    s.followerVisualOffset = new Translation3d(0, -0.5, 0);

    s.motorModel = DCMotor.getKrakenX60(2);            // two Krakens combined on one gearbox
    s.gearReductionStages = new double[] {6};          // 6:1
    s.drumCircumference = Inches.of(1.1 * 2 * Math.PI);// 1.1 in radius drum
    s.carriageMass = Kilograms.of(2.27);               // ~5 lb

    s.minHeight = Meters.of(0);
    s.maxHeight = Meters.of(1.95);                     // 76.75 in travel
    s.startingHeight = Meters.of(0);

    // Sanity check (gotcha #12): free speed at the drum is 100/6 rps × 0.1755 m ≈ 2.93 m/s.
    // 0.9 m/s is ~31% of that — comfortable cruise.
    s.maxVelocity = MetersPerSecond.of(0.9);
    s.maxAcceleration = MetersPerSecondPerSecond.of(2.0);

    s.kG = Volts.of(0.3);                              // initial guess; refine via /tune-mechanism
    s.visualPose3d = new Pose3d(
        new Translation3d(
            Inches.of(5.75).in(Meters),
            Inches.of(4.75).in(Meters),
            Inches.of(3).in(Meters)),
        new Rotation3d());
    return s;
  }

  /** True when the carriage is moving fast enough to count as "in motion" for LED state. */
  public boolean isMoving() {
    return Math.abs(getVelocity().in(MetersPerSecond)) > MOVING_THRESHOLD_MPS;
  }
}
