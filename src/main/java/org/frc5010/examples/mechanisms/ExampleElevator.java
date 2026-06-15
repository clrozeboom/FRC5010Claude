package org.frc5010.examples.mechanisms;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.Elevator;

/**
 * Example LQR elevator: one Kraken X60 on a TalonFX (CAN 21), 12:1 gearbox driving a
 * 22-tooth #25 sprocket, 6 kg carriage, 1.5 m of travel.
 *
 * <p>Robot-specific numbers live here; all control logic is in the common
 * {@link Elevator}. Copy this class and replace the constants for your robot.
 * kG (0.19 V) was computed from the carriage mass, drum radius, gearing, and Kraken
 * motor constants — on a real robot, characterize it with {@code sysId()} instead.
 */
public class ExampleElevator extends Elevator {

  /** CAN ID of the elevator TalonFX. */
  public static final int CAN_ID = 21;
  /** CAN ID of the second motor on the gearbox, following the lead. */
  public static final int FOLLOWER_CAN_ID = 36;

  public ExampleElevator() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleElevator";
    s.canId = CAN_ID;
    // Two-motor gearbox: a follower on CAN 36, drawn in the 3D view as an offset mirror
    // of the elevator 0.5 m to the +Y side (the far rail of the carriage). On a real
    // two-motor gearbox also set motorModel = DCMotor.getKrakenX60(2) and re-characterize
    // kG; this demo keeps the single-motor plant so its tuning matches the other examples.
    s.followerCanId = FOLLOWER_CAN_ID;
    s.followerOpposed = true;
    s.followerVisualOffset = new edu.wpi.first.math.geometry.Translation3d(0, 0.5, 0);
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {4, 3}; // 12:1
    s.drumCircumference = Inches.of(0.25 * 22); // 22T #25-chain sprocket
    s.carriageMass = Kilograms.of(6.0);
    s.minHeight = Meters.of(0);
    s.maxHeight = Meters.of(1.5);
    s.startingHeight = Meters.of(0.1);
    // Free speed at the drum is ~1.16 m/s (100 rps / 12 × 0.1397 m); stay below it.
    s.maxVelocity = MetersPerSecond.of(0.9);
    s.maxAcceleration = MetersPerSecondPerSecond.of(2.0);
    s.kG = Volts.of(0.19); // m·g·r / (gearing · kT / R) for the plant above
    s.visualPose3d = new edu.wpi.first.math.geometry.Pose3d(0.25, 0.25, 0,
        edu.wpi.first.math.geometry.Rotation3d.kZero); // front-left corner, 3D view
    return s;
  }
}
