package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.MechanismMotor;
import org.frc5010.common.mechanisms.YamsElevator;

/**
 * Example LQR elevator: one Kraken X60 on a TalonFX (CAN 21), 12:1 gearbox driving a
 * 22-tooth #25 sprocket, 6 kg carriage, 1.5 m of travel.
 *
 * <p>Robot-specific numbers live here; all control logic is in the common
 * {@link YamsElevator}. Copy this class and replace the constants for your robot.
 * kG (0.19 V) was computed from the carriage mass, drum radius, gearing, and Kraken
 * motor constants — on a real robot, characterize it with {@code sysId()} instead.
 */
public class ExampleElevator extends YamsElevator {

  /** CAN ID of the elevator TalonFX. */
  public static final int CAN_ID = 21;

  public ExampleElevator() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleElevator";
    s.vendor = MechanismMotor.Vendor.TALON_FX;
    s.canId = CAN_ID;
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
    return s;
  }
}
