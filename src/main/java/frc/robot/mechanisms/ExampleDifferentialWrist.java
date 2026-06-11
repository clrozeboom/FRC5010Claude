package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.MechanismMotor;
import org.frc5010.common.mechanisms.YamsDifferentialMechanism;

/**
 * Example differential wrist: two Kraken X60s on TalonFXs (CAN 27 left, CAN 28 right),
 * 60:1, driving a tilt+twist differential gear assembly.
 *
 * <p>Per-motor profiled PID (the YAMS LQR does not model the coupled differential).
 * Robot-specific numbers live here; control logic is in the common
 * {@link YamsDifferentialMechanism}.
 */
public class ExampleDifferentialWrist extends YamsDifferentialMechanism {

  /** CAN ID of the left wrist TalonFX. */
  public static final int LEFT_CAN_ID = 27;
  /** CAN ID of the right wrist TalonFX. */
  public static final int RIGHT_CAN_ID = 28;

  public ExampleDifferentialWrist() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleDiffWrist";
    s.vendor = MechanismMotor.Vendor.TALON_FX;
    s.leftCanId = LEFT_CAN_ID;
    s.rightCanId = RIGHT_CAN_ID;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {3, 4, 5}; // 60:1
    s.length = Meters.of(0.3);
    s.mass = Kilograms.of(1.8);
    s.startingTilt = Degrees.of(90);
    s.startingTwist = Degrees.of(0);
    return s;
  }
}
