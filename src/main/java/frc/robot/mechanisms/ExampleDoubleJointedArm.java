package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.MechanismMotor;
import org.frc5010.common.mechanisms.YamsDoubleJointedArm;

/**
 * Example double-jointed arm: two Kraken X60s on TalonFXs (CAN 25 shoulder, CAN 26
 * elbow), 60:1 per joint, 0.6 m / 2.5 kg lower segment and 0.5 m / 1.5 kg upper segment.
 *
 * <p>Per-joint profiled PID (the YAMS LQR does not model coupled two-joint dynamics).
 * Robot-specific numbers live here; control logic is in the common
 * {@link YamsDoubleJointedArm}.
 */
public class ExampleDoubleJointedArm extends YamsDoubleJointedArm {

  /** CAN ID of the shoulder (lower joint) TalonFX. */
  public static final int LOWER_CAN_ID = 25;
  /** CAN ID of the elbow (upper joint) TalonFX. */
  public static final int UPPER_CAN_ID = 26;

  public ExampleDoubleJointedArm() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleDJArm";
    s.vendor = MechanismMotor.Vendor.TALON_FX;
    s.motorModel = DCMotor.getKrakenX60(1);

    s.lowerJoint.canId = LOWER_CAN_ID;
    s.lowerJoint.gearReductionStages = new double[] {3, 4, 5}; // 60:1
    s.lowerJoint.length = Meters.of(0.6);
    s.lowerJoint.mass = Kilograms.of(2.5);
    s.lowerJoint.startingAngle = Degrees.of(45);

    s.upperJoint.canId = UPPER_CAN_ID;
    s.upperJoint.gearReductionStages = new double[] {3, 4, 5}; // 60:1
    s.upperJoint.length = Meters.of(0.5);
    s.upperJoint.mass = Kilograms.of(1.5);
    s.upperJoint.startingAngle = Degrees.of(45);
    return s;
  }
}
