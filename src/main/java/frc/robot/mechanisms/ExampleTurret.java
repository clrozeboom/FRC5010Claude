package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.MechanismMotor;
import org.frc5010.common.mechanisms.YamsPivot;

/**
 * Example LQR turret: one Kraken X60 on a TalonFX (CAN 23), 40:1 ring-gear drive,
 * 0.5 kg·m² rotating assembly, ±180° wiring-limited travel.
 *
 * <p>A turret is the canonical {@link YamsPivot}: gravity-free rotation, so the
 * ARM-type LQR plant applies with no gravity feedforward. Robot-specific numbers live
 * here; control logic is in the common {@link YamsPivot}.
 */
public class ExampleTurret extends YamsPivot {

  /** CAN ID of the turret TalonFX. */
  public static final int CAN_ID = 23;

  public ExampleTurret() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleTurret";
    s.vendor = MechanismMotor.Vendor.TALON_FX;
    s.canId = CAN_ID;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {10, 4}; // 40:1
    s.moi = KilogramSquareMeters.of(0.5); // turret plate + shooter on top
    s.minAngle = Degrees.of(-180);
    s.maxAngle = Degrees.of(180);
    s.startingAngle = Degrees.of(0);
    s.maxVelocity = DegreesPerSecond.of(360);
    s.maxAcceleration = DegreesPerSecondPerSecond.of(720);
    return s;
  }
}
