package frc.robot.mechanisms;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.Elevator;

/**
 * The same elevator as {@link ExampleElevator}, but the LQR plant is built from
 * <b>measured</b> kV/kA (a SysId characterization) instead of the carriage mass
 * (TalonFX, CAN 35).
 *
 * <h2>Why characterize instead of using mass?</h2>
 *
 * An LQR doesn't actually need to know your elevator's mass — it needs to know how the
 * mechanism <em>responds to voltage</em> (the "plant"). Mass, gearing, and the motor
 * datasheet are one way to predict that response on paper. A SysId test is a way to
 * <em>measure</em> it:
 *
 * <ul>
 *   <li><b>kV</b> (volts per m/s) — how much voltage it takes to hold a steady speed.</li>
 *   <li><b>kA</b> (volts per m/s²) — how much extra voltage it takes to accelerate.</li>
 * </ul>
 *
 * Two values, measured in two minutes with the {@code sysId()} command, fully define
 * the linear plant — and they automatically include everything the paper model gets
 * wrong: gearbox friction, chain drag, the carriage being heavier than CAD said,
 * battery sag. <b>If you can't weigh the carriage, you don't have to:</b> its effective
 * mass is implied by the measured kA (you can even back it out:
 * m = kA·G·kT / (R·r) ≈ 0.019 × 12 × 0.0194 / (0.0328 × 0.0222) ≈ 6 kg for these
 * numbers — matching the 6 kg the LQR twin uses).
 *
 * <h2>Sanity check the measurement</h2>
 *
 * kV is fixed by the motor and gearing alone: theoretical kV = 12 V ÷ free speed
 * (here 12 ÷ 1.16 m/s ≈ 10.3). If your <em>measured</em> kV is much higher, the
 * mechanism has real losses — which is exactly why the measured plant beats the paper
 * one. If it's <em>lower</em>, re-check your units (SysId set to meters?) and gearing.
 *
 * <p>In simulation the mechanism is lossless, so the values below are the theoretical
 * ones and behave identically to {@link ExampleElevator}'s mass-based plant — the
 * functional test verifies that. On a real robot, replace them with the numbers from
 * the SysId tool and the controller will fit reality better than any mass estimate.
 */
public class ExampleCharacterizedElevator extends Elevator {

  /** CAN ID of the elevator TalonFX. */
  public static final int CAN_ID = 35;

  public ExampleCharacterizedElevator() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleCharacterizedElevator";
    s.canId = CAN_ID;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {4, 3}; // 12:1
    s.drumCircumference = Inches.of(0.25 * 22);
    // Still needed for the SIMULATION plant (and hard-limit physics) — but the
    // CONTROLLER no longer uses it once characterizedKv/Ka are set below.
    s.carriageMass = Kilograms.of(6.0);
    s.minHeight = Meters.of(0);
    s.maxHeight = Meters.of(1.5);
    s.startingHeight = Meters.of(0.1);
    s.maxVelocity = MetersPerSecond.of(0.9);
    s.maxAcceleration = MetersPerSecondPerSecond.of(2.0);
    s.kG = Volts.of(0.19); // gravity is not part of the linear plant — still from SysId
    s.visualPosition = new edu.wpi.first.math.geometry.Translation2d(0.85, 0.0); // spot on the RobotMechanisms overlay
    s.visualPose3d = new edu.wpi.first.math.geometry.Pose3d(-0.25, 0.25, 0,
        edu.wpi.first.math.geometry.Rotation3d.kZero); // back-left corner, 3D view

    // From the SysId tool (units: meters). These two numbers REPLACE the mass-based
    // plant — see the class javadoc for why.
    s.characterizedKv = 10.3; // V per m/s   (theoretical: 12 V ÷ 1.16 m/s free speed)
    s.characterizedKa = 0.019; // V per m/s² (implies the ~6 kg carriage)
    return s;
  }
}
