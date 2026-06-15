package org.frc5010.examples.mechanisms;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.mechanisms.Flywheel;

/**
 * Example LQR shooter flywheel: one Kraken X60 on a TalonFX (CAN 24) direct-driving a
 * 4-inch, 1.5 kg wheel.
 *
 * <p>Velocity-only LQR — the {@code LinearSystemLoop} provides plant-inversion
 * feedforward, so no kV is configured. Robot-specific numbers live here; control logic
 * is in the common {@link Flywheel}.
 */
public class ExampleShooter extends Flywheel {

  /** CAN ID of the shooter TalonFX. */
  public static final int CAN_ID = 24;

  public ExampleShooter() {
    super(settings());
  }

  private static Settings settings() {
    var s = new Settings();
    s.name = "ExampleShooter";
    s.canId = CAN_ID;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {1.0}; // direct drive
    s.diameter = Inches.of(4);
    s.mass = Kilograms.of(1.5);
    s.visualPose3d = new edu.wpi.first.math.geometry.Pose3d(-0.1, 0.25, 0.6,
        edu.wpi.first.math.geometry.Rotation3d.kZero); // wheel in the side-view plane, 3D view
    return s;
  }
}
