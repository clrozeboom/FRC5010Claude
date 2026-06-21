package frc.robot.rebuilt;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Tuning constants for the ported 2026 "Rebuilt" robot. Values are copied verbatim from the
 * source competition robot's {@code frc.robot.rebuilt.Constants}; the climb subsystem is out
 * of scope for this port and its constants are omitted.
 */
public class Constants {
  /** Subsystem telemetry names. */
  public static final String INDEXER = "Indexer";
  public static final String INTAKE = "Intake";
  public static final String LAUNCHER = "Launcher";

  /** Launcher (flywheel + hood + turret) constants. */
  public static class Launcher {
    public static final double SHOOTER_TOLERANCE_RPM = 50.0;
    public static final double HOOD_ANGLE_TOLERANCE_DEGREES = 3.5;
    public static final double TURRET_ANGLE_TOLERANCE_DEGREES = 5.0;
    public static final Angle HOPPER_EXTENSION_BUFFER_BEFORE_AIM = Degrees.of(10);
    public static final double HOOD_STALL_CURRENT_THRESHOLD = 20.0;

    public static final double HOOD_LEGACY_START_ANGLE_DEGREES = 30.0;
    public static final double HOOD_CORRECTED_START_ANGLE_DEGREES = 12.723;
    public static final double HOOD_CORRECTED_END_ANGLE_DEGREES = 45.723;
    public static final double HOOD_CALIBRATION_OFFSET_DEGREES =
        HOOD_CORRECTED_START_ANGLE_DEGREES - HOOD_LEGACY_START_ANGLE_DEGREES;

    /**
     * Converts a hood angle authored in the mechanical "legacy" frame (which the shot tables
     * use) to the real, corrected hood frame. Port this exactly or every shot-table value is
     * off by {@link #HOOD_CALIBRATION_OFFSET_DEGREES}.
     */
    public static double offsetLegacyHoodAngleDegrees(double legacyAngleDegrees) {
      return legacyAngleDegrees + HOOD_CALIBRATION_OFFSET_DEGREES;
    }

    public static final Angle LOW_HOOD_ANGLE = Degrees.of(offsetLegacyHoodAngleDegrees(31.0));
    public static final AngularVelocity LOW_FLYWHEEL_RPM = RotationsPerSecond.of(1);

    public static final Angle HUB_HOOD_ANGLE = LOW_HOOD_ANGLE;
    public static final AngularVelocity HUB_FLYWHEEL_RPM = RotationsPerSecond.of(1.25);

    public static final Angle TOWER_HOOD_ANGLE = Degrees.of(offsetLegacyHoodAngleDegrees(40.0));
    public static final AngularVelocity TOWER_FLYWHEEL_RPM = RotationsPerSecond.of(1.5);

    public static final Angle TURRET_FORWARD = Degrees.of(0);
    public static final AngularVelocity FWD_FLYWHEEL_RPM = LOW_FLYWHEEL_RPM;
    public static final Angle FWD_HOOD_ANGLE = LOW_HOOD_ANGLE;
  }

  /** Indexer (spindexer + transfer) speeds. */
  public static class Indexer {
    public static final double SPINDEXER_SPEED = 0.90;
    public static final double TRANSFER_SPEED = 1.0;
    public static final double TRANSFER_CHURN = 0.25;
  }

  /** Intake (spintakes + hopper arm) speeds and homing thresholds. */
  public static class Intake {
    public static final double HOPPER_GO_OUT = -0.3;
    public static final double HOPPER_GO_IN = 0.2;
    public static final double INTAKE_IN = 1.0;
    public static final double INTAKE_INNER_IN = 0.3;
    public static final double INTAKE_AUTO = 1.0;
    public static final double INTAKE_DEADZONE = 0.25;
    public static final double INTAKE_CHURN = -0.25;
    public static final double INTAKE_MAX_IN = 0.9;
    public static final double INTAKE_MAX_OUT = -0.9;
    public static final double HOPPER_ANGLE_TOLERANCE = 3;
    public static final double HOPPER_STALL_TIME = 0.3;
    public static final Angle HOPPER_RETRACTED_ANGLE = Degrees.of(120);
    public static final Angle HOPPER_DEPLOYED_ANGLE = Degrees.of(0);
    public static final Angle HOPPER_ANGLED = Degrees.of(45);
    public static final double HOPPER_STALL_CURRENT_THRESHOLD = 80.0;
    public static final double HOPPER_MOVING_VELOCITY_THRESHOLD = 1.0;
    public static final double HOPPER_DEPLOY_STOP_REZERO_MAX_ANGLE = 20.0;
    /** Degrees — if hopper goes past 0, auto-rezero. */
    public static final double HOPPER_AUTO_REZERO_THRESHOLD = -1.0;
    /** Duty cycle for first-deploy zeroing nudge. */
    public static final double HOPPER_FIRST_DEPLOY_DUTY = -0.2;
    /** Duty cycle for normal deploy nudge after PID. */
    public static final double HOPPER_DEPLOY_NUDGE_DUTY = -0.35;
  }
}
