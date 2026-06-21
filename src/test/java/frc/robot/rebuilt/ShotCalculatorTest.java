package frc.robot.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.rebuilt.subsystems.ShotCalculator;
import frc.robot.rebuilt.subsystems.ShotCalculator.ShootingParameters;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Layer 1 — deterministic unit tests for {@link ShotCalculator} against the default shot
 * tables. No hardware, no HAL: pure interpolation and shoot-on-the-move geometry.
 */
class ShotCalculatorTest {

  private ShotCalculator calc;

  @BeforeEach
  void setup() {
    calc = new ShotCalculator();
    ShotCalculator.flywheelMultiplier = 1.05; // reset shared static
  }

  // ── lookup tables ──────────────────────────────────────────────────────────

  @Test
  void hoodTableUsesCorrectedFrameAndExactRowsMatch() {
    // 33° legacy at 1.4156 m → corrected = 33 + (12.723 − 30) = 15.723°.
    assertEquals(15.723, calc.getLookupHoodAngleDegrees(1.4156), 1e-3);
    // 55° legacy at the far row.
    assertEquals(
        Constants.Launcher.offsetLegacyHoodAngleDegrees(55.00),
        calc.getLookupHoodAngleDegrees(13.0240),
        1e-3);
  }

  @Test
  void hoodAngleIncreasesMonotonicallyWithDistanceOverNearRows() {
    double near = calc.getLookupHoodAngleDegrees(1.5);
    double mid = calc.getLookupHoodAngleDegrees(3.0);
    double far = calc.getLookupHoodAngleDegrees(6.0);
    assertTrue(near < mid && mid < far, "hood angle rises with distance: " + near + "," + mid + "," + far);
  }

  @Test
  void flywheelLookupInterpolatesBetweenRows() {
    // Between 1.4156 (88.00) and 2.0796 (88.41) — interpolated value must lie in [88, 88.41].
    double v = calc.getLookupFlywheelSpeed(1.75);
    assertTrue(v >= 88.0 && v <= 88.41, "interpolated flywheel speed in range, was " + v);
  }

  // ── stationary solve ───────────────────────────────────────────────────────

  @Test
  void stationaryShotAimsTurretStraightAtTargetAhead() {
    // Robot at origin facing +X; target 3 m straight ahead. Turret offset ~0.
    Pose2d pose = new Pose2d(0, 0, Rotation2d.kZero);
    Translation2d target = new Translation2d(3.0, 0.0);
    ShootingParameters p =
        calc.getParameters(new Translation2d(), Rotation2d.kZero, pose, new ChassisSpeeds(), target);

    assertTrue(p.isValid(), "a 3 m shot dead ahead must be valid");
    assertEquals(0.0, p.turretAngle().getDegrees(), 1.0, "turret aims straight ahead");
    assertEquals(3.0, p.distanceToVirtualTargetMeters(), 0.05, "distance ≈ 3 m when stationary");
    // Hood/flywheel come from the tables at ~3 m.
    assertTrue(p.hoodAngleDegrees() > 0, "hood angle set from table");
    assertTrue(p.flywheelSpeed() > 0, "flywheel speed set from table");
  }

  @Test
  void targetToTheLeftRotatesTurretPositive() {
    // Target at +90° (to the robot's left) → turret-local heading ≈ +90°.
    Pose2d pose = new Pose2d(0, 0, Rotation2d.kZero);
    Translation2d target = new Translation2d(0.0, 3.0);
    ShootingParameters p =
        calc.getParameters(new Translation2d(), Rotation2d.kZero, pose, new ChassisSpeeds(), target);
    assertEquals(90.0, p.turretAngle().getDegrees(), 2.0);
  }

  @Test
  void turretLimitMakesShotInvalidBehindTheRobot() {
    // Target directly behind (−X) → turret-local heading 180°, outside ±165° → invalid.
    Pose2d pose = new Pose2d(0, 0, Rotation2d.kZero);
    Translation2d target = new Translation2d(-3.0, 0.0);
    ShootingParameters p =
        calc.getParameters(new Translation2d(), Rotation2d.kZero, pose, new ChassisSpeeds(), target);
    assertFalse(p.isValid(), "a target outside the turret's ±165° travel must be invalid");
  }

  // ── shoot on the move ──────────────────────────────────────────────────────

  @Test
  void movingTowardTargetLeadsTheVirtualTargetBackTowardTheRobot() {
    // Driving +X toward a target at +5 m: the virtual target is pulled closer (lead the shot),
    // so the effective distance is shorter than the static 5 m.
    Pose2d pose = new Pose2d(0, 0, Rotation2d.kZero);
    Translation2d target = new Translation2d(5.0, 0.0);
    ChassisSpeeds movingForward = new ChassisSpeeds(2.0, 0.0, 0.0);

    calc.clearShootingParameters();
    ShootingParameters moving =
        calc.getParameters(new Translation2d(), Rotation2d.kZero, pose, movingForward, target);

    // TOF at ~5 m is ~1.08 s, lead ≈ 2 m → virtual target near x≈3 m.
    assertTrue(
        moving.distanceToVirtualTargetMeters() < 4.5,
        "moving toward target shortens effective distance, was "
            + moving.distanceToVirtualTargetMeters());
    assertTrue(moving.virtualTargetFieldPos().getX() < 5.0, "virtual target led back toward robot");
  }

  @Test
  void flywheelMultiplierScalesCommandedSpeed() {
    Pose2d pose = new Pose2d(0, 0, Rotation2d.kZero);
    Translation2d target = new Translation2d(3.0, 0.0);

    ShotCalculator.flywheelMultiplier = 1.0;
    calc.clearShootingParameters();
    double atOne =
        calc.getParameters(new Translation2d(), Rotation2d.kZero, pose, new ChassisSpeeds(), target)
            .flywheelSpeed();

    ShotCalculator.flywheelMultiplier = 1.10;
    calc.clearShootingParameters();
    double atTenPct =
        calc.getParameters(new Translation2d(), Rotation2d.kZero, pose, new ChassisSpeeds(), target)
            .flywheelSpeed();

    assertEquals(atOne * 1.10, atTenPct, 1e-6, "multiplier scales the commanded flywheel speed");
  }
}
