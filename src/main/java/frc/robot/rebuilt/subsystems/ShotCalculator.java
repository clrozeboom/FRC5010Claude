package frc.robot.rebuilt.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.rebuilt.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Moving-shot solver for the launcher — ported from the 2026 "Rebuilt"
 * {@code ShotCalculator}.
 *
 * <p>Holds three interpolating lookup tables keyed on distance-to-target (meters): hood
 * angle (authored in <em>legacy</em> degrees, converted to the corrected hood frame via
 * {@link Constants.Launcher#offsetLegacyHoodAngleDegrees(double)}), flywheel speed, and
 * time-of-flight. The default tables ({@link #createDefaultTables()}) are copied verbatim
 * from the source (18 hood/speed rows 1.42–13.02 m, 12 TOF rows). {@code flywheelMultiplier}
 * defaults to 1.05 and is operator-adjustable ±0.01.
 *
 * <p>Each shot:
 * <ol>
 *   <li>extrapolates the robot pose forward by {@code phaseDelay} using linear field-frame
 *       extrapolation (matching the source — the drive already discretizes for curvature);
 *   <li>solves for a <em>virtual target</em> via fixed-point iteration: lead the real target
 *       by the robot's field velocity × time-of-flight, re-looking-up the TOF at the updated
 *       distance until it converges (this condenses the source's bisection-based
 *       {@code TurretControlPhysics.solve}, which is hardware-tuned, into the equivalent
 *       shoot-on-the-move geometry);
 *   <li>returns turret angle (turret-local), hood angle, and flywheel speed, respecting the
 *       turret's ±165° limit.
 * </ol>
 */
public class ShotCalculator {

  private static final double LOOP_PERIOD_SECS = 0.02;

  private static ShotCalculator instance;

  /** Operator-adjustable flywheel speed multiplier (default 1.05, ±0.01 via POV/bumpers). */
  public static double flywheelMultiplier = 1.05;

  private final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  private double minDistance = 0.7;
  private double maxDistance = 100.0;
  private double phaseDelaySeconds = 0.03;

  private final Rotation2d minTurretAngle = Rotation2d.fromDegrees(-165.0);
  private final Rotation2d maxTurretAngle = Rotation2d.fromDegrees(165.0);

  private ShootingParameters latestParameters;
  private Rotation2d lastTurretAngle;
  private double lastHoodAngleRad = Double.NaN;

  public ShotCalculator() {
    applyDefaultTables();
  }

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  /** Computed shooting parameters for one cycle. Hood angle is in the corrected (real) frame. */
  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocityRadPerSec,
      double hoodAngleDegrees,
      double hoodVelocityDegPerSec,
      double flywheelSpeed,
      double distanceToVirtualTargetMeters,
      Translation2d virtualTargetFieldPos) {}

  // ── lookup tables ──────────────────────────────────────────────────────────

  private static Rotation2d legacyHoodAngle(double legacyDegrees) {
    return Rotation2d.fromDegrees(Constants.Launcher.offsetLegacyHoodAngleDegrees(legacyDegrees));
  }

  /** Builds and installs the default shot tables (copied verbatim from the source robot). */
  public void applyDefaultTables() {
    hoodAngleMap.clear();
    hoodAngleMap.put(1.4156, legacyHoodAngle(33.00));
    hoodAngleMap.put(2.0796, legacyHoodAngle(35.03));
    hoodAngleMap.put(2.3645, legacyHoodAngle(36.91));
    hoodAngleMap.put(2.7649, legacyHoodAngle(39.13));
    hoodAngleMap.put(3.0481, legacyHoodAngle(41.87));
    hoodAngleMap.put(3.2195, legacyHoodAngle(43.25));
    hoodAngleMap.put(3.5309, legacyHoodAngle(44.69));
    hoodAngleMap.put(3.7474, legacyHoodAngle(45.55));
    hoodAngleMap.put(3.9269, legacyHoodAngle(46.17));
    hoodAngleMap.put(4.3173, legacyHoodAngle(48.64));
    hoodAngleMap.put(4.5403, legacyHoodAngle(49.80));
    hoodAngleMap.put(4.8099, legacyHoodAngle(50.44));
    hoodAngleMap.put(5.2494, legacyHoodAngle(51.14));
    hoodAngleMap.put(5.2859, legacyHoodAngle(51.20));
    hoodAngleMap.put(5.7789, legacyHoodAngle(51.55));
    hoodAngleMap.put(6.3521, legacyHoodAngle(53.04));
    hoodAngleMap.put(10.9907, legacyHoodAngle(51.84));
    hoodAngleMap.put(13.0240, legacyHoodAngle(55.00));

    flywheelSpeedMap.clear();
    flywheelSpeedMap.put(1.4156, 88.00);
    flywheelSpeedMap.put(2.0796, 88.41);
    flywheelSpeedMap.put(2.3645, 94.58);
    flywheelSpeedMap.put(2.7649, 96.83);
    flywheelSpeedMap.put(3.0481, 98.25);
    flywheelSpeedMap.put(3.2195, 99.25);
    flywheelSpeedMap.put(3.5309, 102.69);
    flywheelSpeedMap.put(3.7474, 104.64);
    flywheelSpeedMap.put(3.9269, 106.00);
    flywheelSpeedMap.put(4.3173, 108.00);
    flywheelSpeedMap.put(4.5403, 110.00);
    flywheelSpeedMap.put(4.8099, 112.18);
    flywheelSpeedMap.put(5.2494, 115.97);
    flywheelSpeedMap.put(5.2859, 114.77);
    flywheelSpeedMap.put(5.7789, 119.21);
    flywheelSpeedMap.put(6.3521, 125.75);
    flywheelSpeedMap.put(10.9907, 156.09);
    flywheelSpeedMap.put(13.0240, 173.00);

    timeOfFlightMap.clear();
    timeOfFlightMap.put(1.5090, 0.8768);
    timeOfFlightMap.put(1.7908, 0.8932);
    timeOfFlightMap.put(2.8011, 0.9517);
    timeOfFlightMap.put(2.9721, 0.9616);
    timeOfFlightMap.put(3.6098, 0.9985);
    timeOfFlightMap.put(3.6990, 1.0037);
    timeOfFlightMap.put(4.1380, 1.0291);
    timeOfFlightMap.put(4.4120, 1.0450);
    timeOfFlightMap.put(5.0046, 1.0793);
    timeOfFlightMap.put(5.3802, 1.1011);
    timeOfFlightMap.put(5.4947, 1.1077);
    timeOfFlightMap.put(6.7420, 1.1800);

    minDistance = 0.7;
    maxDistance = 100.0;
    phaseDelaySeconds = 0.03;
  }

  /** Interpolated hood angle (corrected/real degrees) for a distance, or NaN if unavailable. */
  public double getLookupHoodAngleDegrees(double distanceMeters) {
    Rotation2d value = hoodAngleMap.get(distanceMeters);
    return value != null ? value.getDegrees() : Double.NaN;
  }

  /** Interpolated flywheel speed for a distance (before the multiplier), or NaN if unavailable. */
  public double getLookupFlywheelSpeed(double distanceMeters) {
    Double value = flywheelSpeedMap.get(distanceMeters);
    return value != null ? value : Double.NaN;
  }

  /** Interpolated time-of-flight (seconds) for a distance. */
  public double getTimeOfFlightSeconds(double distanceMeters) {
    Double value = timeOfFlightMap.get(distanceMeters);
    return value != null ? value : 0.0;
  }

  public static void incrementFlywheelMultiplier(double amount) {
    flywheelMultiplier += amount;
  }

  public static double getFlywheelMultiplier() {
    return flywheelMultiplier;
  }

  // ── solve ──────────────────────────────────────────────────────────────────

  /** Clears the per-cycle cache; call once per loop before {@link #getParameters}. */
  public void clearShootingParameters() {
    latestParameters = null;
  }

  /**
   * Solves the shot for this cycle. Caches the result until {@link #clearShootingParameters()}.
   *
   * @param turretRelativePosition turret centre offset from robot centre, robot frame (meters)
   * @param turretRelativeAngle    turret zero heading relative to the robot
   * @param robotPose              current estimated field pose
   * @param fieldVelocity          field-relative chassis velocity (for shoot-on-the-move)
   * @param targetFieldPosition    the aim point on the field (e.g. the hub top-centre)
   */
  public ShootingParameters getParameters(
      Translation2d turretRelativePosition,
      Rotation2d turretRelativeAngle,
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      Translation2d targetFieldPosition) {
    if (latestParameters != null) {
      return latestParameters;
    }

    // 1. Phase-delay the pose with linear field-frame extrapolation.
    Pose2d phaseDelayedPose = linearExtrapolate(robotPose, fieldVelocity, phaseDelaySeconds);
    Translation2d turretPos =
        phaseDelayedPose
            .transformBy(
                new Transform2d(
                    turretRelativePosition.getX(),
                    turretRelativePosition.getY(),
                    turretRelativeAngle))
            .getTranslation();

    // 2. Fixed-point virtual-target iteration: lead the target by velocity × time-of-flight.
    Translation2d velocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    Translation2d virtualTarget = targetFieldPosition;
    double distance = turretPos.getDistance(virtualTarget);
    for (int i = 0; i < 5; i++) {
      double tof = getTimeOfFlightSeconds(clampDistance(distance));
      virtualTarget = targetFieldPosition.minus(velocity.times(tof));
      distance = turretPos.getDistance(virtualTarget);
    }
    double effectiveDistance = clampDistance(distance);

    // 3. Turret-local heading to the virtual target.
    Rotation2d fieldAngleToTarget = virtualTarget.minus(turretPos).getAngle();
    Rotation2d turretLocal =
        fieldAngleToTarget.minus(phaseDelayedPose.getRotation()).minus(turretRelativeAngle);
    boolean withinTravel =
        turretLocal.getRadians() >= minTurretAngle.getRadians()
            && turretLocal.getRadians() <= maxTurretAngle.getRadians();
    Rotation2d clampedTurret = clampTurret(turretLocal);

    Rotation2d hoodSetpoint = hoodAngleMap.get(effectiveDistance);
    double hoodRad = hoodSetpoint != null ? hoodSetpoint.getRadians() : 0.0;
    Double flywheelTable = flywheelSpeedMap.get(effectiveDistance);
    double flywheelSpeed = (flywheelTable != null ? flywheelTable : 0.0) * flywheelMultiplier;

    // 4. Filtered turret/hood velocity feedforward (single-step difference, as in the source).
    if (lastTurretAngle == null) lastTurretAngle = clampedTurret;
    if (Double.isNaN(lastHoodAngleRad)) lastHoodAngleRad = hoodRad;
    double turretVel = clampedTurret.minus(lastTurretAngle).getRadians() / LOOP_PERIOD_SECS;
    double hoodVelDeg = Math.toDegrees(hoodRad - lastHoodAngleRad) / LOOP_PERIOD_SECS;
    lastTurretAngle = clampedTurret;
    lastHoodAngleRad = hoodRad;

    latestParameters =
        new ShootingParameters(
            withinTravel && hoodSetpoint != null,
            clampedTurret,
            turretVel,
            Math.toDegrees(hoodRad),
            hoodVelDeg,
            flywheelSpeed,
            effectiveDistance,
            virtualTarget);

    Logger.recordOutput("ShotCalculator/DistanceToVirtualTarget", effectiveDistance);
    Logger.recordOutput("ShotCalculator/TurretAngleDeg", clampedTurret.getDegrees());
    Logger.recordOutput("ShotCalculator/HoodAngleDeg", Math.toDegrees(hoodRad));
    Logger.recordOutput("ShotCalculator/FlywheelSpeed", flywheelSpeed);
    Logger.recordOutput("ShotCalculator/IsValid", latestParameters.isValid());
    Logger.recordOutput(
        "ShotCalculator/VirtualTarget", new Pose2d(virtualTarget, fieldAngleToTarget));

    return latestParameters;
  }

  private double clampDistance(double distance) {
    return Math.max(minDistance, Math.min(maxDistance, distance));
  }

  private Rotation2d clampTurret(Rotation2d angle) {
    double rad =
        Math.max(minTurretAngle.getRadians(), Math.min(maxTurretAngle.getRadians(), angle.getRadians()));
    return Rotation2d.fromRadians(rad);
  }

  /**
   * Linear field-frame pose extrapolation: position advances by v·dt, heading by ω·dt. Avoids
   * {@code Pose2d.exp}'s curvature term because the drive already discretizes for curvature.
   */
  private static Pose2d linearExtrapolate(Pose2d pose, ChassisSpeeds fieldVelocity, double dt) {
    return new Pose2d(
        pose.getTranslation()
            .plus(
                new Translation2d(
                    fieldVelocity.vxMetersPerSecond * dt, fieldVelocity.vyMetersPerSecond * dt)),
        pose.getRotation().plus(Rotation2d.fromRadians(fieldVelocity.omegaRadiansPerSecond * dt)));
  }
}
