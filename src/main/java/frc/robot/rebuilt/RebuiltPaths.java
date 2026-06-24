package frc.robot.rebuilt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Native BLine path definitions for all RebuiltRobot autos.
 *
 * <p>Replaces runtime PathPlanner {@code .path} file loading with hardcoded anchor-waypoint
 * paths derived from the same PathPlanner anchor coordinates. Benefits over
 * {@link org.frc5010.common.drive.swerve.auto.PathPlannerToBLine}:
 * <ul>
 *   <li>No filesystem I/O at robot startup.</li>
 *   <li>Simpler geometry (anchor points only — no Bezier sub-sampling).</li>
 *   <li>BLine default 0.45 m handoff radius everywhere — BLine's speed profiler already slows
 *       the robot enough to round each corner safely. The 0.10 m "sharp corner" approach was
 *       discarded after simulation showed it required ~15 g centripetal acceleration at cruise
 *       speed, causing the robot to overshoot trench-exit corners and clip the inner wall.</li>
 * </ul>
 *
 * <p>Trench safety limits (applied to all anchor/waypoint Y positions):
 * <ul>
 *   <li>Right trench: robot centre must stay Y &lt; 1.064 m (inner wall at Y=1.279 − 0.215 m bumper)</li>
 *   <li>Left trench: robot centre must stay Y &gt; 7.146 m (inner wall at Y=6.931 + 0.215 m bumper)</li>
 * </ul>
 *
 * <p>Element ordering inside each path: {@link Waypoint}/{@link TranslationTarget} (position
 * elements) alternate with {@link RotationTarget}/{@link EventTrigger} (non-position elements).
 * A non-position element fires at {@code t_ratio} along the segment from the preceding position
 * element to the following position element.
 */
public final class RebuiltPaths {

  private static final Map<String, Path> PATHS = buildAll();

  /**
   * Returns a fresh copy of the named native BLine path (copy so each call gets independent
   * flip/mirror state).
   *
   * @param name PathPlanner path name (without {@code .path} extension)
   * @throws RuntimeException if the name is not registered
   */
  public static Path get(String name) {
    Path p = PATHS.get(name);
    if (p == null) throw new RuntimeException("Unknown path: '" + name + "'");
    return p.copy();
  }

  /**
   * Returns the hard-coded blue-alliance starting {@link Pose2d} for the named path
   * (first waypoint + ideal starting heading).
   */
  public static Pose2d startPose(String name) {
    Path p = PATHS.get(name);
    if (p == null) throw new RuntimeException("Unknown path: '" + name + "'");
    return p.getStartPose();
  }

  // ── element shorthands ──────────────────────────────────────────────────────

  private static Waypoint wp(double x, double y, double deg) {
    return new Waypoint(new Translation2d(x, y), Rotation2d.fromDegrees(deg));
  }

  private static TranslationTarget tt(double x, double y) {
    return new TranslationTarget(x, y);
  }

  private static RotationTarget rt(double tRatio, double deg) {
    return new RotationTarget(Rotation2d.fromDegrees(deg), tRatio);
  }

  private static EventTrigger et(double tRatio, String key) {
    return new EventTrigger(tRatio, key);
  }

  private static Path path(PathElement... elements) {
    return new Path(List.of(elements));
  }

  // ── path registry ────────────────────────────────────────────────────────────

  @SuppressWarnings("checkstyle:MethodLength")
  private static Map<String, Path> buildAll() {
    Map<String, Path> m = new HashMap<>();

    // ── Left / Orbit-left ───────────────────────────────────────────────────

    // TL-QTRH: (4.36,7.41)[0°] → (6.95,7.26) → (7.61,4.90)[-118°]
    // Events: intakeIntake@seg0+43.1%; rot 0°@seg0+28.4%, rot -106.6°@seg1+9.7%
    m.put("TL-QTRH", path(
        wp(4.358, 7.412, 0.0),
        rt(0.284, 0.0),
        et(0.431, "intakeIntake"),
        tt(6.948, 7.259),
        rt(0.097, -106.6),
        wp(7.614, 4.898, -118.2)
    ));

    // TL-QTRHLong: (4.36,7.41)[0°] → (6.95,7.26) → (7.58,4.14)[-55.2°]
    m.put("TL-QTRHLong", path(
        wp(4.358, 7.412, 0.0),
        rt(0.284, 0.0),
        et(0.431, "intakeIntake"),
        tt(6.948, 7.259),
        rt(0.097, -51.6),
        wp(7.582, 4.144, -55.2)
    ));

    // QTRLong-TL: (7.61,4.90)[-118°] → (6.18,7.32) → (2.72,7.32)[0°]
    // Events: launcherPrep@seg1+88.8%; rot 0°@seg0+74.5%
    m.put("QTRLong-TL", path(
        wp(7.614, 4.898, -118.2),
        rt(0.745, 0.0),
        tt(6.183, 7.324),
        et(0.888, "launcherPrep"),
        wp(2.719, 7.324, 0.0)
    ));

    // TL-CTR-QTR: (2.72,7.32)[0°] → (6.21,7.32) → (6.35,4.67)[-90.2°]
    // Events: intakeIntake@seg0+33.9%; rot 0°@seg0+61.9%, rot -85.3°@seg1+46.6%
    m.put("TL-CTR-QTR", path(
        wp(2.719, 7.324, 0.0),
        et(0.339, "intakeIntake"),
        rt(0.619, 0.0),
        tt(6.205, 7.324),
        rt(0.466, -85.3),
        wp(6.347, 4.669, -90.2)
    ));

    // QTRL-MID: (6.35,4.67)[-90.2°] → (6.35,7.41) → (4.36,7.41) → (2.47,7.16)[18.8°]
    // Events: launcherPrep@seg2+20.0%; rot 1.1°@seg0+49.5%, rot 0.1°@seg2+20.0%
    m.put("QTRL-MID", path(
        wp(6.347, 4.669, -90.2),
        rt(0.495, 1.1),
        tt(6.347, 7.412),
        tt(4.358, 7.412),
        rt(0.200, 0.1),
        et(0.200, "launcherPrep"),
        wp(2.467, 7.160, 18.8)
    ));

    // TL-CTL-QTL: (4.41,7.44)[0.1°] → (7.83,7.20) → (8.02,4.48)[-90°]
    // Left trench: anchor at Y=7.200 keeps robot centre safely above 7.146 m inner-wall limit.
    // Events: intakeIntake@seg0+23.2%
    m.put("TL-CTL-QTL", path(
        wp(4.413, 7.444, 0.1),
        et(0.232, "intakeIntake"),
        tt(7.827, 7.200),
        wp(8.019, 4.483, -90.0)
    ));

    // QTL-TL: (8.02,4.48)[-90°] → (7.12,7.22) → (1.14,7.22)[-90.1°]
    // Events: launcherLow@seg0+63.7%; rot 0°@seg1+17.7%
    m.put("QTL-TL", path(
        wp(8.019, 4.483, -90.0),
        et(0.637, "launcherLow"),
        tt(7.123, 7.215),
        rt(0.177, 0.0),
        wp(1.141, 7.215, -90.1)
    ));

    // TLback-CTL-QTL: (3.58,7.46)[0.1°] → (7.83,7.20) → (7.96,4.05)[-90°]
    // Left trench: anchor at Y=7.200 keeps robot centre safely above 7.146 m inner-wall limit.
    // Events: intakeIntake@seg0+23.2%
    m.put("TLback-CTL-QTL", path(
        wp(3.583, 7.455, 0.1),
        et(0.232, "intakeIntake"),
        tt(7.827, 7.200),
        wp(7.958, 4.054, -90.0)
    ));

    // QTLong-TL: (7.96,4.05)[-90°] → (7.12,7.22) → (1.14,7.22)[-90.1°]
    // Events: launcherLow@seg0+63.7%; rot 0°@seg1+17.7%
    m.put("QTLong-TL", path(
        wp(7.958, 4.054, -90.0),
        et(0.637, "launcherLow"),
        tt(7.123, 7.215),
        rt(0.177, 0.0),
        wp(1.141, 7.215, -90.1)
    ));

    // QTLong-TLHP: (7.96,4.05)[-90°] → (7.12,7.22) → (1.14,7.22) → (0.56,5.26)[-90.1°]
    // Events: launcherLow@seg0+63.7%, launcherPrep@seg1+64.5%; rot 0°@seg1+17.7%, 0°@seg1+58.2%, -114°@seg2+38%
    m.put("QTLong-TLHP", path(
        wp(7.958, 4.054, -90.0),
        et(0.637, "launcherLow"),
        tt(7.123, 7.215),
        rt(0.177, 0.0),
        rt(0.582, 0.0),
        et(0.645, "launcherPrep"),
        tt(1.141, 7.215),
        rt(0.380, -114.0),
        wp(0.559, 5.258, -90.1)
    ));

    // TL-CTL-QTL-BL-L: (1.14,7.22)[-90.1°] → (8.06,7.16) → (7.00,4.22) → (5.76,5.46) → (3.19,5.46) → (1.55,5.05)[75.7°]
    // Events: launcherPrep@seg3+80.0%; rot 0°@seg0+50%, -111.6°@seg1+24.6%, -147.8°@seg1+67.4%, 89.7°@seg2+89.7%, 90°@seg3+0%, 90°@seg4+0%
    m.put("TL-CTL-QTL-BL-L", path(
        wp(1.141, 7.215, -90.1),
        rt(0.500, 0.0),
        tt(8.064, 7.158),
        rt(0.246, -111.6),
        rt(0.674, -147.8),
        tt(7.002, 4.221),
        rt(0.897, 89.7),
        tt(5.755, 5.456),
        rt(0.000, 90.0),
        et(0.800, "launcherPrep"),
        tt(3.189, 5.456),
        rt(0.000, 90.0),
        wp(1.550, 5.045, 75.7)
    ));

    // TLBack-CTL-QTL-BL-L: (3.58,7.46)[0.1°] → (8.06,7.16) → (6.74,3.66) → (5.76,5.46) → (3.19,5.46) → (0.74,5.30) → (0.55,7.31)[90°]
    // Events: many rotations, no explicit events
    m.put("TLBack-CTL-QTL-BL-L", path(
        wp(3.583, 7.455, 0.1),
        rt(0.500, 0.0),
        tt(8.064, 7.158),
        rt(0.246, -111.6),
        rt(0.674, -147.8),
        tt(6.740, 3.655),
        rt(0.897, 89.7),
        tt(5.755, 5.456),
        rt(0.000, 90.0),
        tt(3.189, 5.456),
        rt(0.000, 90.0),
        tt(0.738, 5.302),
        rt(0.855, 130.5),
        wp(0.552, 7.309, 90.0)
    ));

    // TL-QTLCTLSHORT: (4.41,7.44)[0.1°] → (6.95,7.34) → (7.82,5.85) → (7.75,4.88)[-90°]
    // Events: intakeIntake@seg0+60.4%, launcherPrep@seg0+74.1%; rot 0°@seg0+66.7%, -90°@seg1+45.9%
    m.put("TL-QTLCTLSHORT", path(
        wp(4.413, 7.444, 0.1),
        et(0.604, "intakeIntake"),
        rt(0.667, 0.0),
        et(0.741, "launcherPrep"),
        tt(6.945, 7.338),
        rt(0.459, -90.0),
        tt(7.817, 5.849),
        wp(7.752, 4.882, -90.0)
    ));

    // CTL-NWALL (left): (7.75,4.88)[-90°] → (6.64,4.88) → (7.17,6.77)[-90°]
    // Events: launcherPrep@seg0+7.9%; rot -90.7°@seg0+70.5%
    m.put("CTL-NWALL", path(
        wp(7.752, 4.882, -90.0),
        et(0.079, "launcherPrep"),
        rt(0.705, -90.7),
        tt(6.643, 4.882),
        wp(7.169, 6.773, -90.0)
    ));

    // Left3rdSwipe: (7.17,6.77)[-90°] → (5.81,6.18) → (7.73,6.73)[90°]
    // Events: launcherLow@seg0+0.0%; rot -89.2°@seg1+3.1%, 90°@seg1+82.1%
    m.put("Left3rdSwipe", path(
        wp(7.169, 6.773, -90.0),
        et(0.000, "launcherLow"),
        tt(5.805, 6.179),
        rt(0.031, -89.2),
        rt(0.821, 90.0),
        wp(7.734, 6.734, 90.0)
    ));

    // LeftNwall-Tower: (7.73,6.73)[90°] → (5.61,7.42) → (2.88,7.42) → (0.91,6.93) → (0.73,5.39)[-119.4°]
    // Events: launcherLow@seg0+45.0%, launcherPrep@seg1+58.4%; many rotations
    m.put("LeftNwall-Tower", path(
        wp(7.734, 6.734, 90.0),
        et(0.450, "launcherLow"),
        tt(5.610, 7.416),
        rt(0.061, -178.7),
        rt(0.484, 180.0),
        et(0.584, "launcherPrep"),
        tt(2.881, 7.416),
        rt(0.183, -129.3),
        tt(0.913, 6.929),
        wp(0.728, 5.389, -119.4)
    ));

    // TL-CTR-QTL-BL-TL: big loop path through left trench; default handoff radius at (7.61,7.21)
    // Events: intakeIntake@seg0+19.4%, launcherPrep@seg5+42.3%, launcherLow@seg7+28.1%
    m.put("TL-CTR-QTL-BL-TL", path(
        wp(4.413, 7.444, 0.1),
        et(0.194, "intakeIntake"),
        rt(0.458, 0.0),
        tt(7.607, 7.212),
        rt(0.000, -90.0),
        tt(7.607, 4.795),
        rt(0.440, 178.8),
        tt(6.097, 4.561),
        rt(0.529, 90.0),
        tt(5.631, 5.529),
        rt(0.079, 90.0),
        et(0.423, "launcherPrep"),
        tt(3.529, 5.529),
        tt(1.364, 6.313),
        rt(0.876, 0.0),
        et(0.281, "launcherLow"),
        tt(3.018, 7.444),
        wp(4.413, 7.444, 0.1)
    ));

    // TL-CTR-HLF-BL-HP: (4.41,7.44)[0.1°] → (8.12,7.20) → (7.99,4.11) → …
    // Left trench: raised to Y=7.200 to stay above 7.146 m inner-wall limit.
    // Events: launcherPrep@seg5+25.1%; many rotations
    m.put("TL-CTR-HLF-BL-HP", path(
        wp(4.413, 7.444, 0.1),
        rt(0.447, 0.8),
        tt(8.117, 7.200),
        rt(0.000, -90.0),
        tt(7.987, 4.113),
        rt(0.588, -102.7),
        tt(6.097, 4.337),
        rt(0.440, 174.9),
        tt(5.827, 5.529),
        rt(0.090, 90.0),
        et(0.251, "launcherPrep"),
        tt(3.529, 5.529),
        rt(0.000, 90.0),
        wp(0.830, 7.095, 90.0)
    ));

    // ── Right / Orbit-right ─────────────────────────────────────────────────

    // TR-CTR-QTR: (4.41,0.49)[90°] → (7.00,0.83) → (7.82,3.60)[90°]
    // X=7.000 keeps corner 0.82m west of trench entrance; ~0.5m swerve overshoot peaks at X≈7.5 < 7.82 safe.
    // Events: intakeIntake@seg0+17.3%; rot 90.8°@seg0+50.0%, 89.1°@seg1+9.7%
    m.put("TR-CTR-QTR", path(
        wp(4.413, 0.494, 90.0),
        et(0.173, "intakeIntake"),
        rt(0.500, 90.8),
        tt(7.000, 0.829),
        rt(0.097, 89.1),
        wp(7.822, 3.596, 90.0)
    ));

    // QTR-TR: (7.82,3.60)[90°] → (7.12,0.86) → (3.53,0.62)[0°]
    // Events: launcherLow@seg0+63.7%; rot 0°@seg1+17.7%
    m.put("QTR-TR", path(
        wp(7.822, 3.596, 90.0),
        et(0.637, "launcherLow"),
        tt(7.123, 0.855),
        rt(0.177, 0.0),
        wp(3.534, 0.615, 0.0)
    ));

    // TR-CTR-HALF: (3.53,0.62)[0°] → (5.48,0.64) → (6.22,3.28)[90°]
    // Events: intakeIntake@seg0+39.9%; rot 0°@seg0+94.8%, 89.9°@seg1+46.2%
    m.put("TR-CTR-HALF", path(
        wp(3.534, 0.615, 0.0),
        et(0.399, "intakeIntake"),
        rt(0.948, 0.0),
        tt(5.483, 0.636),
        rt(0.462, 89.9),
        wp(6.216, 3.281, 90.0)
    ));

    // QTRH-HP: (6.22,3.28)[90°] → (4.66,0.61) → (0.48,0.80)[-90°]
    // Events: launcherPrep@seg0+104.6%; rot 0°@seg0+70.9%, 0°@seg1+29.6%
    m.put("QTRH-HP", path(
        wp(6.216, 3.281, 90.0),
        rt(0.709, 0.0),
        et(0.046, "launcherPrep"),
        tt(4.663, 0.611),
        rt(0.296, 0.0),
        wp(0.475, 0.800, -90.0)
    ));

    // TR-CTR-QTRLong: (3.53,0.62)[0°] → (7.00,1.01) → (7.59,3.74)[90°]
    // X=7.000 keeps corner 0.82m west of trench entrance; ~0.5m swerve overshoot peaks at X≈7.5 < 7.82 safe.
    // Events: intakeIntake@seg0+33.7%; rot 0°@seg0+50%, 89.1°@seg1+9.7%
    m.put("TR-CTR-QTRLong", path(
        wp(3.534, 0.615, 0.0),
        et(0.337, "intakeIntake"),
        rt(0.500, 0.0),
        tt(7.000, 1.008),
        rt(0.097, 89.1),
        wp(7.592, 3.740, 90.0)
    ));

    // QTRLong-HP: (7.59,3.74)[90°] → (4.66,0.61) → (0.48,0.80)[-90°]
    // Events: launcherPrep@seg1+6.6%; rot 0°@seg0+70.9%, 0°@seg1+29.6%
    m.put("QTRLong-HP", path(
        wp(7.592, 3.740, 90.0),
        rt(0.709, 0.0),
        tt(4.663, 0.611),
        rt(0.296, 0.0),
        et(0.066, "launcherPrep"),
        wp(0.475, 0.800, -90.0)
    ));

    // TR-CTR-QTRAngled: (4.41,0.49)[90°] → (7.00,0.90) → (7.99,3.17)[90°]
    // X=7.000 keeps corner 0.82m west of trench entrance; ~0.5m swerve overshoot peaks at X≈7.5 < 7.82 safe.
    // Events: intakeIntake@seg0+33.7%; rot 0°@seg0+50%, 90°@seg1+9.7%
    m.put("TR-CTR-QTRAngled", path(
        wp(4.413, 0.494, 90.0),
        et(0.337, "intakeIntake"),
        rt(0.500, 0.0),
        tt(7.000, 0.899),
        rt(0.097, 90.0),
        wp(7.986, 3.172, 90.0)
    ));

    // QTR-TRBack: (7.99,3.17)[90°] → (7.12,0.86) → (2.99,0.68)[0°]
    // Events: launcherLow@seg0+63.7%, launcherPrep@seg1+67.3%; rot 0°@seg1+17.7%
    m.put("QTR-TRBack", path(
        wp(7.986, 3.172, 90.0),
        et(0.637, "launcherLow"),
        tt(7.123, 0.855),
        rt(0.177, 0.0),
        et(0.673, "launcherPrep"),
        wp(2.992, 0.680, 0.0)
    ));

    // TRBack-CTR-HALF: (2.99,0.68)[0°] → (5.48,0.64) → (6.31,3.37)[90°]
    // Events: intakeIntake@seg0+39.9%; rot 0°@seg0+94.8%, 89.9°@seg1+46.2%
    m.put("TRBack-CTR-HALF", path(
        wp(2.992, 0.680, 0.0),
        et(0.399, "intakeIntake"),
        rt(0.948, 0.0),
        tt(5.483, 0.636),
        rt(0.462, 89.9),
        wp(6.314, 3.368, 90.0)
    ));

    // QTRH-TRBack: (6.31,3.37)[90°] → (4.66,0.61) → (2.99,0.68)[0°]
    // Events: launcherPrep@seg1+4.6%; rot 0°@seg0+70.9%, 0°@seg1+29.6%
    m.put("QTRH-TRBack", path(
        wp(6.314, 3.368, 90.0),
        rt(0.709, 0.0),
        tt(4.663, 0.611),
        rt(0.296, 0.0),
        et(0.046, "launcherPrep"),
        wp(2.992, 0.680, 0.0)
    ));

    // TR-CTR-QTRShort: (4.41,0.49)[90°] → (7.00,0.88) → (7.75,3.19)[90°]
    // X=7.000 keeps corner 0.82m west of trench entrance; ~0.5m swerve overshoot peaks at X≈7.5 < 7.82 safe.
    // Events: intakeIntake@seg0+23.2%; rot 89.1°@seg1+9.7%
    m.put("TR-CTR-QTRShort", path(
        wp(4.413, 0.494, 90.0),
        et(0.232, "intakeIntake"),
        tt(7.000, 0.878),
        rt(0.097, 89.1),
        wp(7.752, 3.188, 90.0)
    ));

    // QTRShort-TR: (7.75,3.19)[90°] → (7.12,0.86) → (3.53,0.62)[0°]
    // Events: launcherLow@seg0+63.7%; rot 0°@seg1+17.7%
    m.put("QTRShort-TR", path(
        wp(7.752, 3.188, 90.0),
        et(0.637, "launcherLow"),
        tt(7.123, 0.855),
        rt(0.177, 0.0),
        wp(3.534, 0.615, 0.0)
    ));

    // TRSide-CTR-QTR: (3.52,0.65)[-0.1°] → (7.00,0.85) → (7.82,3.60)[90°]
    // X=7.000 keeps corner 0.82m west of trench entrance; ~0.5m swerve overshoot peaks at X≈7.5 < 7.82 safe.
    // Events: intakeIntake@seg0+0.0%; rot 0°@seg0+69.9%, 89.1°@seg1+9.7%
    m.put("TRSide-CTR-QTR", path(
        wp(3.522, 0.652, -0.1),
        et(0.000, "intakeIntake"),
        rt(0.699, 0.0),
        tt(7.000, 0.849),
        rt(0.097, 89.1),
        wp(7.822, 3.596, 90.0)
    ));

    // TR-QTRCTRSHORT: (3.52,0.65)[-0.1°] → (6.98,0.74) → (7.82,2.22) → (7.75,3.19)[90°]
    // Events: intakeIntake@seg0+60.4%, launcherPrep@seg0+74.1%; rot 0°@seg0+84.9%, 90°@seg1+45.9%
    m.put("TR-QTRCTRSHORT", path(
        wp(3.522, 0.652, -0.1),
        et(0.604, "intakeIntake"),
        rt(0.849, 0.0),
        et(0.741, "launcherPrep"),
        tt(6.984, 0.741),
        rt(0.459, 90.0),
        tt(7.817, 2.221),
        wp(7.752, 3.188, 90.0)
    ));

    // CTR-NWALL (right): (7.75,3.19)[90°] → (6.64,3.19) → (7.17,1.30)[90°]
    // Events: launcherPrep@seg0+7.9%; rot 90.7°@seg0+70.5%
    m.put("CTR-NWALL", path(
        wp(7.752, 3.188, 90.0),
        et(0.079, "launcherPrep"),
        rt(0.705, 90.7),
        tt(6.643, 3.188),
        wp(7.169, 1.297, 90.0)
    ));

    // 3RD-SWIPE: (7.17,1.30)[90°] → (5.94,1.86) → (7.75,1.30)[-91.8°]
    // Events: launcherLow@seg0+0.0%; rot 89.2°@seg1+3.1%, -90°@seg1+82.1%
    m.put("3RD-SWIPE", path(
        wp(7.169, 1.297, 90.0),
        et(0.000, "launcherLow"),
        tt(5.941, 1.862),
        rt(0.031, 89.2),
        rt(0.821, -90.0),
        wp(7.754, 1.297, -91.8)
    ));

    // NWALL-CLIMB: (7.75,1.30)[-91.8°] → (5.59,0.55) → (2.82,0.55) → (0.87,0.85) → (0.61,2.38)[89°]
    // Events: launcherLow@seg0+45.0%, launcherPrep@seg1+58.4%; many rotations
    m.put("NWALL-CLIMB", path(
        wp(7.754, 1.297, -91.8),
        et(0.450, "launcherLow"),
        tt(5.588, 0.550),
        rt(0.484, -179.6),
        et(0.584, "launcherPrep"),
        tt(2.815, 0.550),
        rt(0.183, 89.8),
        tt(0.871, 0.848),
        wp(0.612, 2.376, 89.0)
    ));

    // TR-CTR-QTR-BR-TR: big loop path through right trench; corner at (7.00,0.86)
    // X=7.000 keeps east→north corner 0.82m west of trench entrance; overshoot peaks at X≈7.5 < 7.82 safe.
    // Events: intakeIntake@seg0+19.4%, launcherPrep@seg5+24.9%, launcherLow@seg7+28.1%
    m.put("TR-CTR-QTR-BR-TR", path(
        wp(3.522, 0.652, -0.1),
        et(0.194, "intakeIntake"),
        rt(0.445, 0.0),
        tt(7.000, 0.858),
        rt(0.000, 89.8),
        tt(7.607, 3.275),
        rt(0.662, 90.0),
        tt(6.097, 3.509),
        rt(0.440, -178.8),
        tt(5.755, 2.541),
        rt(0.000, -90.0),
        et(0.249, "launcherPrep"),
        tt(3.529, 2.541),
        rt(0.000, -90.0),
        tt(1.364, 1.757),
        rt(0.876, 0.0),
        et(0.281, "launcherLow"),
        tt(3.018, 0.626),
        wp(3.522, 0.652, -0.1)
    ));

    // TR-CTR-HLF-BR-HP: (3.52,0.65)[-0.1°] → (8.12,1.00) → (8.12,4.22) → …
    // Right trench: lowered to Y=1.000 to keep robot centre safely below 1.064 m inner-wall limit.
    // Events: launcherPrep@seg5+31.5%; many rotations
    m.put("TR-CTR-HLF-BR-HP", path(
        wp(3.522, 0.652, -0.1),
        rt(0.447, -0.8),
        tt(8.117, 1.000),
        rt(0.000, 90.0),
        tt(8.117, 4.221),
        rt(0.588, 102.7),
        tt(6.097, 3.733),
        rt(0.440, -174.9),
        tt(5.807, 2.541),
        rt(0.090, -90.0),
        et(0.315, "launcherPrep"),
        tt(3.529, 2.541),
        tt(0.825, 2.154),
        wp(0.470, 0.741, -90.1)
    ));

    // TR-CTR-QTR-BR-HP Longer: (3.53,0.62)[0°] → (7.00,0.92) → (7.62,4.29) → …
    // X=7.000 keeps east→north corner 0.82m west of trench entrance; overshoot peaks at X≈7.5 < 7.82 safe.
    // Events: intakeIntake@seg0+41.1%, launcherPrep@seg4+27.8%; many rotations
    m.put("TR-CTR-QTR-BR-HP Longer", path(
        wp(3.534, 0.615, 0.0),
        et(0.411, "intakeIntake"),
        rt(0.500, 0.0),
        tt(7.000, 0.923),
        rt(0.000, 77.6),
        tt(7.623, 4.287),
        rt(0.701, 124.5),
        tt(5.766, 2.690),
        rt(0.897, -89.7),
        tt(3.079, 2.461),
        rt(0.000, -89.6),
        et(0.278, "launcherPrep"),
        tt(1.364, 1.707),
        wp(0.475, 0.800, -90.0)
    ));

    // TR-CTR-QTR-BR-HP: (3.53,0.62)[0°] → (8.29,1.00) → (6.81,3.76) → …
    // Right trench: lowered to Y=1.000 to keep robot centre safely below 1.064 m inner-wall limit.
    // Events: launcherPrep@seg4+27.8%; many rotations
    m.put("TR-CTR-QTR-BR-HP", path(
        wp(3.534, 0.615, 0.0),
        rt(0.500, 0.0),
        tt(8.292, 1.000),
        rt(0.000, 111.6),
        tt(6.808, 3.757),
        rt(0.674, -176.9),
        tt(5.647, 2.614),
        rt(0.897, -89.7),
        tt(3.079, 2.461),
        rt(0.000, -73.0),
        et(0.278, "launcherPrep"),
        tt(1.364, 1.707),
        wp(0.475, 0.800, -90.0)
    ));

    // ── Delay / Follow paths ─────────────────────────────────────────────────

    // DelayTR-QTRL: (3.58,0.65)[-2.1°] → (7.10,0.92) → (7.91,3.95)[90°]
    // X=7.100 keeps corner 0.72m west of trench entrance; ~0.5m swerve overshoot peaks at X≈7.6 < 7.82 safe.
    // Events: intakeIntake@seg0+0.0%, launcherLow@seg1+60.1%; rot 0°@seg0+50%, 89.1°@seg1+24.6%
    m.put("DelayTR-QTRL", path(
        wp(3.583, 0.654, -2.1),
        et(0.000, "intakeIntake"),
        rt(0.500, 0.0),
        tt(7.100, 0.923),
        rt(0.097, 89.1),
        et(0.601, "launcherLow"),
        wp(7.909, 3.948, 90.0)
    ));

    // DelayTRS-CTR-QTR-BR-HP Longer: (3.58,0.65)[-2.1°] → (7.00,0.80) → (7.72,3.51) → …
    // X=7.000 keeps east→north corner 0.82m west of trench entrance; overshoot peaks at X≈7.5 < 7.82 safe.
    // Events: intakeIntake@seg0+52.7%, launcherPrep@seg4+6.9%; many rotations
    m.put("DelayTRS-CTR-QTR-BR-HP Longer", path(
        wp(3.583, 0.654, -2.1),
        et(0.527, "intakeIntake"),
        rt(0.500, 0.0),
        tt(7.000, 0.800),
        rt(0.000, 77.6),
        tt(7.724, 3.510),
        rt(0.701, 124.5),
        tt(5.735, 2.461),
        rt(0.897, -89.7),
        tt(3.079, 2.461),
        rt(0.000, -89.6),
        et(0.069, "launcherPrep"),
        tt(1.364, 1.707),
        wp(0.475, 0.800, -90.0)
    ));

    // QTRLeft-HP: (7.91,3.95)[90°] → (7.57,0.95) → (4.63,0.58) → (0.48,0.80)[-90°]
    // Events: launcherLow@seg1+27.5%, launcherPrep@seg2+32.9%; rot 0°@seg2+5.0%
    m.put("QTRLeft-HP", path(
        wp(7.909, 3.948, 90.0),
        tt(7.569, 0.953),
        rt(0.275, 0.0),
        et(0.275, "launcherLow"),
        tt(4.632, 0.582),
        et(0.329, "launcherPrep"),
        wp(0.475, 0.800, -90.0)
    ));

    // QBRight-HP: (7.91,3.95)[90°] → (7.52,2.46) → (4.54,2.46) → (2.25,2.15) → (0.48,0.80)[-90°]
    // Events: launcherLow@seg1+27.5%, launcherPrep@seg3+35.5%; rot 90°@seg2+70.0%
    m.put("QBRight-HP", path(
        wp(7.909, 3.948, 90.0),
        tt(7.516, 2.461),
        et(0.275, "launcherLow"),
        tt(4.544, 2.461),
        rt(0.700, 90.0),
        tt(2.249, 2.145),
        et(0.355, "launcherPrep"),
        wp(0.475, 0.800, -90.0)
    ));

    // ── Disrupt ─────────────────────────────────────────────────────────────

    // RightDisrupt1: (4.31,0.57)[1.5°] → (8.40,1.09) → (8.40,2.63) → (6.66,3.84)[-117.7°]
    // Events: rot 0.1°@seg0+50.0%, 60°@seg1+24.6%
    m.put("RightDisrupt1", path(
        wp(4.314, 0.571, 1.5),
        rt(0.500, 0.1),
        tt(8.401, 1.085),
        rt(0.246, 60.0),
        tt(8.401, 2.625),
        wp(6.664, 3.838, -117.7)
    ));

    // Disrupter2: (6.66,3.84)[-117.7°] → (6.45,1.61) → (3.05,0.65)[-179°]
    // Events: launcherLow@seg1+37.6%, launcherPrep@seg1+87.2%; rot -90°@seg0+50%, 180°@seg1+40.6%
    m.put("Disrupter2", path(
        wp(6.664, 3.838, -117.7),
        rt(0.500, -90.0),
        tt(6.445, 1.609),
        rt(0.406, 180.0),
        et(0.376, "launcherLow"),
        et(0.872, "launcherPrep"),
        wp(3.047, 0.647, -179.0)
    ));

    // ── Long multi-waypoint paths ────────────────────────────────────────────

    // StartTR-CTR-HLF-BR-HP: (3.53,0.53)[90°] → (8.27,1.00) → (8.27,4.17) → …
    // Right trench: lowered to Y=1.000 to keep robot centre safely below 1.064 m inner-wall limit.
    // Events: intakeIntake@seg0+43.5%, launcherPrep@seg5+4.0%; many rotations
    m.put("StartTR-CTR-HLF-BR-HP", path(
        wp(3.529, 0.527, 90.0),
        rt(0.447, 90.9),
        et(0.435, "intakeIntake"),
        tt(8.270, 1.000),
        rt(0.029, 90.0),
        tt(8.270, 4.169),
        rt(0.588, -162.6),
        tt(6.097, 3.733),
        rt(0.440, -90.0),
        tt(5.827, 2.541),
        rt(0.947, -90.0),
        et(0.040, "launcherPrep"),
        tt(3.529, 2.541),
        tt(0.825, 2.154),
        wp(0.475, 0.800, -90.0)
    ));

    return m;
  }

  private RebuiltPaths() {}
}
