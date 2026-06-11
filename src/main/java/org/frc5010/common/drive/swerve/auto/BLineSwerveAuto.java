package org.frc5010.common.drive.swerve.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.util.ArrayList;
import java.util.List;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;

/**
 * Glue between {@link AkitSwerveDrive} and the BLine path-following library.
 *
 * <p>BLine ({@code frc.robot.lib.BLine}) is a polyline path planner + tracker designed
 * for holonomic drives. This class provides two convenience entrypoints:
 *
 * <ul>
 *   <li>{@link #builder(AkitSwerveDrive)} — pre-wires a {@link FollowPath.Builder} with the
 *       drive's pose / chassis-speeds hooks, sensible default global constraints derived
 *       from the drive's {@code getMaxLinearSpeed} / {@code getMaxAngularSpeed}, and default
 *       PID gains. Subclasses or callers can chain additional {@code .with...()} configuration.
 *   <li>{@link #driveTo(AkitSwerveDrive, Pose2d)} — runtime drive-to-pose. Wraps a single-segment
 *       BLine path captured at command-init time, optionally with hand-authored via-points to
 *       skirt fixed field obstacles.
 * </ul>
 *
 * <p>BLine v0.9.1 does not expose a runtime Theta* pathfinder API (only {@code FollowPath},
 * {@code Path}, {@code BLineCommands}, {@code BLineField}, {@code FlippingUtil},
 * {@code ChassisRateLimiter}, {@code JsonUtils} are public). Obstacle avoidance therefore relies
 * on hand-authored via-points; if a future BLine release exposes a pathfinder, swap the
 * via-point list for its output without changing call sites.
 */
public final class BLineSwerveAuto {

  /**
   * Default global constraints applied by {@link #builder(AkitSwerveDrive)}.
   *
   * <p>Derived from drive limits:
   * <ul>
   *   <li>Max translational velocity = 75% of drive max linear speed (headroom for cross-track correction)
   *   <li>Max translational acceleration = 2× max linear speed (FRC swerves can typically pull ≥1 g of
   *       lateral traction; values much lower than this leave BLine unable to decelerate from cruise
   *       within the end-tolerance window, causing the robot to overshoot short paths)
   *   <li>Max rotational velocity = drive max angular speed (deg/s)
   *   <li>Max rotational acceleration = 2× max angular speed (deg/s²)
   *   <li>End translation tolerance = 5 cm
   *   <li>End rotation tolerance = 3°
   *   <li>Intermediate handoff radius = 30 cm
   * </ul>
   *
   * <p>Teams should tune these for their specific robot — see {@code docs/auto.md}.
   */
  public static Path.DefaultGlobalConstraints defaultGlobalConstraints(AkitSwerveDrive drive) {
    double maxV = drive.getMaxLinearSpeed().in(MetersPerSecond);
    double maxOmegaDeg = Math.toDegrees(drive.getMaxAngularSpeed().in(RadiansPerSecond));
    return new Path.DefaultGlobalConstraints(
        maxV * 0.75,
        maxV * 2.0,
        maxOmegaDeg,
        maxOmegaDeg * 2.0,
        0.05,
        3.0,
        0.30);
  }

  /**
   * Builds a {@link FollowPath.Builder} pre-wired to the given drive subsystem.
   *
   * <p>Also installs default global constraints via {@link Path#setDefaultGlobalConstraints}.
   * Call this once at robot init (e.g. inside {@code ExampleRobot}) and reuse the returned builder
   * across all paths.
   *
   * <p>Chains {@link FollowPath.Builder#withDefaultShouldFlip()} so paths are alliance-mirrored
   * automatically, and {@link FollowPath.Builder#withPoseReset(java.util.function.Consumer)}
   * pointing at {@link AkitSwerveDrive#setPose(Pose2d)} so each path's start pose re-anchors
   * the odometry estimator.
   */
  public static FollowPath.Builder builder(AkitSwerveDrive drive) {
    Path.setDefaultGlobalConstraints(defaultGlobalConstraints(drive));
    return new FollowPath.Builder(
            drive,
            drive::getPose,
            drive::getChassisSpeeds,
            drive::runVelocity,
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0))
        .withDefaultShouldFlip()
        .withPoseReset(drive::setPose);
  }

  /**
   * Returns a command that drives the robot to {@code goal} from its current pose, routing
   * straight (no via-points). For obstacle-aware variants see
   * {@link #driveToVia(AkitSwerveDrive, Pose2d, List)}.
   *
   * <p>The start pose is captured at command-init time (not at construction time) by deferring
   * path construction. This means binding the command to a button works correctly even if the
   * robot pose changes between binding setup and button press.
   */
  public static Command driveTo(AkitSwerveDrive drive, Pose2d goal) {
    return driveToVia(drive, goal, List.of());
  }

  /**
   * Returns a command that drives the robot to {@code goal} via the given intermediate
   * translation via-points. Use this for hand-authored obstacle avoidance until BLine exposes
   * a runtime pathfinder.
   *
   * <p>Example: route a Reef-skirting path on the 2026 field.
   * <pre>{@code
   * BLineSwerveAuto.driveToVia(drive,
   *     new Pose2d(4.5, 4.0, Rotation2d.fromDegrees(180)),
   *     List.of(new Translation2d(2.8, 5.5), new Translation2d(3.6, 5.0)));
   * }</pre>
   */
  public static Command driveToVia(AkitSwerveDrive drive, Pose2d goal, List<Translation2d> via) {
    // Local builder: no alliance flip (start was captured in the live field frame) and
    // no pose reset (don't clobber odometry with a stale snapshot at command init).
    Path.setDefaultGlobalConstraints(defaultGlobalConstraints(drive));
    FollowPath.Builder b =
        new FollowPath.Builder(
                drive,
                drive::getPose,
                drive::getChassisSpeeds,
                drive::runVelocity,
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(3.0, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0))
            .withPoseReset(pose -> {});
    return Commands.defer(
        () -> {
          Pose2d start = drive.getPose();
          List<Path.PathElement> elements = new ArrayList<>();
          elements.add(new Path.Waypoint(start.getTranslation(), start.getRotation()));
          for (Translation2d v : via) {
            elements.add(new Path.TranslationTarget(v));
          }
          elements.add(new Path.Waypoint(goal.getTranslation(), goal.getRotation()));
          return b.build(new Path(elements));
        },
        java.util.Set.of(drive));
  }

  private BLineSwerveAuto() {}
}
