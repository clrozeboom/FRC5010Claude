package frc.robot.rebuilt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.auto.BLineSwerveAuto;

/**
 * Game-specific driver-triggered commands for the 2026 Rebuilt field.
 *
 * <p>Counterpart to {@link AutoRoutines}: {@code AutoRoutines} hosts autonomous-mode
 * sequences; this file hosts teleop-mode commands that are bound to driver buttons in
 * {@link RealRobot#configureBindings()}.
 *
 * <p>Coordinates here reference the 2026 Blue-side Reef / Hub. BLine's
 * {@code .withDefaultShouldFlip()} mirrors the path for the Red alliance, so the same
 * Blue-side definition works for both alliances.
 */
public final class TeleopRoutines {

  /**
   * Drives the robot to a fixed pose in front of the Blue Reef, routing around the
   * Reef centre with two hand-authored via-points.
   *
   * <p>Hub geometry (from {@link DemoIntake}): Hub centre at {@code (4.5974, 4.0345)}.
   * The approach pose {@code (3.6, 4.03, 0°)} is inside the Blue alliance scoring zone
   * ({@code X < 3.952 m}) so a follow-up {@link DemoIntake#fireCommand()} selects the
   * ballistic Hub shot.
   *
   * <p>BLine v0.9.1 has no runtime pathfinder, so obstacle avoidance is hand-authored.
   * If a future BLine release exposes a Theta* / pathfinder API, the via-point list
   * can be replaced with its output without changing the call site.
   */
  public static Command driveToHub(AkitSwerveDrive drive) {
    Pose2d hubApproach = new Pose2d(3.6, 4.03, Rotation2d.fromDegrees(0));
    return BLineSwerveAuto.driveToVia(
        drive,
        hubApproach,
        List.of(new Translation2d(2.5, 5.5), new Translation2d(3.1, 4.8)));
  }

  private TeleopRoutines() {}
}
