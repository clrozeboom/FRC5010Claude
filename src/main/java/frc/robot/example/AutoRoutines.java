package frc.robot.example;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.auto.BLineSwerveAuto;

/**
 * BLine autonomous routines for the 2026 demo. All poses are Blue-side; the BLine builder
 * is configured with {@code .withDefaultShouldFlip()} so the same routine drives the
 * alliance-mirrored path on Red.
 *
 * <p>{@link #exampleScore(AkitSwerveDrive)} and {@link #exampleScoreInCode(AkitSwerveDrive)}
 * illustrate the two BLine path-definition workflows. {@link #pickupAndScore(AkitSwerveDrive,
 * DemoIntake)} composes BLine path-following with {@link DemoIntake} mechanism commands —
 * drive out, intake Fuel, drive back inside the alliance scoring zone, fire at the Hub.
 */
public final class AutoRoutines {

  private static final Pose2d EXAMPLE_START =
      new Pose2d(1.5, 2.0, Rotation2d.fromDegrees(0));

  /** Loads {@code src/main/deploy/autos/paths/ExampleScore.json} and follows it. */
  public static Command exampleScore(AkitSwerveDrive drive) {
    FollowPath.Builder b = BLineSwerveAuto.builder(drive);
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(EXAMPLE_START), drive),
        Commands.waitSeconds(0.05),
        b.build(new Path("ExampleScore")),
        Commands.runOnce(drive::stop, drive));
  }

  /** Same trajectory as {@link #exampleScore} but defined inline in Java. */
  public static Command exampleScoreInCode(AkitSwerveDrive drive) {
    FollowPath.Builder b = BLineSwerveAuto.builder(drive);
    Path p = new Path(
        new Path.Waypoint(new Translation2d(1.5, 2.0), Rotation2d.fromDegrees(0)),
        new Path.TranslationTarget(new Translation2d(2.25, 2.0)),
        new Path.Waypoint(new Translation2d(3.0, 2.0), Rotation2d.fromDegrees(0)));
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(EXAMPLE_START), drive),
        Commands.waitSeconds(0.05),
        b.build(p),
        Commands.runOnce(drive::stop, drive));
  }

  /**
   * Composed BLine + DemoIntake auto: start at the alliance wall, drive into the field with
   * the intake extended, collect one Fuel piece, drive back into the alliance scoring zone
   * ({@code X < 3.952 m} on Blue), and fire at the Hub.
   *
   * <p>Geometry references {@link DemoIntake}: the Blue Hub is at {@code (4.5974, 4.0345)} and
   * scoring requires the robot to be inside the alliance zone so {@code fireCommand()} selects
   * the ballistic Hub shot rather than the lob-to-zone fallback.
   */
  public static Command pickupAndScore(AkitSwerveDrive drive, DemoIntake intake) {
    FollowPath.Builder b = BLineSwerveAuto.builder(drive);

    Pose2d start = new Pose2d(1.50, 4.03, Rotation2d.fromDegrees(0));
    Pose2d pickup = new Pose2d(9.30, 2.76, Rotation2d.fromDegrees(0));
    Pose2d shotSpot = new Pose2d(2.80, 4.03, Rotation2d.fromDegrees(0));

    // The center-field Fuel sits in a 5-col × 6-row grid (x 7.43–9.11, rows at y = 1.91, 2.76,
    // 3.61, …; see GamePieceSpawner) — past the Blue Hub, which is a physical obstacle on the
    // field centerline at (4.5974, 4.0345). Skirt the Hub through the clear lane below it
    // (y≈2.5), then drive +X straight through the y=2.76 Fuel row with the front intake extended
    // so it collects the row. Return the same lane to the alliance-zone shot spot and fire.
    Path outbound = new Path(
        new Path.Waypoint(start.getTranslation(), start.getRotation()),
        new Path.TranslationTarget(new Translation2d(3.0, 2.5)),
        new Path.TranslationTarget(new Translation2d(6.5, 2.76)),
        new Path.Waypoint(pickup.getTranslation(), pickup.getRotation()));

    Path returnPath = new Path(
        new Path.Waypoint(pickup.getTranslation(), pickup.getRotation()),
        new Path.TranslationTarget(new Translation2d(5.5, 2.5)),
        new Path.TranslationTarget(new Translation2d(3.5, 2.5)),
        new Path.Waypoint(shotSpot.getTranslation(), shotSpot.getRotation()));

    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(start), drive),
        Commands.waitSeconds(0.05),
        Commands.parallel(b.build(outbound), intake.extendCommand()),
        Commands.waitUntil(() -> intake.getHeldFuel() > 0).withTimeout(2.0),
        intake.retractCommand(),
        b.build(returnPath),
        fireAllFuel(intake),
        Commands.runOnce(drive::stop, drive));
  }

  /**
   * Fires every Fuel piece the intake currently holds at the Hub — one ballistic shot per
   * piece — from wherever the robot is. Append this to an auto so the robot scores its preload
   * (and anything it collected). Score from inside the alliance zone ({@code X < 3.952 m} on
   * Blue) so {@link DemoIntake} targets the Hub rather than lobbing toward the zone.
   */
  public static Command fireAllFuel(DemoIntake intake) {
    return Commands.repeatingSequence(intake.fireCommand(), Commands.waitSeconds(0.25))
        .until(() -> intake.getHeldFuel() == 0)
        .withTimeout(6.0);
  }

  private AutoRoutines() {}
}
