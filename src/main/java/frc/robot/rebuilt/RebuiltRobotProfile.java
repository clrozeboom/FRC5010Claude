package frc.robot.rebuilt;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.profiles.RobotProfile;
import org.frc5010.common.vision.AprilTags;
import org.frc5010.common.vision.CameraConfig;
import org.frc5010.common.vision.Vision;
import org.frc5010.common.vision.VisionFactory;

/**
 * Robot profile for the ported 2026 "Rebuilt" competition robot — the drivetrain geometry
 * (from the source {@code robot.json}) plus the four vision sources from the source
 * {@code rebuilt_robot/cameras/}.
 *
 * <p>Drivetrain: swerve, 22 in track / 22 in wheelbase, 0.1002156588 m wheel diameter,
 * 5.93 m/s physical max speed. {@code createDrive()} builds IronMaple physics in simulation
 * (REAL mode wiring is left as a hardware TODO, like {@code ExampleRobotProfile}).
 *
 * <p>Cameras: three PhotonVision AprilTag cameras (rear / right / left) plus the QuestNav
 * headset as a field-relative pose source. The source robot mounts the cameras pitched
 * 30° <em>down</em>; this robot mounts them pitched 30° <em>up</em>.
 */
public class RebuiltRobotProfile extends RobotProfile {

  // ── drivetrain (from robot.json) ─────────────────────────────────────────────
  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.TALON_FX)
          .gyroType(GyroType.PIGEON2)
          .gyroCanId(0)
          .trackWidth(Inches.of(22))
          .wheelBase(Inches.of(22))
          .wheelRadius(Meters.of(0.1002156588 / 2.0)) // 0.1002156588 m diameter
          .maxLinearSpeed(MetersPerSecond.of(5.93))
          .robotMass(Pounds.of(125))
          .bumperLength(Inches.of(30))
          .bumperWidth(Inches.of(30))
          .frontLeftIds(1, 2, 3)
          .frontRightIds(4, 5, 6)
          .backLeftIds(7, 8, 9)
          .backRightIds(10, 11, 12)
          .build();

  @Override
  public AkitSwerveDrive createDrive() {
    if (RobotBase.isReal()) {
      // TODO: wire ModuleIOTalonFXReal + GyroIOPigeon2 with the team's CTRE TunerX constants
      // (SwerveFactory.build() throws for TALON_FX in REAL mode by design — see ExampleRobotProfile).
      throw new UnsupportedOperationException(
          "RebuiltRobotProfile.createDrive() not yet wired for REAL mode — see the TODO.");
    }
    return SwerveFactory.build(CONSTANTS, blueStartPose());
  }

  @Override
  public Pose2d getBlueAllianceStartPose() {
    return blueStartPose();
  }

  /**
   * Robot starting pose for the blue alliance: bumpers on the starting line (the hub's near face),
   * centred on the field width, facing the hub.
   *
   * <p>Computed at call time (not a static field) so that {@link FieldConstants} is loaded
   * AFTER {@code AprilTags.setAprilTagFieldLayout()} has been called by the container — that
   * call happens before {@code createDrive()}, which is the first caller of this method.
   */
  private Pose2d blueStartPose() {
    // Bumpers on the starting line: robot centre = starting-line X − half bumper length (15 in).
    double x = FieldConstants.LinesVertical.starting - Units.inchesToMeters(15.0);
    double y = FieldConstants.fieldWidth / 2.0;
    return new Pose2d(x, y, new Rotation2d()); // 0° = facing the hub (positive-X direction)
  }

  /**
   * We compete on the AndyMark variant of the 2026 Rebuilt field. The container publishes this
   * to the shared {@link AprilTags} holder, so vision pose estimation ({@code VisionFactory}) and
   * the field geometry ({@link FieldConstants}) all use it.
   */
  @Override
  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return AprilTagFieldLayout.loadField(
        AprilTagFields.k2026RebuiltAndymark);
  }

  // ── cameras (from rebuilt_robot/cameras/) — pitched 30° UP ────────────────────

  /** 30° up-tilt for the AprilTag cameras (source mounts them 30° down). */
  private static final double CAM_PITCH_UP = Math.toRadians(30.0);

  // PhotonVision AprilTag cameras: robot-frame translation (m) + 30°-up pitch + per-camera yaw.
  private static final Transform3d REAR_PROMETHEUS =
      new Transform3d(
          new Translation3d(-0.2961, -0.2212, 0.2368),
          new Rotation3d(0.0, CAM_PITCH_UP, Math.toRadians(175.0)));
  private static final Transform3d RIGHT_RAIKOU =
      new Transform3d(
          new Translation3d(-0.2775, -0.3025, 0.2432),
          new Rotation3d(0.0, CAM_PITCH_UP, Math.toRadians(-81.0)));
  private static final Transform3d LEFT_BAGEL =
      new Transform3d(
          new Translation3d(-0.2778, 0.2975, 0.2368),
          new Rotation3d(0.0, CAM_PITCH_UP, Math.toRadians(70.0)));

  // QuestNav headset mount (pose source, not a PhotonVision camera).
  private static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          new Translation3d(-0.171968, -0.252501, 0.225336),
          new Rotation3d(0.0, 0.0, Math.toRadians(180.0)));

  @Override
  public Vision createVision(AkitSwerveDrive drive) {
    Vision vision =
        VisionFactory.build(
            drive::addVisionMeasurement,
            () -> drive.getSimulatedPose().orElse(drive.getPose()),
            drive::getRotation,
            new CameraConfig[] {
              new CameraConfig.Builder("rear-prometheus")
                  .robotToCamera(REAR_PROMETHEUS)
                  .backend(CameraConfig.Backend.PHOTON)
                  .build(),
              new CameraConfig.Builder("right-raikou")
                  .robotToCamera(RIGHT_RAIKOU)
                  .backend(CameraConfig.Backend.PHOTON)
                  .build(),
              new CameraConfig.Builder("left-bagel")
                  .robotToCamera(LEFT_BAGEL)
                  .backend(CameraConfig.Backend.PHOTON)
                  .build(),
              // QuestNav headset as a field-relative pose source (no-op in SIM).
              new CameraConfig.Builder("QuestNav")
                  .robotToCamera(ROBOT_TO_QUEST)
                  .backend(CameraConfig.Backend.QUESTNAV)
                  .build()
            });

    // Publish each tag's 2D pose so Glass draws the AprilTag overlays.
    AprilTags.aprilTagFieldLayout
        .getTags()
        .forEach(
            tag ->
                drive
                    .getField2d()
                    .getObject("Field Tag " + tag.ID)
                    .setPose(tag.pose.toPose2d()));

    return vision;
  }
}
