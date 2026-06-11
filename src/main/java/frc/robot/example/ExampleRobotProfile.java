package frc.robot.example;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.profiles.RobotProfile;
import org.frc5010.common.vision.CameraConfig;
import org.frc5010.common.vision.Vision;
import org.frc5010.common.vision.VisionFactory;

/**
 * Robot profile for the real competition robot.
 *
 * <p>Fill in the actual CAN IDs, hardware types, and physical measurements below.
 * This profile is used in three contexts:
 * <ol>
 *   <li><strong>Real hardware</strong> — {@code createDrive()} wires the real IO implementations.</li>
 *   <li><strong>Real-robot simulation</strong> — {@code createDrive()} falls through to
 *       {@code SwerveFactory.build()}, which creates IronMaple physics using this profile's
 *       physical constants (mass, bumper size, wheel geometry) for accurate simulation.</li>
 *   <li><strong>Replay</strong> — handled automatically by {@code SwerveFactory.build()}.</li>
 * </ol>
 *
 * <p>This profile is selected by default when running from VSCode's "Simulate Robot Code" menu
 * ({@code ./gradlew simulateJava}). Automated testing agents use {@code -PtestSim} instead to
 * get the lightweight {@link org.frc5010.common.profiles.SimRobotProfile}.
 */
public class ExampleRobotProfile extends RobotProfile {

  // TODO: Replace with actual robot measurements and CAN IDs.
  private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
      .moduleType(ModuleType.TALON_FX)   // or SPARK_TALON
      .gyroType(GyroType.PIGEON2)        // or NAVX
      .gyroCanId(0)
      .trackWidth(Inches.of(22.75))
      .wheelBase(Inches.of(22.75))
      .wheelRadius(Inches.of(2.0))
      .robotMass(Pounds.of(125))
      .bumperLength(Inches.of(30))
      .bumperWidth(Inches.of(30))
      .frontLeftIds(1, 2, 3)
      .frontRightIds(4, 5, 6)
      .backLeftIds(7, 8, 9)
      .backRightIds(10, 11, 12)
      .build();

  // TODO: Set the actual Blue-alliance starting pose for this year's game.
  private static final Pose2d BLUE_START = new Pose2d(1.5, 2.0, new Rotation2d());

  @Override
  public AkitSwerveDrive createDrive() {
    if (RobotBase.isReal()) {
      // TODO: Wire hardware IO here.
      // SwerveFactory.build() throws UnsupportedOperationException for TALON_FX in REAL mode
      // because it cannot construct ModuleIOTalonFXReal without CTRE TunerX SwerveModuleConstants.
      // Replace the placeholder below with the real wiring:
      //
      //   GyroIO gyro = new GyroIOPigeon2(CONSTANTS);
      //   ModuleIO[] modules = {
      //     new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.FrontLeft),
      //     new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.FrontRight),
      //     new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.BackLeft),
      //     new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.BackRight),
      //   };
      //   return new AkitSwerveDrive(CONSTANTS, gyro, modules);
      throw new UnsupportedOperationException(
          "ExampleRobotProfile.createDrive() not yet implemented for REAL mode. "
          + "See the TODO comments in this file.");
    }
    // In simulation: IronMaple uses the real robot's mass/geometry for accurate physics.
    return SwerveFactory.build(CONSTANTS, BLUE_START);
  }

  @Override
  public Pose2d getBlueAllianceStartPose() {
    return BLUE_START;
  }

  // Front-facing camera: 30 cm forward, 50 cm up, aligned with robot heading.
  private static final Transform3d FRONT_CAM_TRANSFORM = new Transform3d(
      new Translation3d(0.30, 0.0, 0.50), new Rotation3d());

  /**
   * Wires the {@code photon_front} PhotonVision camera and publishes static AprilTag poses to
   * the drive's Field2d so Glass renders AT*.png overlays for each tag.
   *
   * <p>The pose supplier uses the TRUE physics position ({@code getSimulatedPose}) rather than
   * the estimator so that injected estimator errors (e.g. push-correction test) do not prevent
   * the camera sim from detecting tags. See CLAUDE.md Vision architecture section.
   */
  @Override
  public Vision createVision(AkitSwerveDrive drive) {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    Vision vision = VisionFactory.build(
        drive::addVisionMeasurement,
        () -> drive.getSimulatedPose().orElse(drive.getPose()),
        drive::getRotation,
        new CameraConfig[] {
            new CameraConfig.Builder("photon_front")
                .robotToCamera(FRONT_CAM_TRANSFORM)
                .backend(CameraConfig.Backend.PHOTON)
                .build()
        });

    // Publish each tag's 2D pose as a named Field2d object so Glass draws the
    // AT*.png overlays configured in simgui.json.
    layout.getTags().forEach(tag ->
        drive.getField2d()
             .getObject("Field Tag " + tag.ID)
             .setPose(tag.pose.toPose2d()));

    return vision;
  }
}
