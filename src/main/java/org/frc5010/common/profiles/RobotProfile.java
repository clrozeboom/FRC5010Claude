package org.frc5010.common.profiles;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.vision.Vision;

/**
 * Encapsulates everything a {@link SwerveRobotContainer} needs to know about a specific robot
 * configuration: how to build the drive subsystem and where the robot starts on the field.
 *
 * <p>Three profiles cover all deployment scenarios:
 * <ol>
 *   <li><strong>Test/sim profile</strong> (e.g. {@link SimRobotProfile}) — lightweight SIM
 *       constants with no real CAN IDs, used for library development and CI.</li>
 *   <li><strong>Real-robot-in-sim profile</strong> — the team's real robot constants but
 *       running in simulation. {@link #createDrive()} delegates to
 *       {@code SwerveFactory.build()} so IronMaple uses the actual robot's mass, bumper
 *       size, and wheel geometry for physically accurate simulation.</li>
 *   <li><strong>Real hardware profile</strong> — same constants as above, but
 *       {@link #createDrive()} branches on {@code RobotBase.isReal()} to wire up
 *       the actual hardware IO implementations ({@code ModuleIOTalonFXReal}, etc.).</li>
 * </ol>
 *
 * <p>Scenarios 2 and 3 are handled by a single profile class — see the template below.
 *
 * <h3>Template for a real robot project</h3>
 * <pre>{@code
 * public class ExampleRobotProfile extends RobotProfile {
 *
 *   // Physical constants — used by both hardware and simulation.
 *   private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
 *       .moduleType(ModuleType.TALON_FX)
 *       .gyroType(GyroType.PIGEON2)
 *       .gyroCanId(1)
 *       .trackWidth(Inches.of(22.75))
 *       .wheelBase(Inches.of(22.75))
 *       .wheelRadius(Inches.of(2.0))
 *       .robotMass(Pounds.of(125))
 *       .bumperLength(Inches.of(30))
 *       .bumperWidth(Inches.of(30))
 *       .frontLeftIds(1, 2, 3)
 *       // ... other IDs
 *       .build();
 *
 *   private static final Pose2d BLUE_START = new Pose2d(1.5, 5.5, new Rotation2d());
 *
 *   {@literal @}Override
 *   public AkitSwerveDrive createDrive() {
 *     if (RobotBase.isReal()) {
 *       // Wire real hardware IO — SwerveFactory.build() throws for TALON_FX in REAL mode.
 *       GyroIO gyro = new GyroIOPigeon2(CONSTANTS);
 *       ModuleIO[] modules = {
 *         new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.FrontLeft),
 *         new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.FrontRight),
 *         new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.BackLeft),
 *         new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.BackRight),
 *       };
 *       return new AkitSwerveDrive(CONSTANTS, gyro, modules);
 *     }
 *     // In simulation: IronMaple uses the real robot's mass/geometry for accurate physics.
 *     return SwerveFactory.build(CONSTANTS, BLUE_START);
 *   }
 *
 *   {@literal @}Override
 *   public Pose2d getBlueAllianceStartPose() { return BLUE_START; }
 * }
 * }</pre>
 */
public abstract class RobotProfile {

  /**
   * Creates and returns the fully wired swerve drive subsystem for this robot.
   *
   * <p>Implementations must handle both hardware and simulation branches where needed.
   * See the class-level Javadoc template for the recommended real-robot pattern.
   */
  public abstract AkitSwerveDrive createDrive();

  /**
   * Returns the robot's starting pose on the Blue alliance side.
   *
   * <p>Used by {@link SwerveRobotContainer#resetToAllianceStart()} to compute the
   * correct pose for either alliance. The Red-alliance pose is derived automatically.
   */
  public abstract Pose2d getBlueAllianceStartPose();

  /**
   * Creates the {@link Vision} subsystem for this robot profile.
   *
   * <p>Returns {@code null} by default (no cameras configured). Override in real-robot profiles
   * to wire cameras via {@link org.frc5010.common.vision.VisionFactory}. Called by
   * {@link SwerveRobotContainer} immediately after {@link #createDrive()}, so the drive is
   * available for the pose supplier and {@code addVisionMeasurement} consumer.
   *
   * @param drive the fully-wired drive subsystem returned by {@link #createDrive()}
   */
  public Vision createVision(AkitSwerveDrive drive) {
    return null;
  }

  /**
   * Returns the field length for this year's game.
   * Default: Arena2026Rebuilt (16.540988 m). Override for other game years.
   */
  public Distance getFieldLength() {
    return Meters.of(16.540988);
  }

  /**
   * The AprilTag field layout this robot plays on.
   *
   * <p>This is the authority for the field: {@link SwerveRobotContainer} pushes it into the
   * shared {@link org.frc5010.common.vision.AprilTags} holder before building the drive and
   * vision, so the rest of the vision and field-geometry code follows the profile's choice.
   * Default: WPILib's current-season default ({@code kDefaultField}, the welded variant).
   * Override to compete on a different variant (e.g. {@code k2026RebuiltAndymark}).
   */
  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }
}
