package frc.robot.tigershark;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.akit.GyroIO;
import org.frc5010.common.drive.swerve.akit.GyroIOPigeon2;
import org.frc5010.common.drive.swerve.akit.ModuleIO;
import org.frc5010.common.drive.swerve.akit.ModuleIOTalonFXReal;
import org.frc5010.common.profiles.RobotProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

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
public class TigerSharkRobotProfile extends RobotProfile {

    private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
      .moduleType(ModuleType.TALON_FX)   // or SPARK_TALON
      .gyroType(GyroType.PIGEON2)        // or NAVX
      .gyroCanId(50)
      .trackWidth(Inches.of(22)) // measured
      .wheelBase(Inches.of(22)) // measured
      .wheelRadius(Inches.of(2.0)) // measured
      .maxLinearSpeed(MetersPerSecond.of(2.5)) // made a safe value
      .maxAngularSpeed(RadiansPerSecond.of(2 * Math.PI)) // OK for now
      .robotMass(Pounds.of(125)) // OK for now
      .bumperLength(Inches.of(30)) // ok for now
      .bumperWidth(Inches.of(30)) // OK for now
      .frontLeftIds(4, 3, 13)
      .frontRightIds(2, 1, 16)
      .backLeftIds(6, 5, 14)
      .backRightIds(8, 7, 0)
      .canBusName("")                                  // match TunerConstants.kCANBus
      .odometryFrequency(Hertz.of(100))                // 100 Hz RIO bus, 250 Hz CANivore
      .build();

  private static final Pose2d BLUE_START = new Pose2d(1.5, 2.0, new Rotation2d());

  @Override
  public AkitSwerveDrive createDrive() {
    if (RobotBase.isReal()) {
        GyroIO gyro = new GyroIOPigeon2(CONSTANTS);
        ModuleIO[] modules = {
          new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.FrontLeft),
          new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.FrontRight),
          new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.BackLeft),
          new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.BackRight),
        };
        return new AkitSwerveDrive(CONSTANTS, gyro, modules);
    }
    // In simulation: IronMaple uses the real robot's mass/geometry for accurate physics.
    return SwerveFactory.build(CONSTANTS, BLUE_START);
  }

  @Override
  public Pose2d getBlueAllianceStartPose() {
    return BLUE_START;
  }

  // Vision is disabled until the PhotonVision coprocessor is available.
  // To re-enable, uncomment createVision() and restore the camera transform + VisionFactory wiring.
  // Front-facing camera: 30 cm forward, 50 cm up, aligned with robot heading.
  // private static final Transform3d FRONT_CAM_TRANSFORM = new Transform3d(
  //     new Translation3d(0.30, 0.0, 0.50), new Rotation3d());
}
