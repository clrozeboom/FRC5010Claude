// FRC5010 Framework — clean AkitSwerveDrive with no frc.robot dependencies

package org.frc5010.common.drive.swerve.akit;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.mechanisms.MechanismVisuals3d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;

/**
 * Swerve drivetrain subsystem built on the AdvantageKit IO abstraction layer.
 *
 * <p>This is a clean rewrite of the original {@code AkitSwerveDrive} that removes
 * all dependencies on {@code frc.robot.Constants}, {@code AkitSwerveConfig},
 * {@code GenericSwerveDrivetrain}, and third-party simulation frameworks.
 *
 * <p>All robot-specific configuration comes from {@link SwerveConstants}, making
 * this class usable in any robot project without modification.
 *
 * <p>The correct IO implementations (real hardware vs simulation vs replay) are
 * selected by {@link org.frc5010.common.drive.swerve.SwerveFactory} before this
 * class is constructed — this class never checks the operating mode itself.
 */
public class AkitSwerveDrive extends SubsystemBase {

  private final SwerveConstants constants;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SwerveDriveSimulation swerveDriveSimulation; // null when not using physics sim

  private final SwerveDriveKinematics kinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  private SysIdRoutine sysId;

  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // Pre-allocated odometry buffers — sized for max samples per 20ms loop.
  // At 250 Hz odometry the theoretical max is ~5 samples; 16 provides headroom.
  private static final int MAX_ODOMETRY_SAMPLES = 16;
  private static final SwerveModuleState[] EMPTY_MODULE_STATES = new SwerveModuleState[]{};

  private final SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[]{
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };

  private final SwerveModulePosition[][] odometryModulePositions =
      new SwerveModulePosition[MAX_ODOMETRY_SAMPLES][4];
  private final SwerveModulePosition[][] odometryModuleDeltas =
      new SwerveModulePosition[MAX_ODOMETRY_SAMPLES][4];

  /**
   * Constructs the swerve drivetrain without physics simulation.
   *
   * @param constants  robot-specific swerve configuration
   * @param gyroIO     gyro IO implementation (real, sim, or replay no-op)
   * @param moduleIOs  four module IO implementations in FL/FR/BL/BR order
   */
  public AkitSwerveDrive(SwerveConstants constants, GyroIO gyroIO, ModuleIO[] moduleIOs) {
    this(constants, gyroIO, moduleIOs, null);
  }

  /**
   * Constructs the swerve drivetrain with optional IronMaple physics simulation.
   *
   * <p>When {@code swerveDriveSimulation} is non-null, {@link #simulationPeriodic()} will
   * automatically advance the physics engine each loop. Use
   * {@link org.frc5010.common.drive.swerve.SwerveFactory#build(SwerveConstants)} rather than
   * calling this constructor directly.
   *
   * @param constants             robot-specific swerve configuration
   * @param gyroIO                gyro IO implementation
   * @param moduleIOs             four module IO implementations in FL/FR/BL/BR order
   * @param swerveDriveSimulation IronMaple physics simulation, or {@code null} for no-physics
   */
  public AkitSwerveDrive(
      SwerveConstants constants,
      GyroIO gyroIO,
      ModuleIO[] moduleIOs,
      SwerveDriveSimulation swerveDriveSimulation) {
    if (moduleIOs.length != 4) {
      throw new IllegalArgumentException(
          "AkitSwerveDrive requires exactly 4 ModuleIO instances, got: " + moduleIOs.length);
    }

    this.constants = constants;
    this.gyroIO = gyroIO;
    this.swerveDriveSimulation = swerveDriveSimulation;

    kinematics = new SwerveDriveKinematics(constants.moduleTranslations);
    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    Translation2d[] translations = constants.moduleTranslations;
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(
          moduleIOs[i], i,
          constants.wheelRadius.in(Units.Meters),
          translations[i].getX(),
          translations[i].getY());
    }

    // Pre-populate odometry buffers so no null checks needed in the hot loop
    for (int s = 0; s < MAX_ODOMETRY_SAMPLES; s++) {
      for (int m = 0; m < 4; m++) {
        odometryModulePositions[s][m] = new SwerveModulePosition();
        odometryModuleDeltas[s][m] = new SwerveModulePosition();
      }
    }

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start the odometry thread. The instance is null in SIM/REPLAY modes since
    // no hardware-backed thread is created; skip in that case.
    OdometryThread odometryThread = OdometryThread.getInstance();
    if (odometryThread != null) odometryThread.start();

    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void periodic() {
    OdometryThread.odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (Module module : modules) {
      module.periodic();
    }
    OdometryThread.odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", EMPTY_MODULE_STATES);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", EMPTY_MODULE_STATES);
    }

    // Feed sim gyro from kinematics when not using a real gyro
    if (RobotBase.isSimulation() && gyroIO instanceof GyroIOSim simGyro) {
      ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
      simGyro.updateAngularVelocity(speeds.omegaRadiansPerSecond);
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = Math.min(sampleTimestamps.length, MAX_ODOMETRY_SAMPLES);
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = odometryModulePositions[i];
      SwerveModulePosition[] moduleDeltas = odometryModuleDeltas[i];

      for (int m = 0; m < 4; m++) {
        SwerveModulePosition fresh = modules[m].getOdometryPositions()[i];
        modulePositions[m].distanceMeters = fresh.distanceMeters;
        modulePositions[m].angle = fresh.angle;
        moduleDeltas[m].distanceMeters =
            fresh.distanceMeters - lastModulePositions[m].distanceMeters;
        moduleDeltas[m].angle = fresh.angle;
        lastModulePositions[m].distanceMeters = fresh.distanceMeters;
        lastModulePositions[m].angle = fresh.angle;
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro disconnect alert — only warn on real hardware
    gyroDisconnectedAlert.set(!gyroInputs.connected && !RobotBase.isSimulation());

    // Update Field2d for SmartDashboard / simulation visualization.
    // In IronMaple physics mode, use the ground-truth physics body pose so the widget
    // shows the physics-constrained position rather than odometry that may have drifted
    // from wheel slip (e.g. when the robot is pressed against a wall).
    field2d.setRobotPose(
        swerveDriveSimulation != null
            ? swerveDriveSimulation.getSimulatedDriveTrainPose()
            : getPose());

    // Unified isometric robot view (chassis + wheels + gyro + mechanisms) on the same
    // RobotMechanisms3D canvas the mechanisms publish to — equivalent to the web panel.
    publishIsoScene();

    // Feed the IronMaple physics body's ground-truth pose into the pose estimator as a
    // near-perfect "vision" measurement.  This must happen AFTER updateWithTime() so the
    // pose buffer's last key is current — addVisionMeasurement() silently discards
    // measurements with timestamps newer than the buffer's latest key.
    //
    // Standard deviations of 1e-4 m/rad give the physics pose essentially full weight
    // over wheel odometry, so getPose() always reflects the physics-constrained position
    // (correct even when wheel slip drifts the encoders while the robot is against a wall).
    if (swerveDriveSimulation != null) {
      poseEstimator.addVisionMeasurement(
          swerveDriveSimulation.getSimulatedDriveTrainPose(),
          Timer.getTimestamp(),
          VecBuilder.fill(1e-4, 1e-4, 1e-4));
    }
  }

  @Override
  public void simulationPeriodic() {
    if (swerveDriveSimulation != null) {
      SimulatedArena.getInstance().simulationPeriodic();
    }
  }

  // ---------------------------------------------------------------------------
  // Drive control
  // ---------------------------------------------------------------------------

  /**
   * Commands the drivetrain to the given robot-relative chassis speeds.
   *
   * @param speeds        desired robot-relative chassis speeds
   * @param torqueCurrents per-module torque current overrides (use empty array for voltage control)
   */
  public void runVelocity(ChassisSpeeds speeds, Current[] torqueCurrents) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, constants.maxLinearSpeed.in(Units.MetersPerSecond));

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      if (torqueCurrents != null && torqueCurrents.length == 4) {
        modules[i].runSetpoint(setpointStates[i], torqueCurrents[i]);
      } else {
        modules[i].runSetpoint(setpointStates[i]);
      }
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Commands robot-relative chassis speeds with voltage control. */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, null);
  }

  /** Commands field-relative chassis speeds, rotating by current heading. */
  public void runVelocityFieldRelative(ChassisSpeeds fieldSpeeds) {
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getRotation()));
  }

  /** Stops all modules. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Rotates all four swerve modules to {@code direction} with zero drive voltage.
   *
   * <p>Call this during the disabled period before autonomous starts so modules are already
   * facing the right way when the first path command runs. Use
   * {@link org.frc5010.common.drive.swerve.auto.BLineSwerveAuto#preAlignForAuto} to obtain the
   * correct direction from a BLine {@code Path}.
   */
  public void preAlignModules(Rotation2d direction) {
    SwerveModuleState state = new SwerveModuleState(0.0, direction);
    for (Module module : modules) {
      module.runSetpoint(state);
    }
  }

  /**
   * Stops and locks wheels in an X-pattern to resist pushing.
   * Modules return to normal orientation on next nonzero velocity command.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = constants.moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // ---------------------------------------------------------------------------
  // Characterization
  // ---------------------------------------------------------------------------

  /** Runs drive motors open-loop for feedforward characterization. */
  public void runCharacterization(double output) {
    for (Module module : modules) {
      module.runCharacterization(output);
    }
  }

  /** Runs steer motors open-loop for steer feedforward characterization. */
  public void runSteerCharacterization(double output) {
    for (Module module : modules) {
      module.runSteerCharacterization(output);
    }
  }

  /** Returns the average drive velocity in rad/sec for feedforward characterization. */
  public double getDriveFFCharacterizationVelocity() {
    double output = 0.0;
    for (Module module : modules) {
      output += module.getDriveFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the average steer velocity in rad/sec for steer feedforward characterization. */
  public double getSteerFFCharacterizationVelocity() {
    double output = 0.0;
    for (Module module : modules) {
      output += module.getSteerFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns per-module drive positions in radians for wheel radius characterization. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  // ---------------------------------------------------------------------------
  // SysId
  // ---------------------------------------------------------------------------

  /** Returns a command to run a quasistatic SysId test in the given direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(getSysId().quasistatic(direction));
  }

  /** Returns a command to run a dynamic SysId test in the given direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(getSysId().dynamic(direction));
  }

  /** Returns a full SysId characterization sequence (quasistatic + dynamic, both directions). */
  public Command sysIdFullRoutine() {
    return sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(10)
        .andThen(Commands.waitSeconds(3))
        .andThen(sysIdQuasistatic(SysIdRoutine.Direction.kReverse)).withTimeout(10)
        .andThen(Commands.waitSeconds(3))
        .andThen(sysIdDynamic(SysIdRoutine.Direction.kForward)).withTimeout(4)
        .andThen(Commands.waitSeconds(3))
        .andThen(sysIdDynamic(SysIdRoutine.Direction.kReverse)).withTimeout(4);
  }

  private SysIdRoutine getSysId() {
    if (sysId == null) {
      sysId = new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second),
              Volts.of(7),
              Second.of(30),
              state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
              voltage -> runCharacterization(voltage.in(Volts)),
              null,
              this));
    }
    return sysId;
  }

  // ---------------------------------------------------------------------------
  // Pose estimation
  // ---------------------------------------------------------------------------

  /** Returns the current estimated pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current estimated heading. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Returns the raw gyro heading (yaw) as accumulated from the gyro / odometry
   * integration — the heading the gyro reports, which can diverge from
   * {@link #getRotation()} once vision corrections nudge the pose estimate.
   */
  public Rotation2d getGyroRotation() {
    return rawGyroRotation;
  }

  /**
   * Returns the true physics-body pose from the IronMaple simulation, or empty when not running
   * with physics (buildWithoutPhysics, REAL, or REPLAY mode).
   *
   * <p>Use this instead of {@link #getPose()} when you need ground-truth position in sim —
   * particularly for boundary-enforcement tests where wheel slip causes odometry to drift
   * significantly while the robot is pinned against a wall by the physics engine.
   */
  public java.util.Optional<Pose2d> getSimulatedPose() {
    if (swerveDriveSimulation != null) {
      return java.util.Optional.of(swerveDriveSimulation.getSimulatedDriveTrainPose());
    }
    return java.util.Optional.empty();
  }

  /**
   * Returns the true physics-body field-relative chassis speeds from IronMaple, or empty when
   * not in physics mode.
   *
   * <p>Use this to check whether the robot was physically stopped by a wall: when the robot is
   * commanded into a boundary, wheel encoders keep spinning (odometry drifts), but the physics
   * body's actual velocity drops to near zero.
   */
  public java.util.Optional<ChassisSpeeds> getSimulatedChassisSpeeds() {
    if (swerveDriveSimulation != null) {
      return java.util.Optional.of(
          swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative());
    }
    return java.util.Optional.empty();
  }

  /**
   * Returns the IronMaple physics drive-train simulation, or empty when not in physics-sim mode
   * ({@code buildWithoutPhysics()}, REAL, or REPLAY). Use this to pass to
   * {@code IntakeSimulation.OverTheBumperIntake()} or other IronMaple subsystem simulations.
   */
  public java.util.Optional<swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation>
      getDriveTrainSimulation() {
    return java.util.Optional.ofNullable(swerveDriveSimulation);
  }

  /**
   * Resets the pose estimator to the given pose.
   * Also resets the simulated gyro if running in simulation.
   */
  public void setPose(Pose2d pose) {
    if (RobotBase.isSimulation() && gyroIO instanceof GyroIOSim simGyro) {
      simGyro.resetYaw(pose.getRotation());
    }
    rawGyroRotation = pose.getRotation();
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Teleports both the IronMaple physics body and the pose estimator to the given pose.
   *
   * <p>Unlike {@link #setPose}, which only resets the odometry estimator, this method
   * also moves the dyn4j physics body so that {@link #getSimulatedPose()} reflects the
   * new position immediately. Use this in visual-test sequences or interactive sim whenever
   * you need physics and odometry to agree after a teleport.
   *
   * <p>Has no effect on the physics body when not in IronMaple physics mode
   * ({@code buildWithoutPhysics()}, REAL, or REPLAY).
   */
  public void resetSimulationPose(Pose2d pose) {
    if (swerveDriveSimulation != null) {
      swerveDriveSimulation.setSimulationWorldPose(pose);
    }
    setPose(pose);
  }

  /**
   * Adds a vision measurement to the pose estimator with custom standard deviations.
   *
   * @param visionPose       estimated robot pose from vision
   * @param timestampSeconds timestamp of the vision measurement
   * @param stdDevs          3-element vector of [x, y, theta] standard deviations in meters/rad
   */
  public void addVisionMeasurement(
      Pose2d visionPose,
      double timestampSeconds,
      Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  /**
   * Publishes the drivetrain stage (chassis box, steered wheels, gyro compass) onto the
   * shared {@code RobotMechanisms3D} iso canvas so the plain simulator shows the same
   * robot view as the web panel.
   */
  private void publishIsoScene() {
    SwerveModuleState[] states = getModuleStates();
    Translation2d[] translations = constants.moduleTranslations;
    double maxSpeed = Math.max(1e-6, constants.maxLinearSpeed.in(Units.MetersPerSecond));
    int n = Math.min(states.length, translations.length);
    double[][] modulesScene = new double[n][4];
    for (int i = 0; i < n; i++) {
      modulesScene[i][0] = translations[i].getX();
      modulesScene[i][1] = translations[i].getY();
      modulesScene[i][2] = states[i].angle.getRadians();
      modulesScene[i][3] = states[i].speedMetersPerSecond / maxSpeed;
    }
    MechanismVisuals3d.setRobotScene(
        constants.bumperLength.in(Units.Meters),
        constants.bumperWidth.in(Units.Meters),
        0.15,
        constants.wheelRadius.in(Units.Meters),
        modulesScene,
        getGyroRotation().getRadians());
  }

  // ---------------------------------------------------------------------------
  // State accessors
  // ---------------------------------------------------------------------------

  /** Returns the current module states (angle + velocity). */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current module positions (angle + distance). */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Returns the current robot-relative chassis speeds. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the current field-relative chassis speeds. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/FieldMeasured")
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
  }

  /** Returns the maximum configured linear speed. */
  public LinearVelocity getMaxLinearSpeed() {
    return constants.maxLinearSpeed;
  }

  /** Returns the maximum configured angular speed. */
  public AngularVelocity getMaxAngularSpeed() {
    return constants.maxAngularSpeed;
  }

  /** Returns the Field2d object for SmartDashboard / sim visualization. */
  public Field2d getField2d() {
    return field2d;
  }

  /** Returns the SwerveConstants this drivetrain was configured with. */
  public SwerveConstants getConstants() {
    return constants;
  }
}
