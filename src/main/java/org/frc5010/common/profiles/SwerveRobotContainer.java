package org.frc5010.common.profiles;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.input.DriveVector;
import org.frc5010.common.input.XboxConfigurableController;
import org.frc5010.common.input.JoystickAxis;
import org.frc5010.common.sim.SwerveVisualTest;
import org.frc5010.common.sim.WebDriveController;
import org.frc5010.common.vision.Vision;

/**
 * Base class for a swerve-drive robot container.
 *
 * <p>Provides the standard wiring used in both simulation and on real hardware:
 * <ul>
 *   <li>Keyboard / joystick drive default command (Glass joystick 0 in sim; override for hardware)</li>
 *   <li>Alliance-aware starting pose ({@link #resetToAllianceStart()})</li>
 *   <li>Autonomous command dispatch ({@link #getAutonomousCommand()}, with {@link SwerveVisualTest}
 *       enabled by the {@code -PvisualTest} Gradle flag)</li>
 * </ul>
 *
 * <h3>Minimal subclass for simulation</h3>
 * <pre>{@code
 * public class RealRobot extends SwerveRobotContainer {
 *   private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
 *       .moduleType(ModuleType.SIM).gyroType(GyroType.SIM).build();
 *   private static final Pose2d BLUE_START = new Pose2d(1.5, 2.0, new Rotation2d());
 *
 *   public RealRobot() {
 *     super(SwerveRobotContainer.selectProfile("frc.robot.RealRobotProfile"));
 *   }
 * }
 * }</pre>
 *
 * <h3>Customising bindings</h3>
 * <p>Override {@link #configureBindings()} to add button/trigger bindings on top of (or instead of)
 * the default keyboard drive. Call {@code super.configureBindings()} first to keep the keyboard
 * drive and only add your own bindings afterwards.
 *
 * <h3>Customising auto</h3>
 * <p>Override {@link #getAutonomousCommand()} to integrate PathPlanner, Choreo, or any other
 * auto framework. The base implementation handles the {@code -PvisualTest} flag.
 */
public abstract class SwerveRobotContainer {

  // Glass keyboard joystick 0 (simgui-ds.json):
  //   axis 0 — A(dec) / D(inc)  →  strafe
  //   axis 1 — W(dec) / S(inc)  →  forward/back (W = negative axis)
  //   axis 2 — E(dec) / R(inc)  →  rotation
  /** The primary driver controller. Accessible to subclasses for additional bindings. */
  protected final XboxConfigurableController controller;

  /** The swerve drive subsystem. Available to subclasses for commands and bindings. */
  protected final AkitSwerveDrive drive;

  /**
   * Vision subsystem. Set this in a subclass constructor after calling {@code super(...)} to
   * enable vision-based pose correction and the visual-test push-correction step (Step 6).
   * Leave {@code null} to skip vision entirely.
   */
  protected Vision vision = null;

  /** Browser-based field visualization and virtual controller. Non-null only when {@code -PwebUI} is set. */
  protected WebDriveController webController = null;

  // Stored when constructed from a RobotProfile; null when constructed from a bare drive.
  private final RobotProfile profile;

  /**
   * Selects the appropriate {@link RobotProfile} for the current execution context.
   *
   * <p>Returns {@link SimRobotProfile} when {@code -PtestSim} is set. Otherwise
   * reflectively instantiates the class named by {@code realProfileClassName}
   * (no-arg constructor required). Teams can subclass {@code RealRobotProfile}
   * and pass the subclass name here without changing any common-library code.
   *
   * <p>Note: the named class is not instantiated in {@code testSim} mode. If
   * its constructor ever performs hardware I/O, this avoids a spurious side effect.
   *
   * @param realProfileClassName fully-qualified class name of the real robot profile
   * @throws RuntimeException if the class cannot be found or instantiated
   */
  public static RobotProfile selectProfile(String realProfileClassName) {
    if (Boolean.getBoolean("testSim")) return new SimRobotProfile();
    try {
      Class<?> clazz = Class.forName(realProfileClassName);
      return (RobotProfile) clazz.getDeclaredConstructor().newInstance();
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("Cannot instantiate robot profile: " + realProfileClassName, e);
    }
  }

  /**
   * Constructs the container from a {@link RobotProfile}.
   *
   * <p>The profile supplies both the drive and the starting pose, so subclasses need
   * not override {@link #getBlueAllianceStartPose()} or {@link #getFieldLength()}.
   *
   * @param profile the robot-specific configuration
   */
  protected SwerveRobotContainer(RobotProfile profile) {
    this(profile, 0);
  }

  /**
   * Constructs the container from a {@link RobotProfile} on the specified joystick port.
   *
   * @param profile        the robot-specific configuration
   * @param controllerPort WPILib joystick port (0 = Glass keyboard in sim)
   */
  protected SwerveRobotContainer(RobotProfile profile, int controllerPort) {
    this.profile  = profile;
    this.drive    = profile.createDrive();
    this.vision   = profile.createVision(this.drive);
    this.controller = new XboxConfigurableController(controllerPort);
    configureBindings();
  }

  /**
   * Constructs the container with the given drive subsystem on joystick port 0.
   *
   * <p>When using this constructor, subclasses must override
   * {@link #getBlueAllianceStartPose()}.
   *
   * @param drive the fully-wired swerve drive subsystem
   */
  protected SwerveRobotContainer(AkitSwerveDrive drive) {
    this(drive, 0);
  }

  /**
   * Constructs the container with the given drive subsystem on the specified joystick port.
   *
   * <p>When using this constructor, subclasses must override
   * {@link #getBlueAllianceStartPose()}.
   *
   * @param drive          the fully-wired swerve drive subsystem
   * @param controllerPort WPILib joystick port (0 = Glass keyboard in sim)
   */
  protected SwerveRobotContainer(AkitSwerveDrive drive, int controllerPort) {
    this.profile  = null;
    this.drive    = drive;
    this.controller = new XboxConfigurableController(controllerPort);
    configureBindings();
  }

  // ---------------------------------------------------------------------------
  // Required override (only when constructed without a RobotProfile)
  // ---------------------------------------------------------------------------

  /**
   * Returns the robot's starting pose on the Blue alliance side.
   *
   * <p>Used by {@link #resetToAllianceStart()} to compute the correct pose for either alliance.
   * The Red-alliance pose is derived by mirroring across the field centre line.
   *
   * <p>When constructed via {@link #SwerveRobotContainer(RobotProfile)}, this delegates to
   * {@link RobotProfile#getBlueAllianceStartPose()} automatically and need not be overridden.
   */
  protected Pose2d getBlueAllianceStartPose() {
    if (profile != null) return profile.getBlueAllianceStartPose();
    throw new UnsupportedOperationException(
        "Override getBlueAllianceStartPose() when constructing without a RobotProfile");
  }

  // ---------------------------------------------------------------------------
  // Optional overrides
  // ---------------------------------------------------------------------------

  /**
   * Returns the field length for this year's game.
   * Defaults to the value from the {@link RobotProfile} when one was provided, or
   * Arena2026Rebuilt (16.540988 m) otherwise. Override to use a different year's field.
   */
  protected Distance getFieldLength() {
    return profile != null ? profile.getFieldLength() : Meters.of(16.540988);
  }

  /**
   * Returns the autonomous command.
   *
   * <p>When the {@code visualTest} system property is set (via {@code -PvisualTest} on the
   * Gradle command line), returns the {@link SwerveVisualTest} sequence. Otherwise returns
   * {@code null}. Override to integrate PathPlanner, Choreo, etc.
   */
  public Command getAutonomousCommand() {
    if (Boolean.getBoolean("visualTest")) {
      return SwerveVisualTest.build(drive, vision, this::getAllianceStartPose);
    }
    return null;
  }

  /**
   * Configures default commands and button bindings.
   *
   * <p>The default implementation wires joystick 0 to field-relative WASD keyboard drive,
   * alliance-aware (Red flips vx/vy so "forward" always faces the opponent's wall).
   *
   * <p>Override to add button/trigger bindings or replace the default command. Calling
   * {@code super.configureBindings()} first preserves the keyboard drive while adding new bindings.
   */
  protected void configureBindings() {
    if (RobotBase.isSimulation() && Boolean.getBoolean("webUI")) {
      webController = new WebDriveController(drive);
      webController.start();

      // Inject web button suppliers so controller.leftBumper() / .a() etc. auto-OR
      // with the corresponding web UI button — no manual || webButton(n) needed.
      java.util.function.BooleanSupplier[] webBtns = new java.util.function.BooleanSupplier[6];
      for (int i = 0; i < webBtns.length; i++) webBtns[i] = webController.getButton(i);
      controller.setWebInputs(webBtns);

      // Apply pending enable/alliance changes from the web interface on a command that
      // runs even while the robot is disabled. The drive default command cannot do this:
      // WPILib does not run subsystem default commands while disabled, so the very click
      // that enables the robot would never be processed (catch-22).
      CommandScheduler.getInstance().schedule(
          Commands.run(() -> webController.applyPendingControl(this::resetToAllianceStart))
              .ignoringDisable(true)
              .withName("WebControlApply"));
    }

    JoystickAxis forward  = controller.axis(1).negate().deadzone(0.05);
    JoystickAxis strafe   = controller.axis(0).negate().deadzone(0.05);
    JoystickAxis rotation = controller.axis(2).negate().deadzone(0.05);
    DriveVector translate = DriveVector.of(forward, strafe).unitCircle();

    drive.setDefaultCommand(
        Commands.run(
            () -> {
              double flip =
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                      ? -1.0 : 1.0;

              if (webController != null && webController.isConnected()) {
                ChassisSpeeds web = webController.getChassisSpeeds();
                drive.runVelocityFieldRelative(new ChassisSpeeds(
                    flip * web.vxMetersPerSecond,
                    flip * web.vyMetersPerSecond,
                    web.omegaRadiansPerSecond));
                return;
              }
              if (webController != null && webController.isStale()) {
                drive.stop();
                return;
              }
              Translation2d xy = translate.get();
              drive.runVelocityFieldRelative(
                  new ChassisSpeeds(
                      flip * xy.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond),
                      flip * xy.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond),
                      rotation.getAsDouble() * drive.getMaxAngularSpeed().in(RadiansPerSecond)
                  )
              );
            },
            drive
        ).withName("KeyboardDrive")
    );
  }

  /**
   * Returns a {@link Trigger} that is active while web interface button {@code idx} is held.
   * Indices: 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB.
   * Returns a never-active trigger when not in simulation.
   */
  public Trigger webButton(int idx) {
    return new Trigger(webController != null ? webController.getButton(idx) : () -> false);
  }

  // ---------------------------------------------------------------------------
  // Concrete helpers
  // ---------------------------------------------------------------------------

  /**
   * Teleports the physics body and pose estimator to the alliance-correct starting pose.
   * Call this from {@code autonomousInit()} and {@code teleopInit()} in simulation.
   */
  public void resetToAllianceStart() {
    drive.resetSimulationPose(getAllianceStartPose());
  }

  /**
   * Computes the alliance-correct starting pose from {@link #getBlueAllianceStartPose()}.
   * On Red alliance, mirrors x across the field centre and adds 180° to the heading.
   */
  protected final Pose2d getAllianceStartPose() {
    Pose2d blue = getBlueAllianceStartPose();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return new Pose2d(
          getFieldLength().in(Meters) - blue.getX(),
          blue.getY(),
          blue.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    return blue;
  }
}
