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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.input.DriveVector;
import org.frc5010.common.input.XboxConfigurableController;
import org.frc5010.common.input.WebXboxController;
import org.frc5010.common.input.JoystickAxis;
import org.frc5010.common.sim.SwerveVisualTest;
import org.frc5010.common.sim.WebControl;
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
 * public class ExampleRobot extends SwerveRobotContainer {
 *   private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
 *       .moduleType(ModuleType.SIM).gyroType(GyroType.SIM).build();
 *   private static final Pose2d BLUE_START = new Pose2d(1.5, 2.0, new Rotation2d());
 *
 *   public ExampleRobot() {
 *     super(SwerveRobotContainer.selectProfile("org.frc5010.examples.ExampleRobotProfile"));
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

  // Glass keyboard joystick 0 (simgui-ds.json) mirrors a real Xbox pad — 6 axes, 10 buttons,
  // 1 POV — so the same standard mapping drives sim and hardware:
  //   axis 0 (Left X)  — A(dec) / D(inc)  →  strafe
  //   axis 1 (Left Y)  — W(dec) / S(inc)  →  forward/back (W = negative axis)
  //   axis 4 (Right X) — E(dec) / Q(inc)  →  rotation   (axis 2 is the left trigger, unused)
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

  /** Web UI control facade. Non-null only when {@code -PwebUI} is set. */
  protected WebControl webControl = null;

  /**
   * Lazy auto factories registered by name, in display order. Populated by {@link #buildAutos()}.
   * Each factory is invoked at most once per selection change; the result is cached in
   * {@link #builtAutoCommand}.
   */
  protected final LinkedHashMap<String, Supplier<Command>> autoFactories = new LinkedHashMap<>();

  /** Blue-alliance start pose for each auto that declares one. Used by {@link #resetToAllianceStart()} in sim. */
  private final Map<String, Pose2d> autoStartPoses = new LinkedHashMap<>();

  /** SmartDashboard chooser for auto selection (stores names; factory is invoked lazily). */
  protected final SendableChooser<String> autoChooser = new SendableChooser<>();

  /** Auto selected from the web UI driver-station panel. Written by the web-select callback. */
  private volatile String webSelectedAuto;

  /** Name of the most recently built auto, or {@code null} if none has been built yet. */
  private String builtAutoName = null;

  /** The most recently built autonomous command (lazy-built from the factory on selection change). */
  private Command builtAutoCommand = null;

  /**
   * Close handles for mechanism subsystems registered via {@link #registerMechanism(Runnable)}.
   * Static so tests that construct robot containers can free hardware handles (CAN
   * devices, background threads) in teardown — {@code CommandScheduler.unregisterAllSubsystems()}
   * does NOT release them, and stale handles would collide with later tests in the
   * same JVM.
   */
  private static final java.util.List<Runnable> mechanismCloseables =
      new java.util.ArrayList<>();

  /**
   * Registers a cleanup hook for a mechanism subsystem (typically {@code mechanism::close}).
   * Register every mechanism that owns background resources — closed-loop threads, CAN
   * devices — so tests and tooling can tear the robot down cleanly. Run and removed by
   * {@link #closeMechanisms()}.
   *
   * @param closer cleanup to run when mechanisms are torn down
   */
  protected static void registerMechanism(Runnable closer) {
    mechanismCloseables.add(closer);
  }

  /** Stops and frees all registered mechanisms. Call from test teardown. */
  public static void closeMechanisms() {
    mechanismCloseables.forEach(Runnable::run);
    mechanismCloseables.clear();
  }

  // Stored when constructed from a RobotProfile; null when constructed from a bare drive.
  private final RobotProfile profile;

  /**
   * Selects the appropriate {@link RobotProfile} for the current execution context.
   *
   * <p>Returns {@link SimRobotProfile} when {@code -PtestSim} is set. Otherwise
   * reflectively instantiates the class named by {@code realProfileClassName}
   * (no-arg constructor required). Teams can subclass {@code ExampleRobotProfile}
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
    // The profile is the authority on the field: publish its AprilTag layout to the shared
    // holder BEFORE building the drive/vision (and before any FieldConstants class loads), so
    // pose estimation and field geometry all follow the profile's chosen field variant.
    org.frc5010.common.vision.AprilTags.setAprilTagFieldLayout(profile.getAprilTagFieldLayout());
    this.drive    = profile.createDrive();
    this.vision   = profile.createVision(this.drive);
    boolean webUI = RobotBase.isSimulation() && Boolean.getBoolean("webUI");
    this.controller = webUI ? new WebXboxController(controllerPort)
                            : new XboxConfigurableController(controllerPort);
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
    boolean webUI = RobotBase.isSimulation() && Boolean.getBoolean("webUI");
    this.controller = webUI ? new WebXboxController(controllerPort)
                            : new XboxConfigurableController(controllerPort);
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
   * Defaults to the value from the {@link RobotProfile} when one was provided, or the
   * 2026 field length (16.540988 m) otherwise. Override to use a different year's field.
   */
  protected Distance getFieldLength() {
    return profile != null ? profile.getFieldLength() : Meters.of(16.540988);
  }

  /**
   * Returns the autonomous command.
   *
   * <p>Priority order:
   * <ol>
   *   <li>When {@code -PvisualTest} is set, returns the {@link SwerveVisualTest} sequence.</li>
   *   <li>When the web UI is active and an auto has been selected, returns the web-selected auto.</li>
   *   <li>Otherwise returns the SmartDashboard {@link SendableChooser} selection (registered via
   *       {@link #addAuto} in {@link #buildAutos}).</li>
   * </ol>
   */
  public Command getAutonomousCommand() {
    if (Boolean.getBoolean("visualTest")) {
      return SwerveVisualTest.build(drive, vision, this::getAllianceStartPose);
    }
    return builtAutoCommand;
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
      // WebXboxController was created in the constructor; cast is guaranteed safe.
      webControl = new WebControl(drive, (WebXboxController) controller, this::resetToAllianceStart);
      // Surface the AprilTags currently in view so the web field highlights them.
      if (vision != null) webControl.bindVision(vision);
    }

    // Standard Xbox axes: left stick translates, right-stick X rotates. The Glass keyboard
    // joystick (simgui-ds.json) is configured to mirror an Xbox pad — 6 axes / 10 buttons / 1 POV,
    // with W/S → leftY, A/D → leftX, E/Q → rightX — so the same mapping drives the sim keyboard
    // and a real controller. (On an Xbox pad axis 2 is the left trigger, NOT rotation.)
    JoystickAxis forward  = controller.leftY().negate().deadzone(0.05);
    JoystickAxis strafe   = controller.leftX().negate().deadzone(0.05);
    JoystickAxis rotation = controller.rightX().negate().deadzone(0.05);
    DriveVector translate = DriveVector.of(forward, strafe).unitCircle();

    drive.setDefaultCommand(
        Commands.run(
            () -> {
              double flip =
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                      ? -1.0 : 1.0;

              if (webControl != null && webControl.isConnected()) {
                ChassisSpeeds web = webControl.getChassisSpeeds();
                drive.runVelocityFieldRelative(new ChassisSpeeds(
                    flip * web.vxMetersPerSecond,
                    flip * web.vyMetersPerSecond,
                    web.omegaRadiansPerSecond));
                return;
              }
              if (webControl != null && webControl.isStale()) {
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

    // Defer auto construction to the first scheduler tick so that all subclass constructors
    // have completed (and fields like demoIntake are set) before buildAutos() is called.
    //
    // This MUST run its body from execute(), not initialize(): Commands.runOnce() is an
    // InstantCommand that runs its body in initialize(), and CommandScheduler.schedule()
    // initializes synchronously when called outside the run loop — i.e. right here, still
    // inside the subclass constructor chain, before fields like demoIntake are assigned.
    // A bare runOnce would therefore build the autos too early and silently drop any
    // routine gated on a not-yet-constructed field. Leading with waitSeconds(0) pushes the
    // build onto the first run() tick, which is genuinely after construction completes.
    // ignoringDisable(true) ensures this runs while the robot is still disabled at startup.
    // Sequence: wait one tick (so subclass constructor fields are set) → build & register
    // all factories → then poll every cycle to build the selected auto lazily.
    // pollAndBuildSelectedAuto() is also called immediately after finalizeAutos() to build
    // the default selection on the very first tick (required by autonomousCommandIsNonNull test).
    CommandScheduler.getInstance().schedule(
        Commands.sequence(
                Commands.waitSeconds(0),
                Commands.runOnce(() -> {
                  buildAutos();
                  finalizeAutos();
                  pollAndBuildSelectedAuto();
                }),
                Commands.run(this::pollAndBuildSelectedAuto))
            .ignoringDisable(true)
            .withName("BuildAndPollAutos"));
  }

  /**
   * Registers a named autonomous routine with a lazy factory.
   *
   * <p>The factory is invoked only when this auto is selected (polled each cycle while disabled),
   * not at registration time — so expensive work like loading PathPlanner path files is deferred
   * until the operator actually picks the routine. The first call sets the SmartDashboard chooser
   * default; subsequent calls add options in insertion order.
   *
   * @param name    display name shown in the chooser and web UI dropdown
   * @param factory called once per selection change to build the autonomous command
   */
  protected void addAuto(String name, Supplier<Command> factory) {
    if (autoFactories.isEmpty()) {
      autoChooser.setDefaultOption(name, name);
    } else {
      autoChooser.addOption(name, name);
    }
    autoFactories.put(name, factory);
  }

  /**
   * Registers a named autonomous routine with a lazy factory and its blue-alliance starting pose.
   *
   * <p>In simulation, {@link #resetToAllianceStart()} teleports the robot to this pose
   * (mirrored for Red alliance) rather than the profile's generic starting pose. Use this
   * overload for path-following autos whose first waypoint is known at build time.
   *
   * @param name          display name shown in the chooser and web UI dropdown
   * @param factory       called once per selection change to build the autonomous command
   * @param blueStartPose blue-alliance starting pose (the path's first waypoint)
   */
  protected void addAuto(String name, Supplier<Command> factory, Pose2d blueStartPose) {
    addAuto(name, factory);
    if (blueStartPose != null) {
      autoStartPoses.put(name, blueStartPose);
    }
  }

  /**
   * Registers a named autonomous routine (eager overload for backward compatibility).
   *
   * <p>The command is wrapped as {@code () -> cmd} so the same lazy polling path is used.
   * Since the command object is already constructed, the "build" cost is trivial.
   *
   * @param name display name shown in the chooser and web UI dropdown
   * @param cmd  the already-constructed command to schedule when this auto is selected
   */
  protected void addAuto(String name, Command cmd) {
    addAuto(name, () -> cmd);
  }

  /**
   * Registers a named autonomous routine with its blue-alliance starting pose (eager overload).
   *
   * @param name          display name shown in the chooser and web UI dropdown
   * @param cmd           the already-constructed command to schedule when this auto is selected
   * @param blueStartPose blue-alliance starting pose (the path's first waypoint)
   */
  protected void addAuto(String name, Command cmd, Pose2d blueStartPose) {
    addAuto(name, () -> cmd, blueStartPose);
  }

  /**
   * Registers all entries from an {@link AutoEntry} list in one call.
   *
   * <p>Convenience for {@link #buildAutos()} implementations that return a list (e.g. from a
   * helper class):
   * <pre>{@code
   * protected void buildAutos() {
   *   registerAutos(new MyAutoRoutines(drive, ...).allAutos());
   * }
   * }</pre>
   *
   * @param entries the auto entries to register, in display order
   */
  protected final void registerAutos(Collection<AutoEntry> entries) {
    for (AutoEntry e : entries) {
      addAuto(e.name(), e.factory(), e.blueStart());
    }
  }

  /**
   * Override to register game-specific autonomous routines via {@link #addAuto}.
   *
   * <p>Called automatically on the first scheduler tick (first {@code robotPeriodic()} cycle),
   * so all subsystems and fields are fully initialized when this runs. The base implementation
   * does nothing — subclasses do not need to call {@code super.buildAutos()}.
   */
  protected void buildAutos() {}

  /**
   * Publishes the auto chooser to SmartDashboard and binds the registered autos to the web UI
   * driver-station panel. Called automatically after {@link #buildAutos()} completes.
   */
  private void finalizeAutos() {
    SmartDashboard.putData("Auto Mode", autoChooser);
    webSelectedAuto = autoFactories.isEmpty() ? null : autoFactories.keySet().iterator().next();
    if (webControl != null && !autoFactories.isEmpty()) {
      webControl.bindAutos(
          autoFactories.keySet().toArray(new String[0]),
          webSelectedAuto,
          name -> { if (autoFactories.containsKey(name)) webSelectedAuto = name; });
    }
  }

  /**
   * Returns a {@link Trigger} that is active while web interface button {@code idx} is held.
   * Indices: 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB.
   * Returns a never-active trigger when not in simulation or web UI is not active.
   */
  public Trigger webButton(int idx) {
    return webControl != null ? webControl.button(idx) : new Trigger(() -> false);
  }

  // ---------------------------------------------------------------------------
  // Concrete helpers
  // ---------------------------------------------------------------------------

  /**
   * Teleports the physics body and pose estimator to the alliance-correct starting pose.
   * In simulation, prefers the selected auto's registered start pose (set via
   * {@link #addAuto(String, Command, Pose2d)}) over the profile's generic start.
   * Call this from {@code autonomousInit()} and {@code teleopInit()} in simulation.
   */
  public void resetToAllianceStart() {
    Pose2d blueStart = getSelectedAutoBlueStart();
    drive.resetSimulationPose(blueStart != null ? mirrorForAlliance(blueStart) : getAllianceStartPose());
  }

  /** Returns the name of the currently selected auto (web UI takes priority over chooser). */
  private String getSelectedAutoName() {
    if (webControl != null && webSelectedAuto != null) return webSelectedAuto;
    return autoChooser.getSelected();
  }

  /**
   * Polls the auto selection and builds the selected auto if it has changed since the last poll.
   * Invoked each scheduler cycle by the always-running {@code BuildAndPollAutos} command, and
   * also called once immediately after {@link #finalizeAutos()} to build the default selection.
   */
  private void pollAndBuildSelectedAuto() {
    String name = getSelectedAutoName();
    if (name == null || name.equals(builtAutoName)) return;
    Supplier<Command> factory = autoFactories.get(name);
    if (factory == null) return;
    builtAutoCommand = factory.get();
    builtAutoName = name;
  }

  /**
   * Returns the blue-alliance start pose registered for the currently selected auto,
   * or {@code null} if no pose was registered for it (falls back to the profile start).
   */
  private Pose2d getSelectedAutoBlueStart() {
    return autoStartPoses.get(getSelectedAutoName());
  }

  /**
   * Applies alliance mirroring to a blue-alliance pose: flips X across the field centre
   * and rotates heading 180° for Red; returns the pose unchanged for Blue.
   */
  private Pose2d mirrorForAlliance(Pose2d blue) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return new Pose2d(
          getFieldLength().in(Meters) - blue.getX(),
          blue.getY(),
          blue.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    return blue;
  }

  /**
   * Computes the alliance-correct starting pose from {@link #getBlueAllianceStartPose()}.
   * On Red alliance, mirrors x across the field centre and adds 180° to the heading.
   */
  protected final Pose2d getAllianceStartPose() {
    return mirrorForAlliance(getBlueAllianceStartPose());
  }
}
