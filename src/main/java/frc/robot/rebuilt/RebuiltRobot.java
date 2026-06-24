package frc.robot.rebuilt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.frc5010.common.input.XboxConfigurableController;
import org.frc5010.common.drive.swerve.auto.PathPlannerToBLine;
import org.frc5010.common.profiles.SwerveRobotContainer;
import org.frc5010.examples.DemoIntake;
import frc.robot.rebuilt.subsystems.HubStatus;
import frc.robot.rebuilt.subsystems.RebuiltIndexer;
import frc.robot.rebuilt.subsystems.RebuiltIndexer.IndexerState;
import frc.robot.rebuilt.subsystems.RebuiltIntake;
import frc.robot.rebuilt.subsystems.RebuiltLauncher;
import frc.robot.rebuilt.subsystems.RebuiltLeds;

/**
 * Robot container for the ported 2026 "Rebuilt" competition robot (Team 5010).
 *
 * <p>Assembles the swerve drive (from {@link org.frc5010.examples.ExampleRobotProfile}, which
 * carries the Rebuilt drivetrain geometry) with the ported subsystems:
 * {@link RebuiltIntake}, {@link RebuiltIndexer}, {@link RebuiltLauncher}, and
 * {@link HubStatus}. The mechanism subsystems work in REAL / SIM / REPLAY via the
 * {@code org.frc5010.common.mechanisms} primitives; the IronMaple Fuel game-piece simulation
 * ({@link DemoIntake} — collection + ballistic hub scoring + web/Field2d rendering) is wired
 * in simulation only, exactly like {@link org.frc5010.examples.ExampleRobot}.
 *
 * <p>The launcher↔indexer coupling, intake↔turret interference, and intake↔game-piece
 * collection/firing are driven each cycle by an always-running {@code RebuiltCouplings}
 * command (no requirements, {@code ignoringDisable(true)}), following the same pattern the
 * web control loop uses (see CLAUDE.md gotcha 11).
 *
 * <p>Controls (work over the web UI for the six driver face buttons A/B/X/Y/LB/RB):
 * <ul>
 *   <li><b>A</b> — intake: deploy hopper to 0° + collect Fuel</li>
 *   <li><b>B</b> (hold) — launcher PREP (aim + spin up; auto-feeds when ready); release → LOW_SPEED</li>
 *   <li><b>X</b> — safe retract: stow launcher → wait for stow → retract hopper to 120°</li>
 *   <li><b>Y</b> — same as X (redundant)</li>
 *   <li><b>LB</b> (hold) — indexer HARD_CHURN (un-jam); release → idle</li>
 *   <li><b>RB</b> — launcher HAMMERTIME (safe stow)</li>
 *   <li><b>Back</b> — hopper deployed (no rollers); <b>Start</b> — hopper retracted</li>
 * </ul>
 * Firing is automatic: the coupling loop scores Fuel when the launcher is at goal and the
 * indexer is feeding (triggered by holding B).
 * Operator (Xbox port 1): A/B/X/Y hold → tower / hub / hub / forward PRESET, release → LOW_SPEED.
 */
public class RebuiltRobot extends SwerveRobotContainer {

  /** PWM header the LED strip is plugged into. */
  private static final int LED_PWM_PORT = 9;

  private RebuiltIntake intake;
  private RebuiltIndexer indexer;
  private RebuiltLauncher launcher;
  private HubStatus hubStatus;
  private RebuiltLeds leds;

  /** Sim-only game-piece layer (Fuel collection + ballistic hub scoring + web/Field2d state). */
  private DemoIntake gameSim;

  /** Operator controller (Xbox port 1) for shot presets. */
  private XboxConfigurableController operator;

  private int fireCooldown = 0;

  public RebuiltRobot() {
    super(SwerveRobotContainer.selectProfile("frc.robot.rebuilt.RebuiltRobotProfile"));
  }

  @Override
  protected void buildAutos() {
    // "Do Nothing" and a "Shoot Preload Only" routine, plus a drive-out-and-shoot. These call
    // the same launcher/indexer building blocks the source's NamedCommandsReg registers; the
    // launcher→indexer coupling and the game-piece layer fire the preload when the shot is ready.
    // No explicit start pose — these start from the profile's blueStartPose() (hub-front centre).
    addAuto("Do Nothing", Commands.none());
    addAuto("Shoot Preload", shootPreload());
    addAuto(
        "Drive Out + Shoot Preload",
        Commands.sequence(
            shootPreload(),
            Commands.run(() -> drive.runVelocityFieldRelative(new ChassisSpeeds(1.5, 0, 0)), drive)
                .withTimeout(1.5),
            Commands.runOnce(drive::stop, drive)));

    // Orbit autos — the source PathPlanner Orbit routines ported to BLine. Each auto follows
    // Bézier-sampled PathPlanner paths (carrying their embedded intake/launcher event markers)
    // composed with the same launcher/indexer state requests as the source .auto files.
    // The start pose is read from the path file's first waypoint so the sim robot spawns there.
    RebuiltAutoRoutines auto = new RebuiltAutoRoutines(drive, intake, launcher, indexer);
    addAuto("Orbit: Left",                       auto.orbitLeft(),              ps("TL-QTRH"));
    addAuto("Orbit: Right",                      auto.orbitRight(),             ps("TR-CTR-QTR"));
    addAuto("Orbit: Left 1 Swipe",               auto.orbitLeft1Swipe(),        ps("TL-QTRHLong"));
    addAuto("Orbit: Right 1 Swipe",              auto.orbitRight1Swipe(),       ps("TR-CTR-QTRLong"));
    addAuto("Orbit: Right 2 Swipe (no HP)",      auto.orbitRight2Swipe(),       ps("TR-CTR-QTRAngled"));
    addAuto("Orbit: Churn Right 2 Swipe (no HP)",auto.churnOrbitRight2Swipe(), ps("TR-CTR-QTRAngled"));

    // Delay Trench / Disrupt
    addAuto("Delay Trench Neutral Bump HP", auto.delayTrenchNeutralBumpHP(), ps("StartTR-CTR-HLF-BR-HP"));
    addAuto("Disrupt",                      auto.disrupt(),                  ps("RightDisrupt1"));
    // Follow — robot starts at the shooting position then drives; pose comes from path's start
    addAuto("Follow: Left Bump Depot",   auto.followLeftBumpDepot(),   ps("TLBack-CTL-QTL-BL-L"));
    addAuto("Follow: Left Trench",       auto.followLeftTrench(),       ps("TLback-CTL-QTL"));
    addAuto("Follow: Right Bump HP",     auto.followRightBumpHP(),     ps("DelayTRS-CTR-QTR-BR-HP Longer"));
    addAuto("Follow: Right Trench HP",   auto.followRightTrenchHP(),   ps("DelayTR-QTRL"));
    // Left
    addAuto("Left: 2056 Double HP",  auto.left2056DoubleHP(),  ps("TL-CTR-QTL-BL-TL"));
    addAuto("Left: 5010 Double",     auto.left5010Double(),    ps("TL-CTL-QTL"));
    addAuto("Left: 3 Shuttle HPC",   auto.left3ShuttleHPC(),   ps("TL-QTLCTLSHORT"));
    // Quals
    addAuto("Quals 110", auto.quals110(), ps("TLback-CTL-QTL"));
    addAuto("Quals 73",  auto.quals73(),  ps("DelayTR-QTRL"));
    // Right
    addAuto("Right: 2056 Double HP",         auto.right2056DoubleHP(),         ps("TR-CTR-QTR-BR-TR"));
    addAuto("Right: 5010 Double",            auto.right5010Double(),            ps("TRSide-CTR-QTR"));
    addAuto("Right: 5010 Double (Old)",      auto.right5010DoubleOld(),         ps("TRSide-CTR-QTR"));
    addAuto("Right: 5010 Double (Optimized)",auto.right5010DoubleOptimized(),  ps("TRSide-CTR-QTR"));
    addAuto("Right: 5010 Double (Short)",    auto.right5010DoubleShort(),       ps("TR-CTR-QTRShort"));
    addAuto("Right: 3 Shuttle HPC",          auto.right3ShuttleHPC(),           ps("TR-QTRCTRSHORT"));
  }

  /** Loads the starting {@link Pose2d} from the first waypoint of a PathPlanner path file. */
  private static Pose2d ps(String pathName) {
    return PathPlannerToBLine.loadStartPose(pathName);
  }

  /**
   * Stows the launcher + idles the indexer, waits up to 2 s for the turret and hood to reach
   * safe stow, then retracts the hopper. Safe to call from both teleop bindings and auto.
   */
  private edu.wpi.first.wpilibj2.command.Command safeRetractCommand() {
    return Commands.sequence(
            Commands.parallel(launcher.hammertimeCommand(), indexer.idleCommand()),
            Commands.waitUntil(launcher::isStowed).withTimeout(2.0),
            intake.retractCommand())
        .withName("SafeRetract");
  }

  /** Deploy the intake, PREP the launcher, auto-feed the preload, then safe-retract. */
  private edu.wpi.first.wpilibj2.command.Command shootPreload() {
    return Commands.sequence(
            intake.intakeCommand(() -> Constants.Intake.INTAKE_IN), // deploy hopper
            Commands.waitUntil(intake::isExtended).withTimeout(2.0),
            launcher.prepCommand(),
            Commands.waitSeconds(3.5), // spin up, aim, and auto-feed the preload via the coupling
            launcher.lowSpeedCommand())
        .withName("Auto/ShootPreload");
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

    Supplier<Pose2d> pose = drive::getPose;
    Supplier<ChassisSpeeds> fieldVelocity =
        () ->
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drive.getChassisSpeeds(), drive.getPose().getRotation());

    // Mechanism subsystems — valid in all robot modes.
    intake = new RebuiltIntake(pose);
    indexer = new RebuiltIndexer();
    launcher = new RebuiltLauncher(pose, fieldVelocity);
    hubStatus = new HubStatus();

    // Cross-subsystem gates.
    launcher.setTurretBlocked(intake::isBlockingTurret);
    indexer.setFlywheelReadyForChurn(launcher::isFlywheelReadyForChurn);

    registerMechanism(intake::close);
    registerMechanism(indexer::close);
    registerMechanism(launcher::close);

    configureDriverBindings();
    configureOperatorBindings();

    // Sim-only game-piece simulation + LEDs + the coupling loop that drives it. (On a real
    // robot the LED strip would be created unconditionally, outside this sim guard.)
    if (RobotBase.isSimulation()) {
      drive
          .getDriveTrainSimulation()
          .ifPresent(
              driveSim -> gameSim = new DemoIntake(driveSim, pose, drive.getField2d()));

      leds =
          new RebuiltLeds(
              LED_PWM_PORT,
              launcher::getState,
              indexer::isFeeding,
              intake::isExtended,
              () -> true); // turret is zeroed at power-on in sim
      registerMechanism(leds::close);
    }

    // Always-running couplings (runs while disabled too, no subsystem requirements).
    CommandScheduler.getInstance()
        .schedule(
            Commands.run(this::updateCouplings).ignoringDisable(true).withName("RebuiltCouplings"));
  }

  // ── controls ───────────────────────────────────────────────────────────────

  private void configureDriverBindings() {
    controller.a().onTrue(intake.intakeCommand(() -> Constants.Intake.INTAKE_IN));
    controller.b().onTrue(launcher.prepCommand()).onFalse(launcher.lowSpeedCommand());
    // X/Y/Start retract: stow launcher first, wait for turret+hood to reach safe stow, then
    // retract the hopper — prevents retracting through the turret's rotation plane.
    controller.x().onTrue(safeRetractCommand());
    controller.y().onTrue(safeRetractCommand());
    controller.leftBumper().onTrue(indexer.hardChurnCommand()).onFalse(indexer.idleCommand());
    controller.rightBumper().onTrue(launcher.hammertimeCommand());
    controller.back().onTrue(intake.deployCommand());
    controller.start().onTrue(safeRetractCommand());
  }

  private void configureOperatorBindings() {
    operator = new XboxConfigurableController(1);
    operator.a().onTrue(launcher.presetCommand(RebuiltLauncher.towerPreset()))
        .onFalse(launcher.lowSpeedCommand());
    operator.b().onTrue(launcher.presetCommand(RebuiltLauncher.hubPreset()))
        .onFalse(launcher.lowSpeedCommand());
    operator.x().onTrue(launcher.presetCommand(RebuiltLauncher.hubPreset()))
        .onFalse(launcher.lowSpeedCommand());
    operator.y().onTrue(launcher.presetCommand(RebuiltLauncher.forwardPreset()))
        .onFalse(launcher.lowSpeedCommand());
    operator.rightBumper().onTrue(intake.angledCommand())
        .onFalse(intake.intakeCommand(() -> Constants.Intake.INTAKE_IN));
    operator.leftBumper().onTrue(indexer.forceCommand()).onFalse(indexer.churnCommand());
    // Flywheel multiplier nudge on operator Back/Start (POV in the source).
    operator.back().onTrue(
        Commands.runOnce(() -> nudgeFlywheelMultiplier(-0.01)).ignoringDisable(true));
    operator.start().onTrue(
        Commands.runOnce(() -> nudgeFlywheelMultiplier(+0.01)).ignoringDisable(true));
  }

  private static void nudgeFlywheelMultiplier(double delta) {
    frc.robot.rebuilt.subsystems.ShotCalculator.incrementFlywheelMultiplier(delta);
  }

  // ── couplings (run every cycle) ────────────────────────────────────────────

  private void updateCouplings() {
    // Launcher → indexer: while prepping, feed when at goal else churn to keep Fuel agitated.
    if (launcher.isPrepping()) {
      indexer.setRequestedState(launcher.isReadyToFire() ? IndexerState.FEED : IndexerState.CHURN);
    }

    if (gameSim == null) return;

    // Intake → game-piece collection.
    boolean collecting = intake.isCollecting();
    if (collecting && !gameSim.isIntakeExtended()) {
      CommandScheduler.getInstance().schedule(gameSim.extendCommand());
    } else if (!collecting && gameSim.isIntakeExtended()) {
      CommandScheduler.getInstance().schedule(gameSim.retractCommand());
    }

    // Launcher ready + indexer feeding → score Fuel at a controlled cadence.
    if (fireCooldown > 0) fireCooldown--;
    if (launcher.isReadyToFire() && indexer.isFeeding() && gameSim.getHeldFuel() > 0 && fireCooldown == 0) {
      CommandScheduler.getInstance().schedule(gameSim.fireCommand());
      fireCooldown = 5; // 50 Hz ÷ 5 = 10 fuel/sec
    }
  }

  // ── accessors (for tests) ──────────────────────────────────────────────────

  public RebuiltIntake getIntake() {
    return intake;
  }

  public RebuiltIndexer getIndexer() {
    return indexer;
  }

  public RebuiltLauncher getLauncher() {
    return launcher;
  }

  /** The sim game-piece layer, if present (simulation only). */
  public java.util.Optional<DemoIntake> getGameSim() {
    return java.util.Optional.ofNullable(gameSim);
  }
}
