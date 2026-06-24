package frc.robot.rebuilt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.frc5010.common.input.XboxConfigurableController;
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
    var auto = new RebuiltAutoRoutines(drive, intake, launcher, indexer);
    for (var e : auto.allAutos()) {
      addAuto(e.name(), e.factory(), e.blueStart());
    }
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
