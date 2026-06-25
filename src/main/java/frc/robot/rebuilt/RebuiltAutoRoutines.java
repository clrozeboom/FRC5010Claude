package frc.robot.rebuilt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.rebuilt.subsystems.RebuiltIndexer;
import frc.robot.rebuilt.subsystems.RebuiltIntake;
import frc.robot.rebuilt.subsystems.RebuiltLauncher;
import java.util.List;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.auto.BLineSwerveAuto;
import org.frc5010.common.profiles.AutoEntry;

/**
 * The 2026 "Rebuilt" robot's <b>Orbit</b> autonomous routines, ported from the source robot's
 * PathPlanner autos to BLine.
 *
 * <p>The source autos are sequences of <em>named commands</em> (subsystem state requests),
 * <em>path follows</em>, and <em>waits</em>. The paths themselves carry the geometry plus
 * embedded <em>event markers</em> (e.g. {@code intakeIntake} part-way down a path) and
 * <em>rotation targets</em>. This port preserves all three:
 *
 * <ul>
 *   <li>PathPlanner {@code .path} files are converted to BLine paths by
 *       {@link PathPlannerToBLine} (Bézier-sampled), so the embedded event markers and rotation
 *       targets travel with the geometry.</li>
 *   <li>{@link #registerEvents()} binds each event-marker / named-command key to the matching
 *       ported subsystem command via {@link FollowPath#registerEventTrigger(String, Command)} —
 *       the BLine analogue of PathPlanner's {@code NamedCommands}.</li>
 *   <li>The named commands that appear in the auto sequences themselves (not as path events) are
 *       scheduled inline from the same subsystem command factories.</li>
 * </ul>
 *
 * <p>The launcher → indexer feed coupling and the sim game-piece firing are handled by
 * {@code RebuiltRobot}'s always-running coupling loop, exactly as in teleop — so a {@code
 * launcherPrep} in an auto spins up, aims, and (in sim) fires the held Fuel without any
 * auto-specific firing logic here.
 *
 * <p>All paths are authored Blue-side; {@link BLineSwerveAuto#builder} applies
 * {@code withDefaultShouldFlip()} so the same routine mirrors onto Red.
 */
public final class RebuiltAutoRoutines {

  /** Handoff radius for the auto paths — larger than the library default so corners round. */
  private static final double HANDOFF_RADIUS_M = 0.45;

  /** Fraction of max linear speed to cruise at — a touch slower than default for corner margin. */
  private static final double CRUISE_FRACTION = 0.65;

  private final AkitSwerveDrive drive;
  private final RebuiltIntake intake;
  private final RebuiltLauncher launcher;
  private final RebuiltIndexer indexer;

  /** Builder that re-anchors odometry to the path start (used for the first path of an auto). */
  private final FollowPath.Builder reset;
  /** Builder that leaves odometry alone (used for continuation paths, which are linked). */
  private final FollowPath.Builder cont;

  /** Per-path constraints applied to every JSON-loaded path to match the cruise-fraction speed. */
  private final Path.PathConstraints autoPathConstraints;

  public RebuiltAutoRoutines(
      AkitSwerveDrive drive,
      RebuiltIntake intake,
      RebuiltLauncher launcher,
      RebuiltIndexer indexer) {
    this.drive = drive;
    this.intake = intake;
    this.launcher = launcher;
    this.indexer = indexer;
    this.reset = BLineSwerveAuto.builder(drive);
    this.cont = BLineSwerveAuto.builder(drive).withPoseReset(pose -> {});
    // BLineSwerveAuto.builder set the library-default global constraints (0.30 m handoff). The
    // source competition paths are tight, curved, and densely shaped, so override with a bigger
    // handoff + slightly lower cruise speed so BLine rounds the corners instead of weaving through
    // them. These autos are the only BLine consumer in this robot, so a one-time global override is
    // safe (no teleop drive-to-pose binding resets it). Tunable via the constants above.
    this.autoPathConstraints = buildAutoConstraints(drive);
    applyAutoConstraints(drive);
    registerEvents();
  }

  /**
   * Builds per-path constraints matching the global defaults — needed because JSON-loaded paths
   * read config.json at construction time and ignore {@link Path#setDefaultGlobalConstraints}.
   */
  private static Path.PathConstraints buildAutoConstraints(AkitSwerveDrive drive) {
    double maxV = drive.getMaxLinearSpeed().in(edu.wpi.first.units.Units.MetersPerSecond);
    double maxOmegaDeg =
        Math.toDegrees(drive.getMaxAngularSpeed().in(edu.wpi.first.units.Units.RadiansPerSecond));
    return new Path.PathConstraints()
        .setMaxVelocityMetersPerSec(maxV * CRUISE_FRACTION)
        .setMaxAccelerationMetersPerSec2(maxV * 2.0)
        .setMaxVelocityDegPerSec(maxOmegaDeg)
        .setMaxAccelerationDegPerSec2(maxOmegaDeg * 2.0)
        .setEndTranslationToleranceMeters(0.05)
        .setEndRotationToleranceDeg(3.0);
  }

  /** Installs the auto-tuned BLine global constraints (bigger handoff, gentler cruise). */
  private static void applyAutoConstraints(AkitSwerveDrive drive) {
    double maxV = drive.getMaxLinearSpeed().in(edu.wpi.first.units.Units.MetersPerSecond);
    double maxOmegaDeg =
        Math.toDegrees(drive.getMaxAngularSpeed().in(edu.wpi.first.units.Units.RadiansPerSecond));
    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            maxV * CRUISE_FRACTION, maxV * 2.0, maxOmegaDeg, maxOmegaDeg * 2.0,
            0.05, 3.0, HANDOFF_RADIUS_M));
  }

  /**
   * Registers the BLine event-trigger keys used by the ported paths/autos to the matching
   * subsystem command. Mirrors the source {@code NamedCommandsReg} (only the keys the Orbit autos
   * and their paths reference, plus the obvious siblings). Each command is a one-shot state
   * request; the subsystem state machines hold the state afterward.
   */
  private void registerEvents() {
    FollowPath.registerEventTrigger(
        "intakeIntake", intake.intakeCommand(() -> Constants.Intake.INTAKE_IN));
    FollowPath.registerEventTrigger("intakeRetracting", intake.retractCommand());
    FollowPath.registerEventTrigger("intakeRetracted", intake.retractCommand());
    FollowPath.registerEventTrigger("launcherPrep", launcher.prepCommand());
    FollowPath.registerEventTrigger("launcherLow", launcher.lowSpeedCommand());
    FollowPath.registerEventTrigger("launcherIdle", launcher.idleCommand());
    FollowPath.registerEventTrigger("indexerChurn", indexer.churnCommand());
    FollowPath.registerEventTrigger("indexerFeed", indexer.feedCommand());
    FollowPath.registerEventTrigger("indexerIdle", indexer.idleCommand());
  }

  // ── named-command building blocks (scheduled inline in the auto sequences) ──────────────────

  private Command launcherPrep() {
    return launcher.prepCommand();
  }

  private Command launcherLow() {
    return launcher.lowSpeedCommand();
  }

  private Command indexerChurn() {
    return indexer.churnCommand();
  }

  private Command intakeIntake() {
    return intake.intakeCommand(() -> Constants.Intake.INTAKE_IN);
  }

  /** Blocks (≤ 3 s) until the intake reports INTAKING — the source's {@code WaitUntilIntaking}. */
  private Command waitUntilIntaking() {
    return Commands.waitUntil(() -> intake.isCurrent(RebuiltIntake.IntakeState.INTAKING))
        .withTimeout(3.0);
  }

  // ── non-path autos ──────────────────────────────────────────────────────────────────────────

  private Command shootPreload() {
    return Commands.sequence(
            intake.intakeCommand(() -> Constants.Intake.INTAKE_IN),
            Commands.waitUntil(intake::isExtended).withTimeout(2.0),
            launcher.prepCommand(),
            Commands.waitSeconds(3.5),
            launcher.lowSpeedCommand())
        .withName("Auto/ShootPreload");
  }

  private Command driveOutShootPreload() {
    return Commands.sequence(
            shootPreload(),
            Commands.run(
                    () -> drive.runVelocityFieldRelative(new ChassisSpeeds(1.5, 0, 0)), drive)
                .withTimeout(1.5),
            Commands.runOnce(drive::stop, drive))
        .withName("Auto/DriveOutShootPreload");
  }

  // ── auto chooser registration ────────────────────────────────────────────────────────────────

  /** Returns the blue-alliance starting pose for the named path (loaded from JSON). */
  private static Pose2d ps(String pathName) {
    return new Path(pathName).getStartPose();
  }

  /**
   * Returns all auto menu entries in display order, each with a lazy factory and an optional
   * blue-alliance start pose. Pass each entry to
   * {@link org.frc5010.common.profiles.SwerveRobotContainer#addAuto(String,
   * java.util.function.Supplier, Pose2d)}.
   */
  public List<AutoEntry> allAutos() {
    return List.of(
        new AutoEntry("Do Nothing", Commands::none, null),
        new AutoEntry("Shoot Preload", this::shootPreload, null),
        new AutoEntry("Drive Out + Shoot Preload", this::driveOutShootPreload, null),
        new AutoEntry("Orbit: Left",                        this::orbitLeft,             ps("TL-QTRH")),
        new AutoEntry("Orbit: Right",                       this::orbitRight,            ps("TR-CTR-QTR")),
        new AutoEntry("Orbit: Left 1 Swipe",                this::orbitLeft1Swipe,       ps("TL-QTRHLong")),
        new AutoEntry("Orbit: Right 1 Swipe",               this::orbitRight1Swipe,      ps("TR-CTR-QTRLong")),
        new AutoEntry("Orbit: Right 2 Swipe (no HP)",       this::orbitRight2Swipe,      ps("TR-CTR-QTRAngled")),
        new AutoEntry("Orbit: Churn Right 2 Swipe (no HP)", this::churnOrbitRight2Swipe, ps("TR-CTR-QTRAngled")),
        new AutoEntry("Delay Trench Neutral Bump HP", this::delayTrenchNeutralBumpHP, ps("StartTR-CTR-HLF-BR-HP")),
        new AutoEntry("Disrupt",                      this::disrupt,                  ps("RightDisrupt1")),
        new AutoEntry("Follow: Left Bump Depot",  this::followLeftBumpDepot,  ps("TLBack-CTL-QTL-BL-L")),
        new AutoEntry("Follow: Left Trench",      this::followLeftTrench,      ps("TLback-CTL-QTL")),
        new AutoEntry("Follow: Right Bump HP",    this::followRightBumpHP,    ps("DelayTRS-CTR-QTR-BR-HP Longer")),
        new AutoEntry("Follow: Right Trench HP",  this::followRightTrenchHP,  ps("DelayTR-QTRL")),
        new AutoEntry("Left: 2056 Double HP", this::left2056DoubleHP, ps("TL-CTR-QTL-BL-TL")),
        new AutoEntry("Left: 5010 Double",    this::left5010Double,   ps("TL-CTL-QTL")),
        new AutoEntry("Left: 3 Shuttle HPC",  this::left3ShuttleHPC,  ps("TL-QTLCTLSHORT")),
        new AutoEntry("Quals 110", this::quals110, ps("TLback-CTL-QTL")),
        new AutoEntry("Quals 73",  this::quals73,  ps("DelayTR-QTRL")),
        new AutoEntry("Right: 2056 Double HP",          this::right2056DoubleHP,          ps("TR-CTR-QTR-BR-TR")),
        new AutoEntry("Right: 5010 Double",             this::right5010Double,            ps("TRSide-CTR-QTR")),
        new AutoEntry("Right: 5010 Double (Old)",       this::right5010DoubleOld,         ps("TRSide-CTR-QTR")),
        new AutoEntry("Right: 5010 Double (Optimized)", this::right5010DoubleOptimized,   ps("TRSide-CTR-QTR")),
        new AutoEntry("Right: 5010 Double (Short)",     this::right5010DoubleShort,       ps("TR-CTR-QTRShort")),
        new AutoEntry("Right: 3 Shuttle HPC",           this::right3ShuttleHPC,           ps("TR-QTRCTRSHORT")));
  }

  /** Follows a native BLine path; {@code first} re-anchors odometry to the path start. */
  private Command path(String name, boolean first) {
    Path p = new Path(name);
    p.setPathConstraints(autoPathConstraints);
    return (first ? reset : cont).build(p);
  }

  // ── Orbit autos (verbatim sequences from the source .auto files) ────────────────────────────

  /** Orbit Left: TL-QTRH → low → QTRLong-TL → (prep ∥ wait 3) → low → TL-CTR-QTR → QTRL-MID → prep → low. */
  public Command orbitLeft() {
    return Commands.sequence(
            path("TL-QTRH", true),
            launcherLow(),
            path("QTRLong-TL", false),
            Commands.parallel(launcherPrep(), Commands.waitSeconds(3.0)),
            launcherLow(),
            path("TL-CTR-QTR", false),
            path("QTRL-MID", false),
            launcherPrep(),
            launcherLow())
        .withName("Auto/OrbitLeft");
  }

  /** Orbit Right: TR-CTR-QTR → QTR-TR → prep → wait 3 → low → TR-CTR-HALF → QTRH-HP → prep. */
  public Command orbitRight() {
    return Commands.sequence(
            path("TR-CTR-QTR", true),
            path("QTR-TR", false),
            launcherPrep(),
            Commands.waitSeconds(3.0),
            launcherLow(),
            path("TR-CTR-HALF", false),
            path("QTRH-HP", false),
            launcherPrep())
        .withName("Auto/OrbitRight");
  }

  /** Orbit Left 1 Swipe: TL-QTRHLong → low → QTRLong-TL → prep. */
  public Command orbitLeft1Swipe() {
    return Commands.sequence(
            path("TL-QTRHLong", true),
            launcherLow(),
            path("QTRLong-TL", false),
            launcherPrep())
        .withName("Auto/OrbitLeft1Swipe");
  }

  /** Orbit Right 1 Swipe: TR-CTR-QTRLong → low → QTRLong-HP → prep. */
  public Command orbitRight1Swipe() {
    return Commands.sequence(
            path("TR-CTR-QTRLong", true),
            launcherLow(),
            path("QTRLong-HP", false),
            launcherPrep())
        .withName("Auto/OrbitRight1Swipe");
  }

  /**
   * Orbit Right 2 Swipe (no HP): TR-CTR-QTRAngled → QTR-TRBack → prep → wait 3 → low →
   * TRBack-CTR-HALF → QTRH-TRBack → prep → wait 2.5 → low → TRBack-CTR-HALF.
   */
  public Command orbitRight2Swipe() {
    return Commands.sequence(
            path("TR-CTR-QTRAngled", true),
            path("QTR-TRBack", false),
            launcherPrep(),
            Commands.waitSeconds(3.0),
            launcherLow(),
            path("TRBack-CTR-HALF", false),
            path("QTRH-TRBack", false),
            launcherPrep(),
            Commands.waitSeconds(2.5),
            launcherLow(),
            path("TRBack-CTR-HALF", false))
        .withName("Auto/OrbitRight2Swipe");
  }

  /**
   * Churn-Orbit Right 2 Swipe (no HP): TR-CTR-QTRAngled → QTR-TRBack → churn → prep → wait 4 →
   * low → TRBack-CTR-HALF → QTRH-TRBack → prep.
   */
  public Command churnOrbitRight2Swipe() {
    return Commands.sequence(
            path("TR-CTR-QTRAngled", true),
            path("QTR-TRBack", false),
            indexerChurn(),
            launcherPrep(),
            Commands.waitSeconds(4.0),
            launcherLow(),
            path("TRBack-CTR-HALF", false),
            path("QTRH-TRBack", false),
            launcherPrep())
        .withName("Auto/ChurnOrbitRight2Swipe");
  }

  // ── Delay Trench / Disrupt ──────────────────────────────────────────────────────────────────

  /** Delay Trench Neutral Bump HP: wait 3 → StartTR-CTR-HLF-BR-HP (collect via the path's markers). */
  public Command delayTrenchNeutralBumpHP() {
    return Commands.sequence(Commands.waitSeconds(3.0), path("StartTR-CTR-HLF-BR-HP", true))
        .withName("Auto/DelayTrenchNeutralBumpHP");
  }

  /** Disrupt: RightDisrupt1 → intakeIntake → waitUntilIntaking → Disrupter2 → prep. */
  public Command disrupt() {
    return Commands.sequence(
            path("RightDisrupt1", true),
            intakeIntake(),
            waitUntilIntaking(),
            path("Disrupter2", false),
            launcherPrep())
        .withName("Auto/Disrupt");
  }

  // ── Follow ──────────────────────────────────────────────────────────────────────────────────

  /** Follow Left Bump Depot: wait 1 → intake → prep → wait 3 → low → TLBack-CTL-QTL-BL-L → prep. */
  public Command followLeftBumpDepot() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            intakeIntake(),
            launcherPrep(),
            Commands.waitSeconds(3.0),
            launcherLow(),
            path("TLBack-CTL-QTL-BL-L", true),
            launcherPrep())
        .withName("Auto/FollowLeftBumpDepot");
  }

  /** Follow Left Trench: wait 1 → intake → prep → wait 4 → low → TLback-CTL-QTL → QTLong-TL → prep. */
  public Command followLeftTrench() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            intakeIntake(),
            launcherPrep(),
            Commands.waitSeconds(4.0),
            launcherLow(),
            path("TLback-CTL-QTL", true),
            path("QTLong-TL", false),
            launcherPrep())
        .withName("Auto/FollowLeftTrench");
  }

  /** Follow Right Bump HP: wait 1 → intake → prep → wait 4 → low → DelayTRS-CTR-QTR-BR-HP Longer → prep. */
  public Command followRightBumpHP() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            intakeIntake(),
            launcherPrep(),
            Commands.waitSeconds(4.0),
            launcherLow(),
            path("DelayTRS-CTR-QTR-BR-HP Longer", true),
            launcherPrep())
        .withName("Auto/FollowRightBumpHP");
  }

  /** Follow Right Trench HP: wait 1 → intake → prep → wait 2 → low → DelayTR-QTRL → QTRLeft-HP → prep. */
  public Command followRightTrenchHP() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            intakeIntake(),
            launcherPrep(),
            Commands.waitSeconds(2.0),
            launcherLow(),
            path("DelayTR-QTRL", true),
            path("QTRLeft-HP", false),
            launcherPrep())
        .withName("Auto/FollowRightTrenchHP");
  }

  // ── Left ────────────────────────────────────────────────────────────────────────────────────

  /** Left 2056 Double HP: TL-CTR-QTL-BL-TL → TL-CTR-HLF-BL-HP (collect/shoot via path markers). */
  public Command left2056DoubleHP() {
    return Commands.sequence(
            path("TL-CTR-QTL-BL-TL", true), path("TL-CTR-HLF-BL-HP", false))
        .withName("Auto/Left2056DoubleHP");
  }

  /** Left 5010 Double: TL-CTL-QTL → QTL-TL → prep → wait 3 → low → TL-CTL-QTL-BL-L → prep. */
  public Command left5010Double() {
    return Commands.sequence(
            path("TL-CTL-QTL", true),
            path("QTL-TL", false),
            launcherPrep(),
            Commands.waitSeconds(3.0),
            launcherLow(),
            path("TL-CTL-QTL-BL-L", false),
            launcherPrep())
        .withName("Auto/Left5010Double");
  }

  /** Left 3 Shuttle HPC: TL-QTLCTLSHORT → CTL-NWALL → Left3rdSwipe → LeftNwall-Tower. */
  public Command left3ShuttleHPC() {
    return Commands.sequence(
            path("TL-QTLCTLSHORT", true),
            path("CTL-NWALL", false),
            path("Left3rdSwipe", false),
            path("LeftNwall-Tower", false))
        .withName("Auto/Left3ShuttleHPC");
  }

  // ── Quals ───────────────────────────────────────────────────────────────────────────────────

  /** Quals 110: wait 1 → intake → prep → wait 2 → low → TLback-CTL-QTL → QTLong-TLHP → prep. */
  public Command quals110() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            intakeIntake(),
            launcherPrep(),
            Commands.waitSeconds(2.0),
            launcherLow(),
            path("TLback-CTL-QTL", true),
            path("QTLong-TLHP", false),
            launcherPrep())
        .withName("Auto/Quals110");
  }

  /** Quals 73: wait 1 → intake → prep → wait 3 → low → DelayTR-QTRL → QBRight-HP → prep. */
  public Command quals73() {
    return Commands.sequence(
            Commands.waitSeconds(1.0),
            intakeIntake(),
            launcherPrep(),
            Commands.waitSeconds(3.0),
            launcherLow(),
            path("DelayTR-QTRL", true),
            path("QBRight-HP", false),
            launcherPrep())
        .withName("Auto/Quals73");
  }

  // ── Right ───────────────────────────────────────────────────────────────────────────────────

  /** Right 2056 Double HP: TR-CTR-QTR-BR-TR → TR-CTR-HLF-BR-HP (collect/shoot via path markers). */
  public Command right2056DoubleHP() {
    return Commands.sequence(
            path("TR-CTR-QTR-BR-TR", true), path("TR-CTR-HLF-BR-HP", false))
        .withName("Auto/Right2056DoubleHP");
  }

  /** Right 5010 Double: TRSide-CTR-QTR → QTR-TR → prep → wait 3 → low → TR-CTR-QTR-BR-HP Longer → prep. */
  public Command right5010Double() {
    return Commands.sequence(
            path("TRSide-CTR-QTR", true),
            path("QTR-TR", false),
            launcherPrep(),
            Commands.waitSeconds(3.0),
            launcherLow(),
            path("TR-CTR-QTR-BR-HP Longer", false),
            launcherPrep())
        .withName("Auto/Right5010Double");
  }

  /** Right 5010 Double (Old): TRSide-CTR-QTR → QTR-TR → prep → wait 3.5 → low → TR-CTR-QTR-BR-HP → prep. */
  public Command right5010DoubleOld() {
    return Commands.sequence(
            path("TRSide-CTR-QTR", true),
            path("QTR-TR", false),
            launcherPrep(),
            Commands.waitSeconds(3.5),
            launcherLow(),
            path("TR-CTR-QTR-BR-HP", false),
            launcherPrep())
        .withName("Auto/Right5010DoubleOld");
  }

  /** Right 5010 Double (Optimized): TRSide-CTR-QTR → QTR-TR → prep → wait 3 → low → TR-CTR-QTR-BR-HP Longer → prep. */
  public Command right5010DoubleOptimized() {
    return Commands.sequence(
            path("TRSide-CTR-QTR", true),
            path("QTR-TR", false),
            launcherPrep(),
            Commands.waitSeconds(3.0),
            launcherLow(),
            path("TR-CTR-QTR-BR-HP Longer", false),
            launcherPrep())
        .withName("Auto/Right5010DoubleOptimized");
  }

  /**
   * Right 5010 Double (Short): deploy intake immediately, TR-CTR-QTRShort → QTRShort-TR → prep
   * → wait 3 → low → TR-CTR-QTR-BR-HP → prep.
   *
   * <p>The inline {@code intakeCommand} at the top guarantees the hopper deploys at auto start.
   * The path-embedded {@code intakeIntake} event (at t_ratio=0.232 in TR-CTR-QTRShort) fires
   * as a second request once the robot is moving — both are harmless when issued together.
   */
  public Command right5010DoubleShort() {
    return Commands.sequence(
            intake.intakeCommand(() -> Constants.Intake.INTAKE_IN),
            path("TR-CTR-QTRShort", true),
            path("QTRShort-TR", false),
            launcherPrep(),
            Commands.waitSeconds(3.0),
            launcherLow(),
            path("TR-CTR-QTR-BR-HP", false),
            launcherPrep())
        .withName("Auto/Right5010DoubleShort");
  }

  /** Right 3 Shuttle HPC: TR-QTRCTRSHORT → CTR-NWALL → 3RD-SWIPE → NWALL-CLIMB. */
  public Command right3ShuttleHPC() {
    return Commands.sequence(
            path("TR-QTRCTRSHORT", true),
            path("CTR-NWALL", false),
            path("3RD-SWIPE", false),
            path("NWALL-CLIMB", false))
        .withName("Auto/Right3ShuttleHPC");
  }
}
