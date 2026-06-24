package frc.robot.rebuilt;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.Waypoint;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.auto.BLineSwerveAuto;
import org.frc5010.common.drive.swerve.auto.PathPlannerToBLine;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Layer 3 — proves the ported Orbit autos work end to end under IronMaple physics: a converted
 * PathPlanner Orbit path ({@link PathPlannerToBLine}) drives the physics-backed
 * {@link AkitSwerveDrive} to the path's end, and the path's embedded event markers fire their
 * registered BLine event triggers along the way.
 *
 * <p>Uses the same harness/teardown as {@code BLineFollowPathSimPhysicsTest}. {@code QTR-TR} is a
 * ~5 m curved Orbit path carrying a {@code launcherLow} event marker (relPos ≈ 0.64) and a
 * rotation target — exercising the full converter output (sampled polyline + rotation + event).
 */
class OrbitAutoSimPhysicsTest extends SimTestBase {

  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  private static final Pose2d SPAWN = new Pose2d(2.0, 2.0, edu.wpi.first.math.geometry.Rotation2d.kZero);

  private AkitSwerveDrive drive;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    drive = SwerveFactory.build(CONSTANTS, SPAWN);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    SimulatedArena.getInstance().shutDown();
    try {
      java.lang.reflect.Field f = SimulatedArena.class.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("Failed to reset SimulatedArena singleton between tests", e);
    }
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  private void step() {
    drive.simulationPeriodic();
    drive.periodic();
    stepOneCycle();
  }

  /** Schedules {@code cmd}, pumps until it finishes (or the budget runs out), then coasts. */
  private int runToCompletion(Command cmd, int maxCycles) {
    enableTeleop();
    CommandScheduler.getInstance().schedule(cmd);
    int cyclesRun = 0;
    while (cyclesRun < maxCycles && cmd.isScheduled()) {
      CommandScheduler.getInstance().run();
      step();
      cyclesRun++;
    }
    for (int i = 0; i < 25; i++) step(); // coast: let module braking settle the body
    return cyclesRun;
  }

  /** The translation of a converted path's final waypoint — its geometric end point. */
  private static Translation2d endOf(Path path) {
    List<PathElement> elements = path.getPathElements();
    for (int i = elements.size() - 1; i >= 0; i--) {
      if (elements.get(i) instanceof Waypoint w) {
        return w.translationTarget().translation();
      }
    }
    throw new IllegalStateException("converted path has no waypoint");
  }

  @Test
  void physicsFollowsConvertedOrbitPathToItsEnd() {
    Path path = PathPlannerToBLine.load("QTR-TR");
    Translation2d goal = endOf(path);

    Command follow = BLineSwerveAuto.builder(drive).build(path);
    // QTR-TR is ~5 m of curved travel; 500 cycles = 10 simulated seconds is generous.
    int cycles = runToCompletion(follow, 500);

    Pose2d finalPose = drive.getPose();
    double distToGoal = finalPose.getTranslation().getDistance(goal);
    assertTrue(cycles < 500, "FollowPath did not finish within 10 simulated seconds");
    assertTrue(
        distToGoal < 0.50,
        "robot should reach the converted Orbit path end " + goal + "; got " + finalPose
            + " distToGoal=" + distToGoal);
  }

  /**
   * Regression guard for the "erratic paths" fix: the source competition paths are tight and
   * curved, and dense Bézier sampling made BLine thread every vertex and badly overshoot/loop
   * (a ~5 m path traveled ~22 m, ratio ≈ 4×). {@code RebuiltAutoRoutines} fixes this with sparse
   * sampling (4/segment) + a larger handoff radius (0.45 m). Here we follow with those settings and
   * assert the robot does not loop — total travel stays a small multiple of the path's span.
   */
  @Test
  void tunedFollowingDoesNotLoop() {
    Path path = PathPlannerToBLine.load("TRSide-CTR-QTR", 4);
    Translation2d goal = endOf(path);

    // Match RebuiltAutoRoutines' tuned global constraints (bigger handoff than the library default).
    double maxV = drive.getMaxLinearSpeed().in(edu.wpi.first.units.Units.MetersPerSecond);
    double maxOmegaDeg =
        Math.toDegrees(drive.getMaxAngularSpeed().in(edu.wpi.first.units.Units.RadiansPerSecond));
    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(maxV * 0.65, maxV * 2.0, maxOmegaDeg, maxOmegaDeg * 2.0, 0.05, 3.0, 0.45));
    Command follow = BLineSwerveAuto.builder(drive).build(path);

    // Bounding-box diagonal of the path = a lower bound on its span.
    double minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
    for (PathElement e : path.getPathElements()) {
      Translation2d t = e instanceof Waypoint w ? w.translationTarget().translation()
          : e instanceof Path.TranslationTarget tt ? tt.translation() : null;
      if (t == null) continue;
      minx = Math.min(minx, t.getX()); maxx = Math.max(maxx, t.getX());
      miny = Math.min(miny, t.getY()); maxy = Math.max(maxy, t.getY());
    }
    double diag = Math.hypot(maxx - minx, maxy - miny);

    enableTeleop();
    CommandScheduler.getInstance().schedule(follow);
    double traveled = 0.0;
    Pose2d prev = null;
    int c = 0;
    while (c < 500 && follow.isScheduled()) {
      CommandScheduler.getInstance().run();
      step();
      Pose2d p = drive.getPose();
      if (prev != null) traveled += p.getTranslation().getDistance(prev.getTranslation());
      prev = p;
      c++;
    }
    for (int i = 0; i < 25; i++) step();

    double ratio = traveled / diag;
    assertTrue(
        drive.getPose().getTranslation().getDistance(goal) < 0.5,
        "tuned follow must still reach the path end");
    assertTrue(
        ratio < 3.0,
        "tuned follow must not loop: traveled " + traveled + " m over a " + diag
            + " m span (ratio " + ratio + "×, dense-sampling regression is ~4×)");
  }

  @Test
  void embeddedEventMarkerFiresDuringFollow() {
    // QTR-TR carries a `launcherLow` event marker partway along it. Register a counter for that
    // key (the BLine analogue of NamedCommands) and confirm BLine fires it while following.
    AtomicInteger fired = new AtomicInteger(0);
    FollowPath.registerEventTrigger("launcherLow", fired::incrementAndGet);

    Path path = PathPlannerToBLine.load("QTR-TR");
    Command follow = BLineSwerveAuto.builder(drive).build(path);
    runToCompletion(follow, 500);

    assertTrue(
        fired.get() > 0,
        "the path's embedded launcherLow event marker must fire its registered trigger");
  }
}
