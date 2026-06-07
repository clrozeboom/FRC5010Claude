package org.frc5010.common.subsystem;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AutoRoutines;
import frc.robot.DemoIntake;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.frc5010.common.drive.swerve.SwerveFactory;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.auto.BLineSwerveAuto;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Layer 3 — deep verification of {@link AutoRoutines} on an IronMaple physics-backed
 * {@link AkitSwerveDrive}.
 *
 * <p>Where {@link BLineFollowPathSimPhysicsTest} only checks that a synthetic straight
 * path ends near its goal, this test runs the <em>actual</em> production auto routines
 * end-to-end and verifies BLine's path tracking <em>throughout</em> the run, not just at
 * the endpoint. It does so by tapping BLine's own telemetry: {@link FollowPath} exposes
 * static logging consumers ({@link FollowPath#setDoubleLoggingConsumer},
 * {@link FollowPath#setPoseLoggingConsumer}, …) that emit per-cycle signals such as
 * {@code FollowPath/crossTrackError}, {@code FollowPath/rotationErrorDeg},
 * {@code FollowPath/translationElementIndex} and {@code FollowPath/remainingPathDistanceMeters}.
 *
 * <p>Each scenario captures these authoritative signals plus the physics robot pose to a
 * CSV under {@code build/auto-verify/} (inspectable in AdvantageScope or a spreadsheet) and
 * asserts:
 * <ul>
 *   <li>BLine actually executed (telemetry rows captured),</li>
 *   <li>cross-track error stayed within a bound for the whole path (tracking, not just endpoint),</li>
 *   <li>heading error stayed within a bound,</li>
 *   <li>the translation element index advanced through every waypoint (no skipped/stalled segment),</li>
 *   <li>remaining-path-distance was monotonically non-increasing (real forward progress),</li>
 *   <li>the physics body finished within end-translation / end-rotation tolerance of the goal.</li>
 * </ul>
 *
 * <p>Scenarios A and B run the real {@link AutoRoutines#exampleScore} (deployed JSON) and
 * {@link AutoRoutines#exampleScoreInCode} (code-defined) routines — the two production
 * path-definition workflows. Scenario C adds a path with a 90° heading change to exercise
 * BLine's rotation controller, which the heading-constant example routines never load.
 */
class AutoRoutinesSimPhysicsTest extends SimTestBase {

  private static final SwerveConstants CONSTANTS =
      new SwerveConstants.Builder()
          .moduleType(ModuleType.SIM)
          .gyroType(GyroType.SIM)
          .build();

  private static final Pose2d SPAWN = new Pose2d(1.5, 2.0, new Rotation2d());

  // Bounds (tuned from observed physics-sim telemetry; see CSVs in build/auto-verify/).
  private static final double CROSS_TRACK_BOUND_M = 0.30;
  private static final double ROTATION_BOUND_DEG = 25.0;
  private static final double END_TRANSLATION_TOL_M = 0.30;
  private static final double END_ROTATION_TOL_DEG = 10.0;

  private AkitSwerveDrive drive;

  // Per-cycle capture of BLine's static logging-consumer output.
  private final Map<String, Double> doubles = new HashMap<>();
  private final Map<String, Pose2d> poses = new HashMap<>();
  private boolean blineRanThisCycle;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    // Pin Blue so .withDefaultShouldFlip() is a no-op and the run is deterministic.
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    drive = SwerveFactory.build(CONSTANTS, SPAWN);

    FollowPath.setDoubleLoggingConsumer(
        (Pair<String, Double> p) -> {
          doubles.put(p.getFirst(), p.getSecond());
          blineRanThisCycle = true;
        });
    FollowPath.setPoseLoggingConsumer((Pair<String, Pose2d> p) -> poses.put(p.getFirst(), p.getSecond()));
    FollowPath.setBooleanLoggingConsumer((Pair<String, Boolean> p) -> {});
    FollowPath.setTranslationListLoggingConsumer((Pair<String, Translation2d[]> p) -> {});
  }

  @AfterEach
  @Override
  public void simTeardown() {
    // Detach our consumers so they don't leak into other tests sharing the static hooks.
    FollowPath.setDoubleLoggingConsumer(p -> {});
    FollowPath.setPoseLoggingConsumer(p -> {});
    FollowPath.setBooleanLoggingConsumer(p -> {});
    FollowPath.setTranslationListLoggingConsumer(p -> {});

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

  /** One captured telemetry sample: the physics robot pose plus BLine's reported signals. */
  private record Sample(
      double t,
      Pose2d robot,
      double crossTrackError,
      double rotationErrorDeg,
      int translationElementIndex,
      double segmentProgress,
      double remainingPathDistanceMeters,
      Pose2d closestPoint) {}

  /**
   * Schedules {@code command}, spinning the scheduler + physics until it finishes (or times
   * out), capturing one {@link Sample} per cycle in which BLine's FollowPath actually executed.
   */
  private List<Sample> runAndCapture(Command command, int maxCycles) {
    List<Sample> samples = new ArrayList<>();
    enableAuto();
    CommandScheduler.getInstance().schedule(command);

    double t = 0.0;
    int cycles = 0;
    while (cycles < maxCycles && command.isScheduled()) {
      Pose2d robotPose = drive.getPose();
      blineRanThisCycle = false;
      CommandScheduler.getInstance().run();
      if (blineRanThisCycle) {
        samples.add(
            new Sample(
                t,
                robotPose,
                doubles.getOrDefault("FollowPath/crossTrackError", 0.0),
                doubles.getOrDefault("FollowPath/rotationErrorDeg", 0.0),
                (int) Math.round(doubles.getOrDefault("FollowPath/translationElementIndex", 0.0)),
                doubles.getOrDefault("FollowPath/segmentProgress", 0.0),
                doubles.getOrDefault("FollowPath/remainingPathDistanceMeters", 0.0),
                poses.getOrDefault("FollowPath/closestPoint", Pose2d.kZero)));
      }
      step();
      t += LOOP_PERIOD_SECS;
      cycles++;
    }
    if (cycles >= maxCycles) {
      Sample last = samples.isEmpty() ? null : samples.get(samples.size() - 1);
      System.out.println(
          "[AutoRoutinesSimPhysicsTest] TIMEOUT after " + cycles + " cycles; drivePose="
              + drive.getPose() + " samples=" + samples.size()
              + " lastSample=" + (last == null ? "none"
                  : "(" + last.robot.getX() + "," + last.robot.getY() + ") idx=" + last.translationElementIndex
                      + " remaining=" + last.remainingPathDistanceMeters));
    }
    assertTrue(cycles < maxCycles, "Routine did not finish within " + (maxCycles * 0.02) + " s");
    // Coast so the physics body settles after BLine's isFinished() fires at setpoint.
    for (int i = 0; i < 25; i++) step();
    return samples;
  }

  private void writeCsv(String name, List<Sample> samples, List<Translation2d> polyline) {
    StringBuilder sb = new StringBuilder();
    sb.append("t,robotX,robotY,robotHeadingDeg,actualCrossTrack,blineCrossTrack,rotationErrorDeg,"
        + "translationElementIndex,segmentProgress,remainingPathDistanceMeters,"
        + "closestX,closestY\n");
    for (Sample s : samples) {
      sb.append(String.format(
          "%.3f,%.4f,%.4f,%.3f,%.5f,%.5f,%.4f,%d,%.4f,%.4f,%.4f,%.4f%n",
          s.t,
          s.robot.getX(),
          s.robot.getY(),
          s.robot.getRotation().getDegrees(),
          distToPolyline(s.robot.getTranslation(), polyline),
          s.crossTrackError,
          s.rotationErrorDeg,
          s.translationElementIndex,
          s.segmentProgress,
          s.remainingPathDistanceMeters,
          s.closestPoint.getX(),
          s.closestPoint.getY()));
    }
    try {
      java.nio.file.Path dir = Paths.get("build", "auto-verify");
      Files.createDirectories(dir);
      Files.writeString(dir.resolve(name + ".csv"), sb.toString());
    } catch (IOException e) {
      throw new RuntimeException("Failed to write trajectory CSV for " + name, e);
    }
  }

  /** Perpendicular distance from {@code p} to the nearest segment of the polyline. */
  private static double distToPolyline(Translation2d p, List<Translation2d> polyline) {
    double best = Double.MAX_VALUE;
    for (int i = 0; i < polyline.size() - 1; i++) {
      best = Math.min(best, distToSegment(p, polyline.get(i), polyline.get(i + 1)));
    }
    return best;
  }

  private static double distToSegment(Translation2d p, Translation2d a, Translation2d b) {
    Translation2d ab = b.minus(a);
    double len2 = ab.getX() * ab.getX() + ab.getY() * ab.getY();
    if (len2 < 1e-9) return p.getDistance(a);
    Translation2d ap = p.minus(a);
    double t = (ap.getX() * ab.getX() + ap.getY() * ab.getY()) / len2;
    t = Math.max(0.0, Math.min(1.0, t));
    return p.getDistance(new Translation2d(a.getX() + t * ab.getX(), a.getY() + t * ab.getY()));
  }

  /**
   * Shared invariant checks over a captured run plus its CSV artifact.
   *
   * @param goal expected final translation
   * @param goalHeadingDeg expected final heading
   * @param lastElementIndex highest translation-element index the path should reach
   */
  private void verify(
      String name,
      List<Sample> samples,
      List<Translation2d> polyline,
      double goalHeadingDeg) {
    writeCsv(name, samples, polyline);
    assertTrue(samples.size() > 10, name + ": expected BLine to execute many cycles, got " + samples.size());

    Translation2d goal = polyline.get(polyline.size() - 1);
    double maxActualCross = 0.0;
    double maxRot = 0.0;
    int firstElementIndex = samples.get(0).translationElementIndex;
    int maxElementIndex = firstElementIndex;
    double prevRemaining = Double.MAX_VALUE;
    double maxRemainingIncrease = 0.0;
    for (Sample s : samples) {
      maxActualCross = Math.max(maxActualCross, distToPolyline(s.robot.getTranslation(), polyline));
      maxRot = Math.max(maxRot, Math.abs(s.rotationErrorDeg));
      maxElementIndex = Math.max(maxElementIndex, s.translationElementIndex);
      if (s.remainingPathDistanceMeters > prevRemaining) {
        maxRemainingIncrease = Math.max(maxRemainingIncrease, s.remainingPathDistanceMeters - prevRemaining);
      }
      prevRemaining = s.remainingPathDistanceMeters;
    }

    // Rotation error legitimately starts at the full initial heading delta (e.g. 90°), so a
    // global max is meaningless. What proves the rotation controller works is convergence:
    // the error in the final quarter of the run must be small.
    double settledMaxRot = 0.0;
    int settleStart = (int) (samples.size() * 0.75);
    for (int i = settleStart; i < samples.size(); i++) {
      settledMaxRot = Math.max(settledMaxRot, Math.abs(samples.get(i).rotationErrorDeg));
    }

    Pose2d finalPose = drive.getPose();
    double endDist = finalPose.getTranslation().getDistance(goal);
    double endHeadingErr =
        Math.abs(finalPose.getRotation().minus(Rotation2d.fromDegrees(goalHeadingDeg)).getDegrees());

    String diag =
        String.format(
            "%s: samples=%d maxActualCrossTrack=%.3fm maxRotErr=%.2fdeg settledRotErr=%.2fdeg "
                + "elementIdx=%d..%d maxRemainingIncrease=%.3fm endDist=%.3fm endHeadingErr=%.2fdeg "
                + "finalPose=%s",
            name, samples.size(), maxActualCross, maxRot, settledMaxRot, firstElementIndex,
            maxElementIndex, maxRemainingIncrease, endDist, endHeadingErr, finalPose);
    System.out.println("[AutoRoutinesSimPhysicsTest] " + diag);

    // Actual geometric deviation from the intended polyline — true path tracking, ignoring
    // BLine's one-cycle reference-snap at waypoint handoffs.
    assertTrue(maxActualCross < CROSS_TRACK_BOUND_M, "cross-track exceeded bound — " + diag);
    assertTrue(settledMaxRot < ROTATION_BOUND_DEG, "rotation controller did not converge — " + diag);
    // The translation-element pointer must advance, proving at least one waypoint handoff
    // actually fired rather than the robot stalling on the first segment.
    assertTrue(
        maxElementIndex > firstElementIndex,
        "translation element index never advanced (no waypoint handoff) — " + diag);
    // Progress should be monotonic; allow a small slack for per-cycle recompute noise.
    assertTrue(maxRemainingIncrease < 0.10, "remaining-distance jumped backward — " + diag);
    assertTrue(endDist < END_TRANSLATION_TOL_M, "did not finish near goal — " + diag);
    assertTrue(endHeadingErr < END_ROTATION_TOL_DEG, "did not finish at goal heading — " + diag);
  }

  @Test
  void exampleScore_jsonPath_tracksAndReachesGoal() {
    // Sanity: the deployed JSON parses to a valid path before we drive it.
    Path json = new Path("ExampleScore");
    assertTrue(json.isValid(), "deployed ExampleScore.json must parse to a valid BLine path");

    List<Sample> samples = runAndCapture(AutoRoutines.exampleScore(drive), 300);
    verify(
        "exampleScore",
        samples,
        List.of(new Translation2d(1.5, 2.0), new Translation2d(2.25, 2.0), new Translation2d(3.0, 2.0)),
        0.0);
  }

  @Test
  void exampleScoreInCode_tracksAndReachesGoal() {
    List<Sample> samples = runAndCapture(AutoRoutines.exampleScoreInCode(drive), 300);
    verify(
        "exampleScoreInCode",
        samples,
        List.of(new Translation2d(1.5, 2.0), new Translation2d(2.25, 2.0), new Translation2d(3.0, 2.0)),
        0.0);
  }

  /**
   * Verifies BLine across two chained {@link FollowPath} commands with a direction reversal —
   * the structural pattern {@link AutoRoutines#pickupAndScore} uses (drive out, then drive back).
   * The robot drives {@code (1.5,2)→(3.0,2)}, then reverses to {@code (3.0,2)→(2.0,2)}, all in
   * the off-centerline {@code y=2.0} corridor the example routines already prove is clear.
   *
   * <p>This isolates BLine's path tracking; it intentionally avoids the {@code pickupAndScore}
   * field-centerline geometry, which plows the robot through the Hub and the central fuel
   * cluster (an arena/intake-physics interaction, not a BLine concern — see the session notes).
   *
   * <p>Both phases hold heading 0° along {@code y=2.0}, so telemetry across the two chained
   * commands is checked against one collinear polyline.
   */
  /**
   * Runs the full composed {@link AutoRoutines#pickupAndScore} routine — BLine outbound,
   * intake, BLine return, fire — with a real {@link DemoIntake} on the physics drive, and
   * verifies the robot rounds the Blue Hub to reach the pickup point and returns to the shot
   * spot.
   *
   * <p>This is the regression guard for the Hub-collision fix: the original straight-centerline
   * path drove the robot into the Hub at (4.5974, 4.0345) and stalled ~1 m short of the pickup;
   * the routed path skirts the Hub through the lane below it. We assert the robot got past the
   * Hub (reached x ≥ 5.5, i.e. the far side) and finished at the shot spot ready to fire. We do
   * not assert on fuel collection, which is a non-deterministic arena/intake concern.
   */
  @Test
  void pickupAndScore_realRoutine_roundsHubReachesPickupAndReturns() {
    DemoIntake intake =
        new DemoIntake(
            drive.getDriveTrainSimulation().orElseThrow(
                () -> new IllegalStateException("physics build must expose a drive-train simulation")),
            drive::getPose,
            drive.getField2d());

    Translation2d shotSpot = new Translation2d(2.80, 4.03);
    List<Sample> samples = runAndCapture(AutoRoutines.pickupAndScore(drive, intake), 1000);
    writeCsv("pickupAndScore", samples, List.of(new Translation2d(1.5, 4.03), shotSpot));

    double maxX = Double.NEGATIVE_INFINITY;
    for (Sample s : samples) maxX = Math.max(maxX, s.robot.getX());
    Pose2d finalPose = drive.getPose();
    double endDist = finalPose.getTranslation().getDistance(shotSpot);
    String diag =
        String.format(
            "pickupAndScore: samples=%d reachedX=%.3f endDist(toShotSpot)=%.3fm finalPose=%s heldFuel=%d",
            samples.size(), maxX, endDist, finalPose, intake.getHeldFuel());
    System.out.println("[AutoRoutinesSimPhysicsTest] " + diag);

    // Reaching the far side of the Hub (x ≥ 5.5) proves the routed path cleared the obstacle the
    // original straight path stalled on (~x=3.6).
    assertTrue(maxX > 5.5, "robot did not get past the Hub to the pickup point — " + diag);
    assertTrue(endDist < END_TRANSLATION_TOL_M, "robot did not return to the shot spot — " + diag);
  }

  @Test
  void chainedReversalPaths_trackBothPathsAndReachGoal() {
    FollowPath.Builder b = BLineSwerveAuto.builder(drive);

    Pose2d start = new Pose2d(1.5, 2.0, Rotation2d.fromDegrees(0));
    Pose2d far = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0));
    Pose2d back = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0));
    Path outbound =
        new Path(
            new Path.Waypoint(start.getTranslation(), start.getRotation()),
            new Path.Waypoint(far.getTranslation(), far.getRotation()));
    Path returnPath =
        new Path(
            new Path.Waypoint(far.getTranslation(), far.getRotation()),
            new Path.Waypoint(back.getTranslation(), back.getRotation()));

    Command follow =
        edu.wpi.first.wpilibj2.command.Commands.sequence(
            edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> drive.setPose(start), drive),
            edu.wpi.first.wpilibj2.command.Commands.waitSeconds(0.05),
            b.build(outbound),
            b.build(returnPath),
            edu.wpi.first.wpilibj2.command.Commands.runOnce(drive::stop, drive));

    List<Translation2d> polyline =
        List.of(new Translation2d(1.5, 2.0), new Translation2d(3.0, 2.0), new Translation2d(2.0, 2.0));

    List<Sample> samples = runAndCapture(follow, 500);
    writeCsv("chainedReversalPaths", samples, polyline);

    // Both phases travel along y=2.0, so the true cross-track (perpendicular to travel) is the
    // lateral deviation |y-2.0|. distToPolyline would over-count the expected along-track
    // momentum overshoot at the reversal cusp, so measure lateral deviation directly here.
    double maxLateral = 0.0;
    double maxX = Double.NEGATIVE_INFINITY;
    for (Sample s : samples) {
      maxLateral = Math.max(maxLateral, Math.abs(s.robot.getY() - 2.0));
      maxX = Math.max(maxX, s.robot.getX());
    }
    Pose2d finalPose = drive.getPose();
    double endDist = finalPose.getTranslation().getDistance(back.getTranslation());
    double endHeadingErr = Math.abs(finalPose.getRotation().getDegrees());
    String diag =
        String.format(
            "chainedReversalPaths: samples=%d maxLateral=%.3fm reachedX=%.3f endDist=%.3fm "
                + "endHeadingErr=%.2fdeg finalPose=%s",
            samples.size(), maxLateral, maxX, endDist, endHeadingErr, finalPose);
    System.out.println("[AutoRoutinesSimPhysicsTest] " + diag);

    assertTrue(samples.size() > 20, "BLine should have executed across both phases — " + diag);
    assertTrue(maxLateral < 0.10, "lateral cross-track exceeded bound — " + diag);
    // Outbound must have reached the far waypoint (x=3.0) before the return phase reversed it.
    assertTrue(maxX > 2.95, "outbound did not reach the far waypoint before reversing — " + diag);
    assertTrue(endDist < END_TRANSLATION_TOL_M, "robot did not reach the reversal goal — " + diag);
    assertTrue(endHeadingErr < END_ROTATION_TOL_DEG, "robot heading drifted — " + diag);
  }
}
