package frc.robot.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
import java.util.List;
import org.frc5010.common.drive.swerve.auto.PathPlannerToBLine;
import org.junit.jupiter.api.Test;

/**
 * Layer 1 unit tests for {@link PathPlannerToBLine} — verifies a real Orbit PathPlanner path is
 * sampled into a BLine polyline with the right element structure, endpoint headings, and the
 * embedded rotation targets / event markers carried through.
 */
class PathPlannerToBLineTest {

  /** {@code TR-CTR-QTR}: 3 waypoints (2 Bézier segments), 2 rotation targets, 1 {@code intakeIntake} marker. */
  @Test
  void samplesBezierIntoPolylineWithEndpointsAsWaypoints() {
    int n = 10;
    Path path = PathPlannerToBLine.load("TR-CTR-QTR", n);
    List<PathElement> elements = path.getPathElements();

    long waypoints = elements.stream().filter(e -> e instanceof Waypoint).count();
    long translations = elements.stream().filter(e -> e instanceof TranslationTarget).count();

    // 2 segments × n sub-steps + 1 = 21 sampled points; first & last are Waypoints, the rest
    // TranslationTargets. (Rotation/event elements are extra and counted separately below.)
    assertEquals(2, waypoints, "first and last sampled points must be Waypoints");
    assertEquals(2 * n + 1 - 2, translations, "interior sampled points must be TranslationTargets");

    assertInstanceOf(Waypoint.class, elements.get(0), "path must start with a Waypoint");
    assertInstanceOf(Waypoint.class, elements.get(elements.size() - 1), "path must end with a Waypoint");
  }

  @Test
  void carriesStartAndEndHeadings() {
    Path path = PathPlannerToBLine.load("TR-CTR-QTR", 8);
    List<PathElement> elements = path.getPathElements();

    Waypoint start = (Waypoint) elements.get(0);
    Waypoint end = (Waypoint) elements.get(elements.size() - 1);

    // idealStartingState.rotation = 90°, goalEndState.rotation = 90° for TR-CTR-QTR.
    assertEquals(
        90.0,
        start.rotationTarget().rotation().getDegrees(),
        1e-6,
        "start heading from idealStartingState");
    assertEquals(
        90.0, end.rotationTarget().rotation().getDegrees(), 1e-6, "end heading from goalEndState");
  }

  @Test
  void carriesRotationTargetsAndEventMarkers() {
    Path path = PathPlannerToBLine.load("TR-CTR-QTR", 12);
    List<PathElement> elements = path.getPathElements();

    long rotationTargets = elements.stream().filter(e -> e instanceof RotationTarget).count();
    List<EventTrigger> events =
        elements.stream().filter(e -> e instanceof EventTrigger).map(e -> (EventTrigger) e).toList();

    assertEquals(2, rotationTargets, "both PathPlanner rotation targets must be carried through");
    assertEquals(1, events.size(), "the intakeIntake event marker must be carried through");
    assertEquals("intakeIntake", events.get(0).libKey(), "event marker key must be the marker name");

    for (PathElement e : elements) {
      if (e instanceof EventTrigger ev) {
        assertTrue(ev.t_ratio() >= 0.0 && ev.t_ratio() <= 1.0, "event t_ratio in [0,1]");
      }
      if (e instanceof RotationTarget rt) {
        assertTrue(rt.t_ratio() >= 0.0 && rt.t_ratio() <= 1.0, "rotation t_ratio in [0,1]");
      }
    }
  }

  @Test
  void higherSamplingProducesMorePoints() {
    int coarse = PathPlannerToBLine.load("TR-CTR-QTR", 4).getPathElements().size();
    int fine = PathPlannerToBLine.load("TR-CTR-QTR", 16).getPathElements().size();
    assertTrue(fine > coarse, "more samples per segment must yield more path elements");
  }

  @Test
  void singleSegmentPathConverts() {
    // QTR-TR is a simple 2-waypoint path: no interior anchors, still a valid polyline.
    Path path = PathPlannerToBLine.load("QTR-TR", 6);
    List<PathElement> elements = path.getPathElements();
    assertFalse(elements.isEmpty(), "converted path must have elements");
    assertInstanceOf(Waypoint.class, elements.get(0));
    assertInstanceOf(Waypoint.class, elements.get(elements.size() - 1));
  }

  /**
   * Every PathPlanner path deployed for the ported autos must convert to a structurally valid
   * BLine polyline (start/end Waypoints, ≥ 2 elements). Guards the whole copied path library at
   * once, including the space- and mixed-case-named files.
   */
  @Test
  void everyDeployedPathConverts() {
    java.io.File dir =
        new java.io.File(
            edu.wpi.first.wpilibj.Filesystem.getDeployDirectory(), "pathplanner/paths");
    java.io.File[] files = dir.listFiles((d, name) -> name.endsWith(".path"));
    org.junit.jupiter.api.Assertions.assertNotNull(files, "deploy paths dir must exist");
    assertTrue(files.length >= 40, "expected the full ported path library, found " + files.length);

    for (java.io.File f : files) {
      String name = f.getName().substring(0, f.getName().length() - ".path".length());
      List<PathElement> elements = PathPlannerToBLine.load(name).getPathElements();
      assertTrue(elements.size() >= 2, name + " must convert to ≥ 2 elements");
      assertInstanceOf(Waypoint.class, elements.get(0), name + " must start with a Waypoint");
      assertInstanceOf(
          Waypoint.class, elements.get(elements.size() - 1), name + " must end with a Waypoint");
    }
  }
}
