package org.frc5010.common.mechanisms;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.frc5010.examples.mechanisms.ExampleElevator;
import java.util.List;
import org.frc5010.common.mechanisms.MechanismVisuals3d.Segment;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Layer 2 — tests for the 3D mechanism visualization registry and its geometry
 * helpers: the planar (Mechanism2d-convention) transforms, the {@code YAW_PLANE}
 * turret mount, the publish/remove lifecycle, the JSON served to the web UI's
 * isometric view, and end-to-end publication from a real mechanism's periodic().
 */
class MechanismVisuals3dTest extends SimTestBase {

  private static final double EPS = 1e-9;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    MechanismVisuals3d.resetForTesting();
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  // ── planar geometry (Mechanism2d convention in 3D) ─────────────────────────

  @Test
  void identityMountKeepsTheSideViewPlane() {
    // Identity rotation = the robot's X-Z plane, exactly like the 2D canvas:
    // planeX maps to robot X, planeUp maps to robot Z, Y untouched.
    Pose3d mount = new Pose3d(1.0, 0.0, 0.5, Rotation3d.kZero);
    Translation3d p = MechanismVisuals3d.planarPoint(mount, 0.2, 0.3);
    assertEquals(1.2, p.getX(), EPS);
    assertEquals(0.0, p.getY(), EPS);
    assertEquals(0.8, p.getZ(), EPS);

    // Angle 90° walks straight up the plane's vertical axis (robot +Z), like a
    // Mechanism2d ligament at 90°.
    Translation3d up = MechanismVisuals3d.planarOffset(
        mount, mount.getTranslation(), Math.PI / 2, 0.6);
    assertEquals(1.0, up.getX(), EPS);
    assertEquals(0.0, up.getY(), EPS);
    assertEquals(1.1, up.getZ(), EPS);
  }

  @Test
  void yawPlaneMountTurnsTheMechanismAngleIntoRobotYaw() {
    // The turret case: with YAW_PLANE the working plane lies flat, so the mechanism
    // angle sweeps in robot X-Y at constant height — 0° points robot-forward,
    // +90° points robot-left.
    Pose3d mount = new Pose3d(0, 0, 0.55, MechanismVisuals3d.YAW_PLANE);
    Translation3d base = MechanismVisuals3d.planarPoint(mount, 0, 0);

    Translation3d forward = MechanismVisuals3d.planarOffset(mount, base, 0, 0.4);
    assertEquals(0.4, forward.getX(), EPS);
    assertEquals(0.0, forward.getY(), EPS);
    assertEquals(0.55, forward.getZ(), EPS);

    Translation3d left = MechanismVisuals3d.planarOffset(mount, base, Math.PI / 2, 0.4);
    assertEquals(0.0, left.getX(), EPS);
    assertEquals(0.4, left.getY(), EPS);
    assertEquals(0.55, left.getZ(), EPS, "a turret's tip must stay at constant height");
  }

  @Test
  void rollPlaneMountSweepsInTheYZPlane() {
    // The side-mounted case: ROLL_PLANE stands the working plane up in robot Y-Z, so
    // the mechanism angle sweeps left-to-up at constant fore-aft (x) position.
    Pose3d mount = new Pose3d(0.3, 0, 0.5, MechanismVisuals3d.ROLL_PLANE);
    Translation3d base = MechanismVisuals3d.planarPoint(mount, 0, 0);

    Translation3d left = MechanismVisuals3d.planarOffset(mount, base, 0, 0.4);
    assertEquals(0.3, left.getX(), EPS, "Y-Z plane mechanism must keep its fore-aft x");
    assertEquals(0.4, left.getY(), EPS);
    assertEquals(0.5, left.getZ(), EPS);

    Translation3d up = MechanismVisuals3d.planarOffset(mount, base, Math.PI / 2, 0.4);
    assertEquals(0.3, up.getX(), EPS);
    assertEquals(0.0, up.getY(), EPS);
    assertEquals(0.9, up.getZ(), EPS);
  }

  @Test
  void resolveMountComposesParentEndpointWithLocalOffset() {
    // Parent yawed 90° about Z at (1,2,3); a child offset 0.5 along the parent's local X
    // lands 0.5 along the parent's +Y in the world.
    Pose3d parent = new Pose3d(1, 2, 3, new Rotation3d(0, 0, Math.PI / 2));
    Pose3d coupled =
        MechanismVisuals3d.resolveMount(new Pose3d(0.5, 0, 0, Rotation3d.kZero), () -> parent);
    assertEquals(1.0, coupled.getX(), 1e-9);
    assertEquals(2.5, coupled.getY(), 1e-9);
    assertEquals(3.0, coupled.getZ(), 1e-9);

    // No parent → the local pose is the absolute mount, unchanged.
    Pose3d absolute =
        MechanismVisuals3d.resolveMount(new Pose3d(1, 1, 1, Rotation3d.kZero), null);
    assertEquals(1.0, absolute.getY(), 1e-9);
  }

  @Test
  void resolveMountAppliesLinkageOffsetBeforeLocalPose() {
    // Parent endpoint at the origin facing forward. A 0.5 m linkage standoff along the
    // parent's +X carries the child off the endpoint; the child's own local pose then
    // adds 0.2 m more along the (now shifted) frame. They compose: 0.7 m total.
    Pose3d parent = new Pose3d(0, 0, 0, Rotation3d.kZero);
    Pose3d coupled = MechanismVisuals3d.resolveMount(
        new Pose3d(0.2, 0, 0, Rotation3d.kZero), () -> parent,
        new edu.wpi.first.math.geometry.Transform3d(
            new Translation3d(0.5, 0, 0), Rotation3d.kZero));
    assertEquals(0.7, coupled.getX(), 1e-9);
    assertEquals(0.0, coupled.getY(), 1e-9);

    // A null linkage offset behaves exactly like the 2-arg resolveMount (offset = none).
    Pose3d noOffset = MechanismVisuals3d.resolveMount(
        new Pose3d(0.2, 0, 0, Rotation3d.kZero), () -> parent, null);
    assertEquals(0.2, noOffset.getX(), 1e-9);
  }

  @Test
  void offsetMountShiftsTheMountInItsOwnFrame() {
    // Identity mount: the local offset is a plain robot-frame shift; rotation unchanged.
    Pose3d mount = new Pose3d(1, 2, 0.5, Rotation3d.kZero);
    Pose3d shifted = MechanismVisuals3d.offsetMount(mount, new Translation3d(0.1, 0.2, 0.3));
    assertEquals(1.1, shifted.getX(), 1e-9);
    assertEquals(2.2, shifted.getY(), 1e-9);
    assertEquals(0.8, shifted.getZ(), 1e-9);
    assertEquals(mount.getRotation(), shifted.getRotation(), "mirror keeps the mount's orientation");

    // Yawed mount (90° about Z): the local +X offset comes out along robot +Y.
    Pose3d yawed = new Pose3d(0, 0, 0, new Rotation3d(0, 0, Math.PI / 2));
    Pose3d yawShift = MechanismVisuals3d.offsetMount(yawed, new Translation3d(0.5, 0, 0));
    assertEquals(0.0, yawShift.getX(), 1e-9);
    assertEquals(0.5, yawShift.getY(), 1e-9);
  }

  @Test
  void followerDrawsAnOffsetMirrorOfTheMechanism() {
    // ExampleElevator carries a follower offset 0.5 m along +Y: every drawn segment must
    // appear twice (lead + mirror), the mirror shifted by the offset, tracking live state.
    var elevator = new ExampleElevator();
    try {
      elevator.periodic();
      List<Segment> segments = MechanismVisuals3d.getSegments("ExampleElevator");

      List<Segment> carriages = segments.stream()
          .filter(s -> "carriage".equals(s.label())).toList();
      assertEquals(2, carriages.size(), "a follower mirrors the whole mechanism");

      Translation3d offset = elevator.getSettings().followerVisualOffset;
      Segment lead = carriages.get(0);
      Segment mirror = carriages.get(1);
      // Identity mount, so the local +Y offset is a robot +Y shift between the two copies.
      assertEquals(offset.getY(), mirror.start().getY() - lead.start().getY(), 1e-6);
      assertEquals(0.0, mirror.start().getX() - lead.start().getX(), 1e-6);
      assertEquals(0.0, mirror.start().getZ() - lead.start().getZ(), 1e-6,
          "the mirror is at the same height as the lead — it tracks the live carriage");
    } finally {
      elevator.close();
    }
  }

  @Test
  void noFollowerDrawsNoMirror() {
    // ExampleArm has no follower configured, so each segment appears exactly once.
    var arm = new org.frc5010.examples.mechanisms.ExampleArm();
    try {
      arm.periodic();
      long armBars = MechanismVisuals3d.getSegments("ExampleArm").stream()
          .filter(s -> "arm".equals(s.label())).count();
      assertEquals(1, armBars, "no follower → no mirror");
    } finally {
      arm.close();
    }
  }

  @Test
  void childMechanismRidesItsParentsEndpoint() {
    // The coupled demo: an arm mounted on an elevator carriage. The arm's base must sit
    // exactly at the elevator's live attachment pose (the carriage), not its own
    // absolute visualPose3d.
    var elevator = new ExampleElevator();
    var arm = new org.frc5010.examples.mechanisms.ExampleArm();
    try {
      arm.getSettings().visualParent = elevator::attachmentPose;
      arm.getSettings().visualPose3d = new Pose3d(); // right on the carriage
      elevator.periodic();
      arm.periodic();

      Pose3d carriage = elevator.attachmentPose();
      assertTrue(carriage.getTranslation().getZ() > 0.05,
          "carriage should ride up to the elevator's starting height");

      Segment armSeg = MechanismVisuals3d.getSegments("ExampleArm").stream()
          .filter(s -> "arm".equals(s.label())).findFirst().orElseThrow();
      assertEquals(0.0, armSeg.start().getDistance(carriage.getTranslation()), 1e-6,
          "the arm's base must track the elevator carriage, not its own absolute mount");
    } finally {
      arm.close();
      elevator.close();
    }
  }

  @Test
  void isoProjectionKeepsZStraightUp() {
    // The Glass iso projection: a purely vertical 3D segment (same x,y) stays vertical on
    // the canvas — px unchanged, py rising by the height times the scale.
    double[] floor = MechanismIsoCanvas.project(new Translation3d(0, 0, 0));
    double[] high = MechanismIsoCanvas.project(new Translation3d(0, 0, 1));
    assertEquals(floor[0], high[0], 1e-9, "vertical segments must not drift sideways");
    assertTrue(high[1] > floor[1], "+z must project upward on the canvas");
  }

  @Test
  void isoCanvasPoolsSlotsAndCollapsesUnusedOnes() {
    // Two segments this cycle, then one next cycle: the pool keeps its high-water size and
    // the now-unused slot collapses to zero length rather than lingering.
    MechanismVisuals3d.publish("IsoMech", List.of(
        new Segment("a", new Translation3d(0, 0, 0), new Translation3d(0, 0, 1), "#58a6ff", 3),
        new Segment("b", new Translation3d(0, 0, 1), new Translation3d(0.5, 0, 1), "#ffffff", 1)));
    assertEquals(2, MechanismIsoCanvas.slotCount("IsoMech"));
    assertTrue(MechanismIsoCanvas.ligamentLength("IsoMech", 1) > 0);

    MechanismVisuals3d.publish("IsoMech", List.of(
        new Segment("a", new Translation3d(0, 0, 0), new Translation3d(0, 0, 1), "#58a6ff", 3)));
    assertEquals(2, MechanismIsoCanvas.slotCount("IsoMech"), "pool keeps its high-water size");
    assertEquals(0.0, MechanismIsoCanvas.ligamentLength("IsoMech", 1), 1e-9,
        "an unused slot must collapse to zero length");

    // remove() hides the surviving slot too.
    MechanismVisuals3d.remove("IsoMech");
    assertEquals(0, MechanismIsoCanvas.slotCount("IsoMech"));
  }

  @Test
  void robotSceneBuildsChassisWheelsAndCompass() {
    // 0.8 x 0.7 chassis, 4 modules: 12 box edges + 4 wheels + 16 compass ring + 1 heading.
    double[][] modules = {
        {0.3, 0.3, 0, 0}, {0.3, -0.3, 0, 0}, {-0.3, 0.3, 0, 0}, {-0.3, -0.3, 0, 0}};
    List<Segment> scene =
        MechanismVisuals3d.robotSceneSegments(0.8, 0.7, 0.15, 0.05, modules, 0.0);
    assertEquals(12, scene.stream().filter(s -> "chassis".equals(s.label())).count());
    assertEquals(4, scene.stream().filter(s -> "wheel".equals(s.label())).count());
    assertEquals(16, scene.stream().filter(s -> "compass".equals(s.label())).count());
    assertEquals(1, scene.stream().filter(s -> "heading".equals(s.label())).count());

    // The heading needle points along +X (robot forward) at gyro 0.
    Segment heading = scene.stream().filter(s -> "heading".equals(s.label())).findFirst().orElseThrow();
    assertTrue(heading.end().getX() > heading.start().getX(), "gyro 0 points robot-forward");
    assertEquals(0.0, heading.end().getY(), 1e-9);
  }

  @Test
  void setRobotSceneRendersOntoTheIsoCanvas() {
    double[][] modules = {{0.3, 0.3, 0, 0}, {0.3, -0.3, 0, 0}};
    MechanismVisuals3d.setRobotScene(0.8, 0.7, 0.15, 0.05, modules, 0.0);
    // 12 chassis + 2 wheels + 16 compass + 1 heading = 31 slots on the shared canvas.
    assertEquals(31, MechanismIsoCanvas.slotCount(MechanismVisuals3d.SCENE_NAME));
  }

  @Test
  void mechanismsArrayJsonIsABareArrayWithoutChassis() {
    MechanismVisuals3d.publish("M", List.of(new Segment(
        "bar", new Translation3d(0, 0, 0), new Translation3d(0, 0, 1), "#58a6ff", 3)));
    String arr = MechanismVisuals3d.mechanismsArrayJson();
    assertTrue(arr.startsWith("[") && arr.endsWith("]"), "must be a JSON array: " + arr);
    assertFalse(arr.contains("chassis"), "array form must not carry the chassis: " + arr);
    assertTrue(arr.contains("\"name\":\"M\""));
  }

  @Test
  void segmentPoseAlignsXAxisAlongTheSegment() {
    // The AdvantageScope component pose: position at the start, X-axis along the
    // segment — here a vertical segment, so unit X must map to unit Z.
    var pose = MechanismVisuals3d.segmentPose(new Segment(
        "s", new Translation3d(0.1, 0.2, 0.3), new Translation3d(0.1, 0.2, 1.3),
        "#ffffff", 1));
    assertEquals(new Translation3d(0.1, 0.2, 0.3), pose.getTranslation());
    Translation3d alongX = new Translation3d(1, 0, 0).rotateBy(pose.getRotation());
    assertEquals(0.0, alongX.getX(), 1e-6);
    assertEquals(0.0, alongX.getY(), 1e-6);
    assertEquals(1.0, alongX.getZ(), 1e-6);
  }

  // ── registry lifecycle + JSON ──────────────────────────────────────────────

  @Test
  void publishGetJsonRemoveRoundTrip() {
    assertTrue(MechanismVisuals3d.getSegments("TestMech").isEmpty());

    MechanismVisuals3d.publish("TestMech", List.of(new Segment(
        "bar", new Translation3d(0, 0, 0), new Translation3d(0, 0, 1), "#58a6ff", 3)));
    assertEquals(1, MechanismVisuals3d.getSegments("TestMech").size());

    String json = MechanismVisuals3d.toJson();
    assertTrue(json.contains("\"name\":\"TestMech\""), "JSON must list the mechanism: " + json);
    assertTrue(json.contains("\"c\":\"#58a6ff\""), "JSON must carry the color: " + json);
    assertTrue(json.contains("\"chassis\""), "JSON must include the chassis box: " + json);
    assertTrue(json.contains("\"b\":[0.0000,0.0000,1.0000]"),
        "JSON must carry the endpoint: " + json);

    MechanismVisuals3d.remove("TestMech");
    assertTrue(MechanismVisuals3d.getSegments("TestMech").isEmpty());
    assertFalse(MechanismVisuals3d.toJson().contains("TestMech"));
  }

  @Test
  void planarCircleClosesOnItself() {
    Pose3d mount = new Pose3d(0, 0, 0.6, Rotation3d.kZero);
    List<Segment> rim = MechanismVisuals3d.planarCircle(
        mount, 0, 0, 0.05, 12, "rim", "#2e6e40", 1);
    assertEquals(12, rim.size());
    // Consecutive segments chain, and the last ends where the first starts.
    for (int i = 1; i < rim.size(); i++) {
      assertEquals(rim.get(i - 1).end(), rim.get(i).start());
    }
    assertEquals(0.0, rim.get(11).end().getDistance(rim.get(0).start()), 1e-9);
  }

  // ── end-to-end: a real mechanism publishes from periodic() ─────────────────

  @Test
  void exampleElevatorPublishesItsCarriageAndCloseRemovesIt() {
    var elevator = new ExampleElevator();
    try {
      elevator.periodic(); // one disabled cycle is enough to publish the visuals

      List<Segment> segments = MechanismVisuals3d.getSegments("ExampleElevator");
      assertFalse(segments.isEmpty(), "periodic() must publish 3D segments");

      Segment carriage = segments.stream()
          .filter(s -> "carriage".equals(s.label())).findFirst().orElseThrow();
      // ExampleElevator mounts at (0.25, 0.25, 0): the carriage bar lies in the
      // side-view plane (y constant) at whatever height the mechanism reports.
      // Don't assert the absolute starting height — the simulated TalonFX seeds its
      // sensor on a real-time device thread, so the first cycles can read stale
      // sim state left by earlier tests on the same CAN ID (docs/mechanisms.md
      // gotcha 6). Consistency with getHeight() is the actual contract.
      assertEquals(0.25, carriage.start().getY(), EPS);
      assertEquals(0.25, carriage.end().getY(), EPS);
      double expectedZ = Math.max(0.02, elevator.getHeight().in(edu.wpi.first.units.Units.Meters));
      assertEquals(expectedZ, carriage.start().getZ(), 1e-6,
          "carriage must publish at the height the mechanism reports");
    } finally {
      elevator.close();
    }
    assertTrue(MechanismVisuals3d.getSegments("ExampleElevator").isEmpty(),
        "close() must remove the mechanism from the 3D registry");
  }
}
