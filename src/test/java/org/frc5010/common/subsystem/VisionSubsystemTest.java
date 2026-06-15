package org.frc5010.common.subsystem;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.frc5010.common.vision.CameraConfig;
import org.frc5010.common.vision.Vision;
import org.frc5010.common.vision.VisionIO;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Layer 2 — unit tests for {@link Vision} subsystem logic.
 *
 * <p>Uses hand-crafted {@link VisionIO} stubs to inject known observations,
 * so tests are purely about the filtering, std-dev model, and consumer-forwarding
 * logic in {@code Vision.periodic()} — not about camera hardware or simulation.
 */
class VisionSubsystemTest extends SimTestBase {

  private static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** A valid in-field pose: centre of the 2026 field. */
  private static final Pose3d VALID_POSE = new Pose3d(
      LAYOUT.getFieldLength() / 2.0,
      LAYOUT.getFieldWidth()  / 2.0,
      0.0,
      new Rotation3d());

  private CameraConfig cfg;
  private List<Pose2d> acceptedPoses;
  private Vision.VisionConsumer consumer;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM);
    cfg           = new CameraConfig.Builder("test_cam").build();
    acceptedPoses = new ArrayList<>();
    consumer      = (pose, ts, stdDevs) -> acceptedPoses.add(pose);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  // ---------------------------------------------------------------------------
  // Acceptance and forwarding
  // ---------------------------------------------------------------------------

  @Test
  void acceptedObservationIsForwardedToConsumer() {
    Vision vision = buildWith(new VisionIO() {
      @Override public void updateInputs(VisionIOInputs inputs) {
        inputs.connected               = true;
        inputs.observationTimestamps   = new double[] {1.0};
        inputs.observationPoses        = new Pose3d[]  {VALID_POSE};
        inputs.observationAmbiguities  = new double[] {0.1};   // below MAX_AMBIGUITY
        inputs.observationTagCounts    = new int[]    {2};      // multi-tag
        inputs.observationTagDistances = new double[] {1.0};
        inputs.observationTypes        = new int[]    {PoseObservationType.PHOTONVISION.ordinal()};
        inputs.tagIds                  = new int[]    {1};
      }
    });

    vision.periodic();
    stepOneCycle();

    assertEquals(1, acceptedPoses.size(), "Consumer should receive exactly one pose");
    assertEquals(VALID_POSE.getX(), acceptedPoses.get(0).getX(), 1e-6);
    assertEquals(VALID_POSE.getY(), acceptedPoses.get(0).getY(), 1e-6);
  }

  // ---------------------------------------------------------------------------
  // Visible-tag reporting (web UI highlight feed)
  // ---------------------------------------------------------------------------

  @Test
  void visibleTagIdsReflectSeenTagsEachCycle() {
    int[][] seen = { { 1, 7 } };
    Vision vision = buildWith(new VisionIO() {
      @Override public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = true;
        inputs.tagIds    = seen[0];
      }
    });

    // Before any cycle: nothing seen.
    assertEquals(0, vision.getVisibleTagIds().length, "No tags reported before first periodic");

    vision.periodic();
    int[] ids = vision.getVisibleTagIds();
    java.util.Arrays.sort(ids);
    assertArrayEquals(new int[] {1, 7}, ids, "Visible IDs should be the tags seen this cycle");

    // A later cycle with no tags clears the set.
    seen[0] = new int[0];
    vision.periodic();
    assertEquals(0, vision.getVisibleTagIds().length, "Visible IDs should clear when no tags seen");
  }

  // ---------------------------------------------------------------------------
  // Rejection filters
  // ---------------------------------------------------------------------------

  @Test
  void highAmbiguitySingleTagIsRejected() {
    Vision vision = buildWith(new VisionIO() {
      @Override public void updateInputs(VisionIOInputs inputs) {
        inputs.connected               = true;
        inputs.observationTimestamps   = new double[] {1.0};
        inputs.observationPoses        = new Pose3d[]  {VALID_POSE};
        inputs.observationAmbiguities  = new double[] {0.5};  // > MAX_AMBIGUITY (0.3)
        inputs.observationTagCounts    = new int[]    {1};    // single-tag → ambiguity gate active
        inputs.observationTagDistances = new double[] {1.0};
        inputs.observationTypes        = new int[]    {PoseObservationType.PHOTONVISION.ordinal()};
        inputs.tagIds                  = new int[]    {};
      }
    });

    vision.periodic();
    stepOneCycle();

    assertTrue(acceptedPoses.isEmpty(), "High-ambiguity single-tag should be rejected");
  }

  @Test
  void zeroTagCountIsAlwaysRejected() {
    Vision vision = buildWith(new VisionIO() {
      @Override public void updateInputs(VisionIOInputs inputs) {
        inputs.connected               = true;
        inputs.observationTimestamps   = new double[] {1.0};
        inputs.observationPoses        = new Pose3d[]  {VALID_POSE};
        inputs.observationAmbiguities  = new double[] {0.0};
        inputs.observationTagCounts    = new int[]    {0};   // no tags
        inputs.observationTagDistances = new double[] {1.0};
        inputs.observationTypes        = new int[]    {PoseObservationType.PHOTONVISION.ordinal()};
        inputs.tagIds                  = new int[]    {};
      }
    });

    vision.periodic();
    stepOneCycle();

    assertTrue(acceptedPoses.isEmpty(), "Zero-tag observation must always be rejected");
  }

  @Test
  void outOfFieldPoseIsRejected() {
    Pose3d outsidePose = new Pose3d(-1.0, 3.0, 0.0, new Rotation3d()); // X < 0

    Vision vision = buildWith(new VisionIO() {
      @Override public void updateInputs(VisionIOInputs inputs) {
        inputs.connected               = true;
        inputs.observationTimestamps   = new double[] {1.0};
        inputs.observationPoses        = new Pose3d[]  {outsidePose};
        inputs.observationAmbiguities  = new double[] {0.1};
        inputs.observationTagCounts    = new int[]    {2};
        inputs.observationTagDistances = new double[] {1.0};
        inputs.observationTypes        = new int[]    {PoseObservationType.PHOTONVISION.ordinal()};
        inputs.tagIds                  = new int[]    {};
      }
    });

    vision.periodic();
    stepOneCycle();

    assertTrue(acceptedPoses.isEmpty(), "Out-of-field pose should be rejected");
  }

  // ---------------------------------------------------------------------------
  // Std-dev model
  // ---------------------------------------------------------------------------

  @Test
  void megaTag2GetsHigherAngularStdDev() {
    // PHOTONVISION observation
    List<Matrix<N3, N1>> pvStdDevs = new ArrayList<>();
    Vision visionPV = new Vision(
        (pose, ts, s) -> pvStdDevs.add(s), LAYOUT, new CameraConfig[]{cfg},
        new VisionIO() {
          @Override public void updateInputs(VisionIOInputs inputs) {
            inputs.connected               = true;
            inputs.observationTimestamps   = new double[] {1.0};
            inputs.observationPoses        = new Pose3d[]  {VALID_POSE};
            inputs.observationAmbiguities  = new double[] {0.0};
            inputs.observationTagCounts    = new int[]    {2};
            inputs.observationTagDistances = new double[] {1.0};
            inputs.observationTypes        = new int[]    {PoseObservationType.PHOTONVISION.ordinal()};
            inputs.tagIds                  = new int[]    {};
          }
        });
    visionPV.periodic();

    // MEGATAG_2 observation at same distance and tag count
    List<Matrix<N3, N1>> mt2StdDevs = new ArrayList<>();
    Vision visionMT2 = new Vision(
        (pose, ts, s) -> mt2StdDevs.add(s), LAYOUT, new CameraConfig[]{cfg},
        new VisionIO() {
          @Override public void updateInputs(VisionIOInputs inputs) {
            inputs.connected               = true;
            inputs.observationTimestamps   = new double[] {1.0};
            inputs.observationPoses        = new Pose3d[]  {VALID_POSE};
            inputs.observationAmbiguities  = new double[] {0.0};
            inputs.observationTagCounts    = new int[]    {2};
            inputs.observationTagDistances = new double[] {1.0};
            inputs.observationTypes        = new int[]    {PoseObservationType.MEGATAG_2.ordinal()};
            inputs.tagIds                  = new int[]    {};
          }
        });
    visionMT2.periodic();

    assertEquals(1, pvStdDevs.size(),  "PHOTONVISION: consumer called once");
    assertEquals(1, mt2StdDevs.size(), "MEGATAG_2: consumer called once");

    double pvAngStd  = pvStdDevs.get(0).get(2, 0);
    double mt2AngStd = mt2StdDevs.get(0).get(2, 0);
    assertTrue(mt2AngStd > pvAngStd * 100,
        "MegaTag 2 angular std dev should be orders of magnitude higher than PhotonVision");
  }

  // ---------------------------------------------------------------------------
  // Helper
  // ---------------------------------------------------------------------------

  private Vision buildWith(VisionIO io) {
    return new Vision(consumer, LAYOUT, new CameraConfig[]{cfg}, io);
  }
}
