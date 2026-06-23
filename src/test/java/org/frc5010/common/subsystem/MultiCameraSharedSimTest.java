package org.frc5010.common.subsystem;

import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.lang.reflect.Field;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;
import org.frc5010.common.util.SimTestBase;
import org.frc5010.common.vision.CameraConfig;
import org.frc5010.common.vision.Vision;
import org.frc5010.common.vision.VisionFactory;
import org.frc5010.common.vision.VisionIO;
import org.frc5010.common.vision.VisionIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Verifies that the vision sim uses <b>one</b> PhotonVision {@code VisionSystemSim} regardless of
 * how many cameras are configured. Two PHOTON cameras are built through {@link VisionFactory}; the
 * test confirms (1) both {@link VisionIOSim} instances reference the same shared simulator and
 * (2) both still detect tags off that single simulator after one cycle.
 *
 * <p>Headless — like {@code VisionSimIntegrationTest}'s Test 1, the cameras are placed at a fixed
 * pose ({@link #VISION_SPAWN}) with line-of-sight to default-field tags 25/26; no drive needed.
 */
class MultiCameraSharedSimTest extends SimTestBase {

  /** Spawn that places a forward-facing camera in direct line-of-sight of tags 25 and 26. */
  private static final Pose2d VISION_SPAWN = new Pose2d(2.0, 4.0, Rotation2d.kZero);

  // Two forward-facing cameras at the same mount: both see the same tags through the shared sim.
  private static final Transform3d FRONT_CAM = new Transform3d(
      new Translation3d(0.30, 0.0, 0.50), new Rotation3d());

  private static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    RobotMode.set(Mode.SIM); // factory selects VisionIOSim for PHOTON cameras only in SIM
  }

  @AfterEach
  @Override
  public void simTeardown() {
    RobotMode.resetForTesting();
    super.simTeardown();
  }

  @Test
  void twoPhotonCamerasShareOneSimulatorAndBothDetectTags() throws Exception {
    Vision vision = VisionFactory.build(
        (pose, ts, std) -> { /* consumer not under test here */ },
        () -> VISION_SPAWN,
        () -> VISION_SPAWN.getRotation(),
        new CameraConfig[] {
            new CameraConfig.Builder("shared_cam_a")
                .robotToCamera(FRONT_CAM)
                .backend(CameraConfig.Backend.PHOTON)
                .build(),
            new CameraConfig.Builder("shared_cam_b")
                .robotToCamera(FRONT_CAM)
                .backend(CameraConfig.Backend.PHOTON)
                .build(),
        });

    VisionIO[] io = ioArray(vision);

    // (1) Both sim cameras must point at the very same VisionSystemSim instance.
    Object simA = sharedSim((VisionIOSim) io[0]);
    Object simB = sharedSim((VisionIOSim) io[1]);
    assertSame(simA, simB,
        "All PhotonVision sim cameras must share a single VisionSystemSim, regardless of count");

    // (2) One cycle: the shared sim is stepped once, yet both cameras read observations from it.
    vision.periodic();
    var inputs = inputsArray(vision);
    assertTrue(observationCount(inputs[0]) >= 1,
        "Camera A must detect tags through the shared simulator");
    assertTrue(observationCount(inputs[1]) >= 1,
        "Camera B must detect tags through the shared simulator");
  }

  // --- reflection helpers (no production test-seams; mirrors the project's other sim tests) ---

  private static VisionIO[] ioArray(Vision vision) throws Exception {
    Field f = Vision.class.getDeclaredField("io");
    f.setAccessible(true);
    return (VisionIO[]) f.get(vision);
  }

  private static Object[] inputsArray(Vision vision) throws Exception {
    Field f = Vision.class.getDeclaredField("inputs");
    f.setAccessible(true);
    return (Object[]) f.get(vision);
  }

  private static Object sharedSim(VisionIOSim io) throws Exception {
    Field f = VisionIOSim.class.getDeclaredField("visionSim");
    f.setAccessible(true);
    return f.get(io);
  }

  private static int observationCount(Object inputs) throws Exception {
    Field f = inputs.getClass().getField("observationTimestamps");
    return ((double[]) f.get(inputs)).length;
  }
}
