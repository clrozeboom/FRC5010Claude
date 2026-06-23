package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import org.frc5010.common.robot.Mode;
import org.frc5010.common.robot.RobotMode;

/**
 * Factory for constructing a {@link Vision} subsystem.
 *
 * <p>Usage:
 * <pre>{@code
 * Vision vision = VisionFactory.build(
 *     drive::addVisionMeasurement,
 *     drive::getPose,
 *     drive::getRotation,
 *     new CameraConfig[] {
 *         new CameraConfig.Builder("photon_front")
 *             .robotToCamera(FRONT_CAM_TRANSFORM)
 *             .backend(CameraConfig.Backend.PHOTON)
 *             .build(),
 *         new CameraConfig.Builder("limelight")
 *             .robotToCamera(REAR_CAM_TRANSFORM)
 *             .backend(CameraConfig.Backend.LIMELIGHT)
 *             .build()
 *     });
 * }</pre>
 *
 * <p>In REAL mode, hardware IO is wired ({@link VisionIOPhoton} or {@link VisionIOLimelight}).
 * In SIM mode, {@link VisionIOSim} is used for PhotonVision cameras; Limelight cameras fall
 * back to a no-op IO (Limelights don't have a PhotonVision sim equivalent).
 * In REPLAY mode, a no-op IO is used — AdvantageKit replays logged inputs automatically.
 */
public final class VisionFactory {

  private VisionFactory() {}

  /**
   * Builds a {@link Vision} subsystem wired for the current robot mode.
   *
   * @param consumer       Pose measurement consumer — use {@code drive::addVisionMeasurement}.
   * @param poseSupplier   Current robot pose — use {@code drive::getPose} (for sim).
   * @param headingSupplier Current robot heading — use {@code drive::getRotation} (for Limelight).
   * @param configs        Per-camera configurations; length determines number of cameras.
   */
  public static Vision build(
      Vision.VisionConsumer consumer,
      Supplier<Pose2d> poseSupplier,
      Supplier<Rotation2d> headingSupplier,
      CameraConfig[] configs) {

    // Single source of truth: the field layout the active profile published into the shared
    // AprilTags holder, so pose estimation and the field-geometry helpers never diverge.
    AprilTagFieldLayout layout = AprilTags.aprilTagFieldLayout;
    Mode mode = RobotMode.get();

    // One PhotonVision simulator shared by every sim camera, regardless of camera count —
    // created lazily on the first PHOTON camera in SIM mode (null otherwise).
    SharedVisionSim sharedSim = null;

    VisionIO[] io = new VisionIO[configs.length];
    for (int i = 0; i < configs.length; i++) {
      CameraConfig cfg = configs[i];
      io[i] = switch (mode) {
        case REAL -> switch (cfg.backend) {
          case PHOTON   -> new VisionIOPhoton(cfg, layout);
          case LIMELIGHT -> new VisionIOLimelight(cfg, headingSupplier);
          // QuestNav is a NetworkTables source — seed its field frame from the robot pose.
          case QUESTNAV -> new VisionIOQuestNav(cfg, poseSupplier);
        };
        case SIM -> switch (cfg.backend) {
          case PHOTON    -> {
            if (sharedSim == null) sharedSim = new SharedVisionSim(layout);
            yield new VisionIOSim(cfg, layout, poseSupplier, sharedSim);
          }
          // Limelight has no PhotonVision sim equivalent — use no-op; logs will show no tags.
          case LIMELIGHT -> new VisionIO() {};
          // No Quest headset in simulation — use no-op; logs will show no QuestNav poses.
          case QUESTNAV -> new VisionIO() {};
        };
        // REPLAY: no-op — AKit replays logged inputs automatically.
        case REPLAY -> new VisionIO() {};
      };
    }

    return new Vision(consumer, layout, configs, io);
  }
}
