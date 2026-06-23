package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * VisionIO simulation implementation using PhotonVision's {@link VisionSystemSim}.
 *
 * <p>Extends {@link VisionIOPhoton} — the same {@code PhotonCamera} object is used for both
 * real and simulated result parsing, so {@code updateInputs()} logic is shared.
 * Each cycle, the sim is updated with the robot's current pose before reading results.
 *
 * <p>The camera registers into a {@link SharedVisionSim}, so a robot with several cameras uses
 * <b>one</b> {@code VisionSystemSim} for all of them (see {@link VisionFactory}). Constructed
 * standalone, it gets its own dedicated sim — convenient for single-camera setups and tests.
 *
 * <p>Works for both Layer 2 tests ({@code buildWithoutPhysics}) and Layer 3 tests
 * ({@code build()}) — the caller supplies the pose via {@code poseSupplier}.
 */
public class VisionIOSim extends VisionIOPhoton {

  private final SharedVisionSim visionSim;
  private final Supplier<Pose2d> poseSupplier;

  /**
   * Standalone constructor — this camera gets its own dedicated {@link VisionSystemSim}. Use for
   * single-camera setups and direct construction in tests. When the {@link VisionFactory} builds
   * several cameras together they instead share one sim.
   *
   * @param config       Camera config — name, transform, and backend (must be PHOTON).
   * @param layout       Field AprilTag layout used to place simulated targets.
   * @param poseSupplier Current robot pose; typically {@code drive::getPose}.
   */
  public VisionIOSim(CameraConfig config, AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier) {
    this(config, layout, poseSupplier, new SharedVisionSim(layout));
  }

  /**
   * Shared-sim constructor — registers this camera into a {@link SharedVisionSim} owned by the
   * caller, so every camera built together resolves against the one simulator.
   */
  VisionIOSim(
      CameraConfig config,
      AprilTagFieldLayout layout,
      Supplier<Pose2d> poseSupplier,
      SharedVisionSim sharedSim) {
    super(config, layout); // creates the PhotonCamera
    this.poseSupplier = poseSupplier;
    this.visionSim = sharedSim;

    SimCameraProperties props = SimCameraProperties.PERFECT_90DEG();
    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, props);
    cameraSim.enableDrawWireframe(true);
    visionSim.addCamera(cameraSim, config.robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.updateOncePerLoop(poseSupplier.get()); // advance shared sim (once per loop)
    super.updateInputs(inputs);                      // read camera results via PhotonCamera
  }
}
