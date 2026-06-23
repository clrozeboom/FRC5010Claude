package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

/**
 * One {@link VisionSystemSim} shared by every PhotonVision sim camera on the robot.
 *
 * <p>PhotonVision's {@code VisionSystemSim} already models any number of cameras: each
 * {@link PhotonCameraSim} is registered with {@link #addCamera}, and a single
 * {@link VisionSystemSim#update} advances all of them against the one field. Sharing a single
 * instance — instead of constructing one per camera — keeps every camera ray-casting against the
 * same world, publishes a single "Sim Field" debug widget, and steps the simulation exactly once
 * per robot loop no matter how many cameras are wired.
 *
 * <p>{@link #updateOncePerLoop} de-duplicates by FPGA timestamp: every camera's
 * {@code updateInputs()} calls it within a cycle, but the underlying sim is stepped only the first
 * time per timestamp. This is order-independent (no camera has to be designated the "primary").
 */
final class SharedVisionSim {

  private final VisionSystemSim sim;
  private double lastUpdateTime = Double.NaN;

  SharedVisionSim(AprilTagFieldLayout layout) {
    sim = new VisionSystemSim("vision");
    sim.addAprilTags(layout);
  }

  /** Registers a camera into the shared sim at its robot-relative mount transform. */
  void addCamera(PhotonCameraSim cameraSim, Transform3d robotToCamera) {
    sim.addCamera(cameraSim, robotToCamera);
  }

  /** Advances the shared sim to {@code robotPose}, but at most once per robot loop. */
  void updateOncePerLoop(Pose2d robotPose) {
    double now = Timer.getFPGATimestamp();
    if (now != lastUpdateTime) {
      sim.update(robotPose);
      lastUpdateTime = now;
    }
  }
}
