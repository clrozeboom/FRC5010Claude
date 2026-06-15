package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem. Manages an array of {@link VisionIO} implementations, processes their
 * inputs each cycle, and feeds accepted observations to the drive pose estimator via a
 * {@link VisionConsumer} callback.
 *
 * <p>Standard deviation model (adapted from 6328 AdvantageKit template):
 * <ul>
 *   <li>Base std dev scales with {@code averageTagDistance²} and is divided by tag count.
 *   <li>MegaTag 2 gets lower linear std dev (heading is locked) but angular is ignored.
 *   <li>Each camera has an optional per-camera multiplier ({@link CameraConfig#stdDevFactor}).
 * </ul>
 *
 * <p>Rejected poses (high ambiguity, out-of-field, Z error) are logged but not sent to the
 * estimator.
 */
public class Vision extends SubsystemBase {

  // ── Filtering thresholds ────────────────────────────────────────────────────
  private static final double MAX_AMBIGUITY        = 0.3;
  private static final double MAX_Z_ERROR_METERS   = 0.75;

  // ── Standard deviation model ────────────────────────────────────────────────
  /** Base linear std dev (m) at 1 m distance with 1 tag. */
  private static final double LINEAR_STD_DEV_BASE    = 0.02;
  /** Base angular std dev (rad) at 1 m distance with 1 tag. */
  private static final double ANGULAR_STD_DEV_BASE   = 0.06;
  /** MegaTag 2 linear std dev multiplier (heading locked → lower translational noise). */
  private static final double MEGATAG2_LINEAR_FACTOR = 0.5;
  /** MegaTag 2 angular std dev multiplier (heading locked → nearly infinite angular std dev). */
  private static final double MEGATAG2_ANGULAR_FACTOR = 1e6;

  // ── Fields ──────────────────────────────────────────────────────────────────
  private final VisionConsumer consumer;
  private final AprilTagFieldLayout fieldLayout;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final double[] stdDevFactors;
  private final Alert[] disconnectedAlerts;

  /**
   * IDs of every AprilTag any camera saw on the most recent {@link #periodic()} cycle.
   * Replaced wholesale each cycle on the robot thread; read (e.g. by the web UI HTTP
   * thread via {@link #getVisibleTagIds()}) without locking, hence {@code volatile}.
   */
  private volatile int[] visibleTagIds = new int[0];

  /**
   * Constructs a Vision subsystem.
   *
   * @param consumer    Called for each accepted pose observation. Use
   *                    {@code drive::addVisionMeasurement}.
   * @param fieldLayout AprilTag field layout for boundary checking and tag visualization.
   * @param configs     Per-camera configurations (order matches {@code io}).
   * @param io          One {@link VisionIO} per camera, in the same order as {@code configs}.
   */
  public Vision(
      VisionConsumer consumer,
      AprilTagFieldLayout fieldLayout,
      CameraConfig[] configs,
      VisionIO... io) {
    this.consumer = consumer;
    this.fieldLayout = fieldLayout;
    this.io = io;

    inputs = new VisionIOInputsAutoLogged[io.length];
    stdDevFactors = new double[io.length];
    disconnectedAlerts = new Alert[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      stdDevFactors[i] = (i < configs.length) ? configs[i].stdDevFactor : 1.0;
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " (" + (i < configs.length ? configs[i].name : "?")
              + ") disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    // Update and log all camera inputs.
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    List<Pose3d> allTagPoses   = new ArrayList<>();
    List<Pose3d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allAccepted   = new ArrayList<>();
    List<Pose3d> allRejected   = new ArrayList<>();
    List<Integer> allTagIds    = new ArrayList<>();

    for (int ci = 0; ci < io.length; ci++) {
      disconnectedAlerts[ci].set(!inputs[ci].connected);

      List<Pose3d> tagPoses   = new ArrayList<>();
      List<Pose3d> robotPoses = new ArrayList<>();
      List<Pose3d> accepted   = new ArrayList<>();
      List<Pose3d> rejected   = new ArrayList<>();

      // Visualize seen tags.
      for (int id : inputs[ci].tagIds) {
        fieldLayout.getTagPose(id).ifPresent(tagPoses::add);
        if (!allTagIds.contains(id)) allTagIds.add(id);
      }

      // Reconstruct PoseObservation from parallel arrays and process each one.
      int n = inputs[ci].observationTimestamps.length;
      for (int oi = 0; oi < n; oi++) {
        Pose3d pose        = inputs[ci].observationPoses[oi];
        double ambiguity   = inputs[ci].observationAmbiguities[oi];
        int    tagCount    = inputs[ci].observationTagCounts[oi];
        double tagDist     = inputs[ci].observationTagDistances[oi];
        double timestamp   = inputs[ci].observationTimestamps[oi];
        VisionIO.PoseObservationType type =
            VisionIO.PoseObservationType.values()[inputs[ci].observationTypes[oi]];

        boolean reject =
            tagCount == 0
                || (tagCount == 1 && ambiguity > MAX_AMBIGUITY)
                || Math.abs(pose.getZ()) > MAX_Z_ERROR_METERS
                || pose.getX() < 0.0
                || pose.getX() > fieldLayout.getFieldLength()
                || pose.getY() < 0.0
                || pose.getY() > fieldLayout.getFieldWidth();

        robotPoses.add(pose);
        (reject ? rejected : accepted).add(pose);

        if (reject) continue;

        // Compute std devs: scale with distance² / tagCount.
        double factor = Math.pow(tagDist, 2.0) / tagCount * stdDevFactors[ci];
        double linStdDev = LINEAR_STD_DEV_BASE * factor;
        double angStdDev = ANGULAR_STD_DEV_BASE * factor;

        if (type == VisionIO.PoseObservationType.MEGATAG_2) {
          linStdDev *= MEGATAG2_LINEAR_FACTOR;
          angStdDev *= MEGATAG2_ANGULAR_FACTOR;
        }

        consumer.accept(
            pose.toPose2d(),
            timestamp,
            VecBuilder.fill(linStdDev, linStdDev, angStdDev));
      }

      Logger.recordOutput("Vision/Camera" + ci + "/TagPoses",   tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Camera" + ci + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Camera" + ci + "/Accepted",   accepted.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Camera" + ci + "/Rejected",   rejected.toArray(new Pose3d[0]));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allAccepted.addAll(accepted);
      allRejected.addAll(rejected);
    }

    Logger.recordOutput("Vision/Summary/TagPoses",   allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/Accepted",   allAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/Rejected",   allRejected.toArray(new Pose3d[0]));

    // Publish the per-cycle set of visible tag IDs for consumers outside the robot loop
    // (e.g. the web UI, which highlights tags currently in view).
    int[] ids = new int[allTagIds.size()];
    for (int i = 0; i < ids.length; i++) ids[i] = allTagIds.get(i);
    visibleTagIds = ids;
  }

  /**
   * Returns the IDs of every AprilTag any camera saw on the most recent cycle.
   *
   * <p>Thread-safe: returns a snapshot reference that is replaced wholesale each cycle, so
   * callers (such as the web UI HTTP threads) may read it without locking.
   *
   * @return the visible tag IDs (empty when no tags are in view); do not mutate
   */
  public int[] getVisibleTagIds() {
    return visibleTagIds;
  }

  /** Matches the signature of {@code AkitSwerveDrive.addVisionMeasurement}. */
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }
}
