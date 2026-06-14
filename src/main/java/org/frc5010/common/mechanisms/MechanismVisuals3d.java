package org.frc5010.common.mechanisms;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Robot-frame 3D visualization registry for the mechanism subsystems.
 *
 * <p>{@link MechanismVisuals} gives a single 2D side view, but robots are 3D — an
 * elevator at the back-left and a turret spinning in the horizontal plane can't share
 * one plane. Each mechanism therefore also carries a {@code settings.visualPose3d}
 * <b>mount pose</b>: where the mechanism sits on the robot (x forward, y left, z up,
 * meters from robot center at floor level) <em>and</em> how its working plane is
 * oriented. With the default identity rotation the mechanism moves in the robot's X-Z
 * (side-view) plane, exactly like its Mechanism2d; rotating the mount re-aims that
 * plane — e.g. {@link #YAW_PLANE} lays it flat so a pivot becomes a turret spinning
 * about the vertical axis.
 *
 * <p>Every cycle each mechanism publishes its current line segments (robot frame) here.
 * Two consumers read them:
 * <ul>
 *   <li>the sim web UI's isometric robot view ({@code /api/mechanisms3d} →
 *       {@link #toJson()}), and</li>
 *   <li>AdvantageScope's 3D field view — {@link #publish} logs one {@link Pose3d} per
 *       segment under <b>Mechanisms3d/&lt;name&gt;</b> (position = segment start,
 *       X-axis aligned along the segment), ready to attach as articulated components.</li>
 * </ul>
 *
 * <p>Thread model: {@link #publish}/{@link #remove} run on the robot thread;
 * {@link #toJson()} may run on HTTP threads. Segment lists are immutable snapshots in a
 * concurrent map, so readers always see a consistent cycle.
 */
public final class MechanismVisuals3d {

  /**
   * Mount rotation that lays the mechanism's working plane flat (horizontal): the
   * mechanism angle becomes a yaw about the robot's vertical axis, 0° = robot forward,
   * positive CCW. Use for turrets and other vertical-axis pivots.
   */
  public static final Rotation3d YAW_PLANE = new Rotation3d(-Math.PI / 2, 0, 0);

  /**
   * Mount rotation that stands the mechanism's working plane up in the robot's Y-Z
   * plane: the mechanism angle sweeps side-to-side (about the robot's fore-aft axis),
   * 0° = robot-left, positive toward up. Use for side-mounted deploys / arms that
   * swing left-right rather than fore-aft.
   */
  public static final Rotation3d ROLL_PLANE = new Rotation3d(0, 0, Math.PI / 2);

  /**
   * One drawn line segment in the robot frame.
   *
   * @param label    segment role within the mechanism (e.g. "carriage", "goal")
   * @param start    start point, robot frame, meters
   * @param end      end point, robot frame, meters
   * @param colorHex CSS color like {@code "#58a6ff"}
   * @param weight   line weight in pixels for the web renderer (goal ghosts use 1)
   */
  public record Segment(
      String label, Translation3d start, Translation3d end, String colorHex, double weight) {}

  private static final Map<String, List<Segment>> registry = new ConcurrentHashMap<>();

  // Chassis wireframe drawn under the mechanisms; bumper-to-bumper meters.
  private static volatile double chassisLength = 0.8;
  private static volatile double chassisWidth = 0.8;
  private static volatile double chassisHeight = 0.13;

  private MechanismVisuals3d() {}

  /**
   * Sets the chassis box for the standalone {@link #toJson()} envelope (defaults to
   * 0.8 × 0.8 × 0.13 m). Note the {@code -PwebUI} isometric view does not use this — it
   * sizes the chassis from the drivetrain's real bumper dimensions instead.
   *
   * @param lengthMeters bumper-to-bumper along robot X
   * @param widthMeters  bumper-to-bumper along robot Y
   * @param heightMeters floor to chassis top along robot Z
   */
  public static void setChassis(double lengthMeters, double widthMeters, double heightMeters) {
    chassisLength = lengthMeters;
    chassisWidth = widthMeters;
    chassisHeight = heightMeters;
  }

  /**
   * Transforms a point in the mechanism's working plane to the robot frame.
   *
   * @param mount   the mechanism's mount pose on the robot
   * @param planeX  distance along the plane's horizontal axis (Mechanism2d x), meters
   * @param planeUp distance along the plane's vertical axis (Mechanism2d y), meters
   * @return the point in robot coordinates
   */
  public static Translation3d planarPoint(Pose3d mount, double planeX, double planeUp) {
    return localPoint(mount, new Translation3d(planeX, 0, planeUp));
  }

  /**
   * Extends a chain within the mechanism's working plane: from {@code from}, walk
   * {@code length} meters at {@code angleRad} (Mechanism2d convention: 0 = along the
   * plane's horizontal axis, positive CCW toward its vertical axis).
   *
   * @param mount    the mechanism's mount pose on the robot
   * @param from     chain point in robot coordinates (e.g. the previous joint)
   * @param angleRad direction within the plane, radians
   * @param length   segment length, meters
   * @return the chain end point in robot coordinates
   */
  public static Translation3d planarOffset(
      Pose3d mount, Translation3d from, double angleRad, double length) {
    return from.plus(
        new Translation3d(length * Math.cos(angleRad), 0, length * Math.sin(angleRad))
            .rotateBy(mount.getRotation()));
  }

  /**
   * Transforms an arbitrary mechanism-local vector offset to the robot frame and adds
   * it to {@code from} — for out-of-plane geometry like a differential wrist's twist.
   *
   * @param mount the mechanism's mount pose on the robot
   * @param from  base point in robot coordinates
   * @param local offset in the mount's local frame (x = plane horizontal, y = plane
   *              normal, z = plane vertical), meters
   * @return the offset point in robot coordinates
   */
  public static Translation3d localOffset(Pose3d mount, Translation3d from, Translation3d local) {
    return from.plus(local.rotateBy(mount.getRotation()));
  }

  private static Translation3d localPoint(Pose3d mount, Translation3d local) {
    return mount.getTranslation().plus(local.rotateBy(mount.getRotation()));
  }

  /**
   * Resolves a mechanism's mount pose, supporting parent-child coupling. With no parent
   * the mount is absolute (robot frame). With a parent, {@code localPose} is interpreted
   * as an offset <em>relative to the parent's live attachment pose</em> — so a mechanism
   * mounted on another's moving endpoint (an arm on an elevator carriage, a flywheel on
   * an arm tip) tracks it every cycle.
   *
   * @param localPose the mechanism's mount: absolute when {@code parent} is null,
   *                  otherwise an offset from the parent's attachment frame
   * @param parent    supplier of the parent's live attachment pose, or null
   * @return the resolved mount pose in the robot frame
   */
  public static Pose3d resolveMount(Pose3d localPose, Supplier<Pose3d> parent) {
    Pose3d base = parent != null ? parent.get() : null;
    if (base == null) {
      return localPose;
    }
    return base.transformBy(new Transform3d(localPose.getTranslation(), localPose.getRotation()));
  }

  /**
   * Publishes this cycle's segments for one mechanism (replacing last cycle's) and logs
   * the matching {@code Pose3d[]} to AdvantageKit under <b>Mechanisms3d/&lt;name&gt;</b>.
   * Call from the mechanism's {@code periodic()} on the robot thread.
   *
   * @param name     the mechanism's unique name (its telemetry-table name)
   * @param segments the segments to draw this cycle
   */
  public static void publish(String name, List<Segment> segments) {
    List<Segment> snapshot = List.copyOf(segments);
    registry.put(name, snapshot);

    Pose3d[] poses = new Pose3d[snapshot.size()];
    for (int i = 0; i < snapshot.size(); i++) {
      poses[i] = segmentPose(snapshot.get(i));
    }
    Logger.recordOutput("Mechanisms3d/" + name, poses);
  }

  /** Removes a mechanism's segments (call from {@code close()}). */
  public static void remove(String name) {
    registry.remove(name);
  }

  /** A {@link Pose3d} at the segment start with its X-axis pointing along the segment. */
  static Pose3d segmentPose(Segment s) {
    Translation3d dir = s.end().minus(s.start());
    Rotation3d rot = dir.getNorm() < 1e-9
        ? Rotation3d.kZero
        : new Rotation3d(
            VecBuilder.fill(1, 0, 0), VecBuilder.fill(dir.getX(), dir.getY(), dir.getZ()));
    return new Pose3d(s.start(), rot);
  }

  /**
   * Serializes the chassis box and all published mechanisms as JSON. Standalone form
   * (chassis from {@link #setChassis}); the {@code -PwebUI} view instead composes its
   * own envelope with the real drivetrain chassis + swerve wheels around
   * {@link #mechanismsArrayJson()}. Safe to call from HTTP threads.
   */
  public static String toJson() {
    return "{\"chassis\":{\"length\":" + fmt(chassisLength)
        + ",\"width\":" + fmt(chassisWidth)
        + ",\"height\":" + fmt(chassisHeight)
        + "},\"mechanisms\":" + mechanismsArrayJson() + "}";
  }

  /**
   * Serializes just the published mechanisms as a JSON array
   * ({@code [{"name":...,"segments":[...]}, ...]}). Safe to call from HTTP threads.
   */
  public static String mechanismsArrayJson() {
    StringBuilder sb = new StringBuilder(512).append('[');
    // Sorted for a stable order in the UI and in tests.
    boolean firstMech = true;
    for (Map.Entry<String, List<Segment>> e : new TreeMap<>(registry).entrySet()) {
      if (!firstMech) sb.append(',');
      firstMech = false;
      sb.append("{\"name\":\"").append(e.getKey()).append("\",\"segments\":[");
      boolean firstSeg = true;
      for (Segment s : e.getValue()) {
        if (!firstSeg) sb.append(',');
        firstSeg = false;
        sb.append("{\"l\":\"").append(s.label())
            .append("\",\"a\":").append(point(s.start()))
            .append(",\"b\":").append(point(s.end()))
            .append(",\"c\":\"").append(s.colorHex())
            .append("\",\"w\":").append(fmt(s.weight()))
            .append('}');
      }
      sb.append("]}");
    }
    return sb.append(']').toString();
  }

  /** The current segments for one mechanism, or an empty list. For tests. */
  public static List<Segment> getSegments(String name) {
    List<Segment> segments = registry.get(name);
    return segments != null ? segments : List.of();
  }

  /** Approximates a circle in the mechanism's working plane as a segment polyline. */
  public static List<Segment> planarCircle(
      Pose3d mount, double centerPlaneX, double centerPlaneUp, double radius,
      int sides, String label, String colorHex, double weight) {
    List<Segment> segments = new ArrayList<>(sides);
    Translation3d prev = planarPoint(
        mount, centerPlaneX + radius, centerPlaneUp);
    for (int k = 1; k <= sides; k++) {
      double a = 2 * Math.PI * k / sides;
      Translation3d next = planarPoint(
          mount, centerPlaneX + radius * Math.cos(a), centerPlaneUp + radius * Math.sin(a));
      segments.add(new Segment(label, prev, next, colorHex, weight));
      prev = next;
    }
    return segments;
  }

  /** Clears all published mechanisms and restores the default chassis. For unit tests. */
  public static void resetForTesting() {
    registry.clear();
    chassisLength = 0.8;
    chassisWidth = 0.8;
    chassisHeight = 0.13;
  }

  private static String fmt(double v) {
    return String.format(java.util.Locale.ROOT, "%.4f", v);
  }

  private static String point(Translation3d p) {
    return String.format(
        java.util.Locale.ROOT, "[%.4f,%.4f,%.4f]", p.getX(), p.getY(), p.getZ());
  }
}
