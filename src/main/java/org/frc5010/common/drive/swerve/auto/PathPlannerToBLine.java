package org.frc5010.common.drive.swerve.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Converts a PathPlanner {@code .path} file into a BLine {@link Path}, so PathPlanner-authored
 * autonomous paths can be followed by {@link BLineSwerveAuto} without a PathPlanner runtime.
 *
 * <p>PathPlanner paths are cubic Bézier splines (anchor + control points per waypoint); BLine
 * paths are polylines (straight segments between translation targets) with optional rotation
 * and event elements. The conversion <b>samples</b> each Bézier segment into
 * {@link #DEFAULT_SAMPLES_PER_SEGMENT} straight sub-segments so the BLine polyline follows the
 * spline's curve shape — important near field obstacles where the curve bows away from the
 * straight chord.
 *
 * <p>The following PathPlanner fields are translated:
 * <ul>
 *   <li><b>waypoints</b> → sampled {@link TranslationTarget}s; the first and last become
 *       {@link Waypoint}s carrying the path's start ({@code idealStartingState.rotation}) and
 *       end ({@code goalEndState.rotation}) holonomic headings.</li>
 *   <li><b>rotationTargets</b> → {@link RotationTarget}s, placed on the sub-segment containing
 *       the target's {@code waypointRelativePos} with the local {@code t_ratio}.</li>
 *   <li><b>eventMarkers</b> (point markers) → {@link EventTrigger}s keyed by marker name; register
 *       a command for the name via {@link frc.robot.lib.BLine.FollowPath#registerEventTrigger}.</li>
 * </ul>
 *
 * <p>Not translated (BLine has no equivalent, or it is robot-tuning rather than geometry):
 * constraint zones, point-towards zones, per-path global constraints (BLine uses its own default
 * global constraints — see {@link BLineSwerveAuto}), and {@code reversed} (holonomic drives do
 * not reverse). Zone-style event markers ({@code endWaypointRelativePos != null}) fire once at
 * their start position.
 */
public final class PathPlannerToBLine {

  /** Default number of straight sub-segments each Bézier segment is sampled into. */
  public static final int DEFAULT_SAMPLES_PER_SEGMENT = 12;

  /**
   * Loads {@code deploy/pathplanner/paths/<pathName>.path} and converts it to a BLine path with
   * the default Bézier sampling density.
   *
   * @param pathName the PathPlanner path file name (without extension)
   * @return the converted BLine path
   */
  public static Path load(String pathName) {
    return load(pathName, DEFAULT_SAMPLES_PER_SEGMENT);
  }

  /**
   * Loads {@code deploy/pathplanner/paths/<pathName>.path} and converts it to a BLine path.
   *
   * @param pathName          the PathPlanner path file name (without extension)
   * @param samplesPerSegment straight sub-segments per Bézier segment (≥ 1; higher = closer curve fit)
   * @return the converted BLine path
   */
  public static Path load(String pathName, int samplesPerSegment) {
    File file =
        new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path");
    try (FileReader reader = new FileReader(file)) {
      JSONObject json = (JSONObject) new JSONParser().parse(reader);
      return convert(json, samplesPerSegment);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load PathPlanner path '" + pathName + "'", e);
    }
  }

  /** A path element to weave into the polyline after a given base sample index. */
  private record Insertion(int afterSample, double localT, PathElement element) {}

  /**
   * Converts a parsed PathPlanner path JSON object into a BLine path.
   *
   * @param json              the parsed {@code .path} JSON root
   * @param samplesPerSegment straight sub-segments per Bézier segment (clamped to ≥ 1)
   * @return the converted BLine path
   */
  static Path convert(JSONObject json, int samplesPerSegment) {
    int n = Math.max(1, samplesPerSegment);
    JSONArray waypoints = (JSONArray) json.get("waypoints");
    int numWp = waypoints.size();
    if (numWp < 2) {
      throw new IllegalArgumentException("PathPlanner path needs at least 2 waypoints");
    }
    int numSeg = numWp - 1;

    // Sample the Bézier spline. Point index for (segment seg, sub-step k) is seg*n + k, so the
    // shared anchor between segments lands at a single index (seg's k=n == next seg's k=0).
    List<Translation2d> points = new ArrayList<>();
    for (int seg = 0; seg < numSeg; seg++) {
      Translation2d p0 = anchor(waypoints, seg);
      Translation2d p1 = control(waypoints, seg, "nextControl", p0);
      Translation2d p3 = anchor(waypoints, seg + 1);
      Translation2d p2 = control(waypoints, seg + 1, "prevControl", p3);
      int kStart = (seg == 0) ? 0 : 1; // skip the duplicate shared anchor
      for (int k = kStart; k <= n; k++) {
        points.add(cubic(p0, p1, p2, p3, (double) k / n));
      }
    }

    double startRot = rotationDeg(json, "idealStartingState");
    double endRot = rotationDeg(json, "goalEndState");

    // Base polyline: endpoints are Waypoints carrying the start/end holonomic heading.
    List<PathElement> elements = new ArrayList<>();
    int last = points.size() - 1;
    for (int i = 0; i <= last; i++) {
      if (i == 0) {
        elements.add(new Waypoint(points.get(i), Rotation2d.fromDegrees(startRot)));
      } else if (i == last) {
        elements.add(new Waypoint(points.get(i), Rotation2d.fromDegrees(endRot)));
      } else {
        elements.add(new TranslationTarget(points.get(i)));
      }
    }

    // Weave rotation targets and event markers onto the sub-segment that contains them.
    List<Insertion> insertions = new ArrayList<>();
    JSONArray rotationTargets = (JSONArray) json.get("rotationTargets");
    if (rotationTargets != null) {
      for (Object o : rotationTargets) {
        JSONObject rt = (JSONObject) o;
        double[] at = locate(num(rt.get("waypointRelativePos")), numSeg, n);
        double rotDeg = num(rt.get("rotationDegrees"));
        insertions.add(
            new Insertion(
                (int) at[0], at[1], new RotationTarget(Rotation2d.fromDegrees(rotDeg), at[1])));
      }
    }
    JSONArray eventMarkers = (JSONArray) json.get("eventMarkers");
    if (eventMarkers != null) {
      for (Object o : eventMarkers) {
        JSONObject m = (JSONObject) o;
        String name = (String) m.get("name");
        if (name == null || name.isEmpty()) {
          continue;
        }
        double[] at = locate(num(m.get("waypointRelativePos")), numSeg, n);
        insertions.add(new Insertion((int) at[0], at[1], new EventTrigger(at[1], name)));
      }
    }

    // Insert back-to-front so earlier indices stay valid; for a shared base sample, higher
    // local-t first so the final order is ascending in t within the sub-segment.
    insertions.sort(
        Comparator.comparingInt(Insertion::afterSample)
            .thenComparingDouble(Insertion::localT)
            .reversed());
    for (Insertion ins : insertions) {
      int insertAt = Math.min(ins.afterSample + 1, elements.size());
      elements.add(insertAt, ins.element);
    }

    return new Path(elements);
  }

  /**
   * Maps a PathPlanner {@code waypointRelativePos} (segment index + Bézier fraction) to the
   * sampled sub-segment that contains it.
   *
   * @return {@code [baseSampleIndex, localTRatio]} — insert after {@code baseSampleIndex}, with
   *     {@code localTRatio} the position along that sub-segment (0–1)
   */
  private static double[] locate(double relPos, int numSeg, int n) {
    double r = Math.max(0.0, Math.min(numSeg, relPos));
    int seg = Math.min((int) Math.floor(r), numSeg - 1);
    double fracT = r - seg; // Bézier parameter within the PathPlanner segment
    double global = seg * n + fracT * n; // continuous sample-space position
    int base = (int) Math.floor(global);
    int maxBase = numSeg * n - 1; // last insertable sub-segment start
    base = Math.max(0, Math.min(base, maxBase));
    double localT = Math.max(0.0, Math.min(1.0, global - base));
    return new double[] {base, localT};
  }

  /** Cubic Bézier interpolation at parameter {@code t} ∈ [0, 1]. */
  private static Translation2d cubic(
      Translation2d p0, Translation2d p1, Translation2d p2, Translation2d p3, double t) {
    double u = 1.0 - t;
    double a = u * u * u;
    double b = 3 * u * u * t;
    double c = 3 * u * t * t;
    double d = t * t * t;
    return new Translation2d(
        a * p0.getX() + b * p1.getX() + c * p2.getX() + d * p3.getX(),
        a * p0.getY() + b * p1.getY() + c * p2.getY() + d * p3.getY());
  }

  private static Translation2d anchor(JSONArray waypoints, int index) {
    JSONObject anchor = (JSONObject) ((JSONObject) waypoints.get(index)).get("anchor");
    return new Translation2d(num(anchor.get("x")), num(anchor.get("y")));
  }

  /** A waypoint control point, or the anchor itself when that control is null (path endpoints). */
  private static Translation2d control(JSONArray waypoints, int index, String key, Translation2d fallback) {
    Object c = ((JSONObject) waypoints.get(index)).get(key);
    if (c == null) {
      return fallback;
    }
    JSONObject ctrl = (JSONObject) c;
    return new Translation2d(num(ctrl.get("x")), num(ctrl.get("y")));
  }

  private static double rotationDeg(JSONObject json, String stateKey) {
    JSONObject state = (JSONObject) json.get(stateKey);
    return state == null ? 0.0 : num(state.get("rotation"));
  }

  /** json.simple yields {@code Long} or {@code Double} for numbers; normalize to double. */
  private static double num(Object o) {
    return ((Number) o).doubleValue();
  }

  private PathPlannerToBLine() {}
}
