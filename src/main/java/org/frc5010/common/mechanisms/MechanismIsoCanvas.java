package org.frc5010.common.mechanisms;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.frc5010.common.mechanisms.MechanismVisuals3d.Segment;

/**
 * Renders the {@link MechanismVisuals3d} segments as a fixed isometric projection on a
 * single {@link Mechanism2d} (published once as <b>SmartDashboard → RobotMechanisms3D</b>),
 * so the same robot-frame 3D view the web UI and AdvantageScope show is also visible in
 * the plain simulator (Glass) — no browser, no orbiting, just a static iso angle.
 *
 * <p>{@link MechanismVisuals3d#publish} drives this every cycle on the robot thread: each
 * 3D segment becomes one {@link MechanismLigament2d} on its own root, with the start point
 * projected to the canvas and the ligament length/angle/color tracing the segment. Roots
 * can't be removed from a {@code Mechanism2d}, so a per-mechanism pool of ligaments is
 * reused across cycles and any now-unused slots are collapsed to zero length.
 *
 * <p>Projection (robot frame: x forward, y left, z up): a classic 30° isometric with z
 * straight up — {@code px = (x − y)·cos30}, {@code py = z + (x + y)·sin30}, scaled and
 * offset to sit inside the canvas. Depth (further forward/left) recedes up-and-right.
 */
final class MechanismIsoCanvas {

  /** Canvas width, meters. */
  static final double WIDTH = 4.0;
  /** Canvas height, meters — kept short so Glass scales the scene up to fill the widget. */
  static final double HEIGHT = 2.6;

  private static final double COS30 = Math.cos(Math.PI / 6);
  private static final double SIN30 = Math.sin(Math.PI / 6);
  private static final double SCALE = 0.8;
  private static final double ORIGIN_X = 1.8;
  private static final double ORIGIN_Y = 0.3;

  private record Slot(MechanismRoot2d root, MechanismLigament2d ligament) {}

  private static volatile boolean enabled = true;
  private static Mechanism2d canvas;
  private static final Map<String, List<Slot>> pools = new HashMap<>();

  private MechanismIsoCanvas() {}

  /** Enables/disables the Glass iso canvas (default enabled). Call before mechanisms publish. */
  static void setEnabled(boolean on) {
    enabled = on;
  }

  /** Projects a robot-frame point to canvas coordinates {@code [px, py]}. */
  static double[] project(Translation3d p) {
    double px = ORIGIN_X + (p.getX() - p.getY()) * COS30 * SCALE;
    double py = ORIGIN_Y + (p.getZ() + (p.getX() + p.getY()) * SIN30) * SCALE;
    return new double[] {px, py};
  }

  /** Draws one mechanism's segments onto the shared iso canvas, reusing its slot pool. */
  static synchronized void render(String name, List<Segment> segments) {
    if (!enabled) {
      return;
    }
    if (canvas == null) {
      canvas = new Mechanism2d(WIDTH, HEIGHT);
      SmartDashboard.putData("RobotMechanisms3D", canvas);
    }
    List<Slot> pool = pools.computeIfAbsent(name, k -> new ArrayList<>());

    for (int i = 0; i < segments.size(); i++) {
      Segment s = segments.get(i);
      double[] a = project(s.start());
      double[] b = project(s.end());
      double dx = b[0] - a[0];
      double dy = b[1] - a[1];
      double length = Math.hypot(dx, dy);
      double angleDeg = Math.toDegrees(Math.atan2(dy, dx));

      Slot slot;
      if (i < pool.size()) {
        slot = pool.get(i);
      } else {
        MechanismRoot2d root = canvas.getRoot(name + "#" + i, a[0], a[1]);
        slot = new Slot(root, root.append(new MechanismLigament2d(name + "#" + i, length, angleDeg)));
        pool.add(slot);
      }
      slot.root().setPosition(a[0], a[1]);
      slot.ligament().setLength(Math.max(length, 1e-4));
      slot.ligament().setAngle(angleDeg);
      slot.ligament().setColor(new Color8Bit(new Color(s.colorHex())));
      slot.ligament().setLineWeight(s.weight());
    }
    // Collapse slots no longer used this cycle (roots can't be removed from a Mechanism2d).
    for (int i = segments.size(); i < pool.size(); i++) {
      pool.get(i).ligament().setLength(0);
    }
  }

  /** Hides a mechanism's segments when it is removed (its roots stay but go to zero length). */
  static synchronized void remove(String name) {
    List<Slot> pool = pools.remove(name);
    if (pool != null) {
      pool.forEach(slot -> slot.ligament().setLength(0));
    }
  }

  /** Drops the canvas and pools so the next render starts fresh. For unit tests. */
  static synchronized void resetForTesting() {
    canvas = null;
    pools.clear();
    enabled = true;
  }

  /** Number of pooled ligaments for a mechanism (its high-water segment count). For tests. */
  static synchronized int slotCount(String name) {
    List<Slot> pool = pools.get(name);
    return pool != null ? pool.size() : 0;
  }

  /** The current drawn length of a mechanism's i-th slot (0 = collapsed/hidden). For tests. */
  static synchronized double ligamentLength(String name, int index) {
    return pools.get(name).get(index).ligament().getLength();
  }
}
