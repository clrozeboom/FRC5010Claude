package org.frc5010.common.drive.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.lang.reflect.Field;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.world.World;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.utils.mathutils.GeometryConvertor;

/**
 * Patches a known bug in IronMaple's {@code Arena2026Rebuilt} field map.
 *
 * <p>The 2026 (Rebuilt) field has four 53"&times;12" TRENCH bars, one at each corner
 * around the field centre (8.27, 4.035): blue-near, red-near, blue-far, red-far.
 * The library builds the bars with this offset pattern:
 *
 * <pre>
 *   (8.27 - dx, 4.035 - dy)   blue-near
 *   (8.27 + dx, 4.035 - dy)   red-near
 *   (8.27 - dx, 4.035 + dy)   blue-far
 *   (8.27 - dx, 4.035 - dy)   &lt;-- copy-paste bug: repeats blue-near
 * </pre>
 *
 * <p>The fourth call should read {@code (8.27 + dx, 4.035 + dy)} — the red-far
 * corner at (11.915, 6.639). Because of the duplicate, the physics engine has no
 * collision body there and a robot can drive through where the real-field trench
 * stands. {@link #applyMissingRedFarTrench()} adds that body back so simulation
 * collisions match the real field.
 *
 * <p>The patch is idempotent and self-disabling: it only touches an
 * {@code Arena2026Rebuilt} instance, and it skips if an obstacle already sits at
 * the red-far corner (so it becomes a no-op once the library fixes the bug).
 */
public final class Arena2026Patch {

  private Arena2026Patch() {}

  // 53"x12" trench bar, matching the other three bars in Arena2026Rebuilt.
  private static final double TRENCH_W = Units.inchesToMeters(53);
  private static final double TRENCH_H = Units.inchesToMeters(12);
  // The corner the library bug omits: centre (8.27,4.035) + (dx,dy),
  // dx = (120+23.5)", dy = (73+23.5+6)".
  private static final Pose2d RED_FAR_TRENCH =
      new Pose2d(11.9149, 6.6385, Rotation2d.kZero);
  // Friction/restitution used by FieldMap.createObstacle for every static obstacle.
  private static final double OBSTACLE_FRICTION = 0.6;
  private static final double OBSTACLE_RESTITUTION = 0.3;
  private static final double MATCH_TOLERANCE_M = 0.1;

  /**
   * Adds the missing red-far trench obstacle to the current {@link SimulatedArena}
   * physics world. No-op when not running an {@code Arena2026Rebuilt} field, when the
   * obstacle already exists, or when the world cannot be reached reflectively.
   */
  public static void applyMissingRedFarTrench() {
    SimulatedArena arena = SimulatedArena.getInstance();
    if (!arena.getClass().getName().contains("Arena2026Rebuilt")) return;

    World<Body> world = physicsWorld(arena);
    if (world == null) return;

    for (Body b : world.getBodies()) {
      Transform t = b.getTransform();
      if (Math.hypot(t.getTranslationX() - RED_FAR_TRENCH.getX(),
                     t.getTranslationY() - RED_FAR_TRENCH.getY()) < MATCH_TOLERANCE_M) {
        return; // already present — patch already applied, or library bug fixed upstream
      }
    }

    Body body = new Body();
    body.setMass(MassType.INFINITE);
    BodyFixture fixture = body.addFixture(Geometry.createRectangle(TRENCH_W, TRENCH_H));
    fixture.setFriction(OBSTACLE_FRICTION);
    fixture.setRestitution(OBSTACLE_RESTITUTION);
    body.getTransform().set(GeometryConvertor.toDyn4jTransform(RED_FAR_TRENCH));
    world.addBody(body);

    System.out.println("[Arena2026Patch] Added missing red-far TRENCH obstacle at "
        + "(11.915, 6.639) — patch over an Arena2026Rebuilt copy-paste bug "
        + "that duplicated the blue-near bar.");
  }

  @SuppressWarnings("unchecked")
  private static World<Body> physicsWorld(SimulatedArena arena) {
    try {
      Field field = SimulatedArena.class.getDeclaredField("physicsWorld");
      field.setAccessible(true);
      return (World<Body>) field.get(arena);
    } catch (ReflectiveOperationException e) {
      System.err.println("[Arena2026Patch] could not access physicsWorld; "
          + "skipping trench patch: " + e);
      return null;
    }
  }
}
