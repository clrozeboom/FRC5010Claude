package org.frc5010.common.subsystem;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Transform;
import org.dyn4j.world.World;
import org.frc5010.common.drive.swerve.Arena2026Patch;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

/**
 * Verifies {@link Arena2026Patch} injects the red-far TRENCH obstacle that the
 * IronMaple {@code Arena2026Rebuilt} field map omits (copy-paste bug duplicates the
 * blue-near bar) and that the patch is idempotent.
 */
class Arena2026PatchTest extends SimTestBase {

  private static final double RED_FAR_X = 11.9149;
  private static final double RED_FAR_Y = 6.6385;
  private static final double TOL = 0.1;

  @AfterEach
  @Override
  public void simTeardown() {
    SimulatedArena.getInstance().shutDown();
    try {
      Field f = SimulatedArena.class.getDeclaredField("instance");
      f.setAccessible(true);
      f.set(null, null);
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException("Failed to reset SimulatedArena singleton", e);
    }
    super.simTeardown();
  }

  private static int redFarObstacleCount() throws ReflectiveOperationException {
    Field field = SimulatedArena.class.getDeclaredField("physicsWorld");
    field.setAccessible(true);
    @SuppressWarnings("unchecked")
    World<Body> world = (World<Body>) field.get(SimulatedArena.getInstance());
    int count = 0;
    for (Body b : world.getBodies()) {
      Transform t = b.getTransform();
      if (Math.hypot(t.getTranslationX() - RED_FAR_X, t.getTranslationY() - RED_FAR_Y) < TOL) {
        count++;
      }
    }
    return count;
  }

  @Test
  void redFarTrenchMissingBeforePatch() throws ReflectiveOperationException {
    // Fresh Arena2026Rebuilt — the bugged field map has no body at the red-far corner.
    assertEquals(0, redFarObstacleCount(),
        "Arena2026Rebuilt unexpectedly already has a red-far trench; library bug may be fixed");
  }

  @Test
  void patchAddsRedFarTrench() throws ReflectiveOperationException {
    Arena2026Patch.applyMissingRedFarTrench();
    assertEquals(1, redFarObstacleCount(),
        "patch should add exactly one obstacle at the red-far corner");
  }

  @Test
  void patchIsIdempotent() throws ReflectiveOperationException {
    Arena2026Patch.applyMissingRedFarTrench();
    Arena2026Patch.applyMissingRedFarTrench();
    Arena2026Patch.applyMissingRedFarTrench();
    assertEquals(1, redFarObstacleCount(),
        "repeated patch calls must not stack duplicate obstacles");
  }
}
