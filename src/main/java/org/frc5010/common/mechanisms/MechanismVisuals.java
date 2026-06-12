package org.frc5010.common.mechanisms;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Shared {@link Mechanism2d} canvas for the mechanism visualizations.
 *
 * <p>By default every mechanism draws onto one robot-side-view canvas (published once
 * as <b>SmartDashboard → RobotMechanisms</b>), rooted at its
 * {@code settings.visualPosition} — x along the robot's length, y above the floor,
 * meters. That gives a single overlay of the whole superstructure in Glass /
 * AdvantageScope. To split mechanisms onto separate widgets instead, pass your own
 * canvas via {@code settings.mechanism2d} (the caller publishes custom canvases;
 * the library only publishes the shared default).
 *
 * <p>Root names on the canvas come from each mechanism's {@code settings.name}, so
 * names must be unique per robot — they already must be for telemetry/tuning tables.
 */
public final class MechanismVisuals {

  /** Default canvas width, meters (robot length axis of the side view). */
  public static final double DEFAULT_WIDTH = 3.0;
  /** Default canvas height, meters (above the floor). */
  public static final double DEFAULT_HEIGHT = 2.5;

  private static Mechanism2d defaultCanvas;

  private MechanismVisuals() {}

  /**
   * Resolves the canvas a mechanism should draw on.
   *
   * @param custom the canvas from settings, or null for the shared default
   * @return the canvas to use
   */
  static synchronized Mechanism2d canvasFor(Mechanism2d custom) {
    if (custom != null) {
      return custom;
    }
    if (defaultCanvas == null) {
      defaultCanvas = new Mechanism2d(DEFAULT_WIDTH, DEFAULT_HEIGHT);
      SmartDashboard.putData("RobotMechanisms", defaultCanvas);
    }
    return defaultCanvas;
  }

  /** The shared default canvas (created/published on first use). */
  public static Mechanism2d getDefault() {
    return canvasFor(null);
  }

  /**
   * Drops the shared canvas so the next use creates a fresh one. For unit tests —
   * re-constructing a mechanism with the same name would otherwise collide with the
   * ligaments left on the old canvas.
   */
  public static synchronized void resetForTesting() {
    defaultCanvas = null;
  }
}
