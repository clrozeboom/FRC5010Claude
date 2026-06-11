package org.frc5010.common.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * An addressable LED strip split into independently animated segments.
 *
 * <p>Each {@link Segment} owns a contiguous slice of the strip (an
 * {@link AddressableLEDBufferView}) and a {@link LEDPattern} that is re-applied every
 * {@link #periodic()} cycle, so time-based patterns (scrolling rainbow,
 * {@link LedAnimations#larson Larson scanner}, {@link LedAnimations#laser laser pulse}) animate
 * automatically. Different segments can run different patterns at the same time.
 *
 * <p>A whole-strip <em>override</em> pattern ({@link #setOverride(LEDPattern)}) takes precedence
 * over all segments while set — useful for global states such as "disabled" or "startup" —
 * without disturbing the per-segment patterns, which resume as soon as the override is
 * {@link #clearOverride() cleared}.
 *
 * <p>WPILib supports a single {@link AddressableLED} driver per robot, so create exactly one
 * instance of this class and divide it into as many segments as needed. The class is
 * {@link AutoCloseable} so tests can free the PWM port between robot constructions.
 *
 * <pre>{@code
 * LedStripSegments strip = new LedStripSegments(9, 30);
 * LedStripSegments.Segment left   = strip.addSegment(0, 9, true);  // reversed: animates outward
 * LedStripSegments.Segment middle = strip.addSegment(10, 19);
 * LedStripSegments.Segment right  = strip.addSegment(20, 29);
 * middle.setPattern(LedAnimations.larson(Color.kRed, Seconds.of(1.0), 3.0));
 * }</pre>
 */
public class LedStripSegments extends SubsystemBase implements AutoCloseable {

  /** A contiguous slice of the strip with its own independently set pattern. */
  public static final class Segment {
    private final AddressableLEDBufferView view;
    private LEDPattern pattern = LEDPattern.kOff;

    private Segment(AddressableLEDBufferView view) {
      this.view = view;
    }

    /**
     * Sets the pattern rendered on this segment each cycle (until the next call).
     *
     * @param pattern the pattern; use {@link LEDPattern#kOff} to blank the segment
     */
    public void setPattern(LEDPattern pattern) {
      this.pattern = Objects.requireNonNull(pattern, "pattern");
    }

    /** Number of LEDs in this segment. */
    public int getLength() {
      return view.getLength();
    }
  }

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final List<Segment> segments = new ArrayList<>();
  private LEDPattern override = null;

  /**
   * Creates and starts the strip. All LEDs are off until segments (or an override) are set.
   *
   * <p>When the sim web UI is active ({@code -PwebUI}), the strip auto-binds to
   * {@link org.frc5010.common.sim.WebControl} so the browser renders a live copy of it
   * under the field — same pattern as {@link org.frc5010.common.sim.SimRobotState}.
   *
   * @param pwmPort the roboRIO PWM header the strip's data line is plugged into
   * @param length  total number of LEDs on the strip
   */
  public LedStripSegments(int pwmPort, int length) {
    led = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(length);
    led.setLength(length);
    led.start();
    org.frc5010.common.sim.WebControl.getInstance().ifPresent(wc -> wc.bindLeds(this));
  }

  /**
   * Defines a forward (ascending-index) segment. Segments may not be redefined; define the
   * full layout once at construction time.
   *
   * @param startIndex first LED of the segment (inclusive)
   * @param endIndex   last LED of the segment (inclusive)
   * @return the new segment, initially {@link LEDPattern#kOff}
   */
  public Segment addSegment(int startIndex, int endIndex) {
    return addSegment(startIndex, endIndex, false);
  }

  /**
   * Defines a segment, optionally reversed so directional patterns (laser, scroll) travel
   * from {@code endIndex} toward {@code startIndex}. Use this to make the two halves of a
   * strip animate symmetrically outward from the centre.
   *
   * @param startIndex first LED of the segment (inclusive)
   * @param endIndex   last LED of the segment (inclusive)
   * @param reversed   whether patterns render back-to-front on this segment
   * @return the new segment, initially {@link LEDPattern#kOff}
   */
  public Segment addSegment(int startIndex, int endIndex, boolean reversed) {
    AddressableLEDBufferView view = buffer.createView(startIndex, endIndex);
    Segment segment = new Segment(reversed ? view.reversed() : view);
    segments.add(segment);
    return segment;
  }

  /**
   * Renders the given pattern across the whole strip, taking precedence over every segment
   * until {@link #clearOverride()} is called. Segment patterns are retained and resume
   * afterwards.
   *
   * @param pattern the whole-strip pattern
   */
  public void setOverride(LEDPattern pattern) {
    this.override = Objects.requireNonNull(pattern, "pattern");
  }

  /** Removes the whole-strip override; segments render again from the next cycle. */
  public void clearOverride() {
    this.override = null;
  }

  /** Total number of LEDs on the strip. */
  public int getLength() {
    return buffer.getLength();
  }

  /**
   * Returns the colour most recently rendered at the given strip-global index.
   * Intended for tests and telemetry.
   *
   * @param index LED index on the full strip (0-based)
   */
  public Color getColor(int index) {
    return buffer.getLED(index);
  }

  @Override
  public void periodic() {
    if (override != null) {
      override.applyTo(buffer);
    } else {
      for (Segment segment : segments) {
        segment.pattern.applyTo(segment.view);
      }
    }
    led.setData(buffer);
  }

  /** Frees the PWM port so another strip (e.g. in a later test) can be constructed. */
  @Override
  public void close() {
    led.close();
  }
}
