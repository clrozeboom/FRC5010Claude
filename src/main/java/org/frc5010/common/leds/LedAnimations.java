package org.frc5010.common.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Time-based {@link LEDPattern} animations that WPILib doesn't ship.
 *
 * <p>All patterns are pure functions of the FPGA timestamp, so they are stateless,
 * shareable between segments, deterministic under the stepped sim clock used by the
 * test suite, and animate simply by being re-applied every robot loop (which
 * {@link LedStripSegments#periodic()} does).
 */
public final class LedAnimations {

  private LedAnimations() {}

  /**
   * A Larson scanner (Knight Rider / Cylon): a bright "eye" that sweeps back and forth
   * across the segment with a linear falloff tail on both sides.
   *
   * @param color      eye colour at full brightness
   * @param period     time for one full sweep (end → end → back)
   * @param tailLength falloff distance in LEDs; pixels further than this from the eye are off
   * @return the pattern
   */
  public static LEDPattern larson(Color color, Time period, double tailLength) {
    double periodSecs = period.in(Seconds);
    return (reader, writer) -> {
      int length = reader.getLength();
      if (length == 0) return;
      double t = (Timer.getFPGATimestamp() % periodSecs) / periodSecs; // 0..1 over one sweep
      double phase = t * 2.0; // 0..1 forward, 1..2 backward
      double eye = (phase <= 1.0 ? phase : 2.0 - phase) * (length - 1);
      for (int i = 0; i < length; i++) {
        double brightness = Math.max(0.0, 1.0 - Math.abs(i - eye) / tailLength);
        writer.setLED(i, dim(color, brightness));
      }
    };
  }

  /**
   * A laser bolt: a bright pulse with a fading tail that repeatedly travels from the start
   * of the segment off its far end, leaving the rest of the segment dark. Apply to a
   * {@link LedStripSegments#addSegment(int, int, boolean) reversed} segment to flip the
   * travel direction.
   *
   * @param color       bolt colour at full brightness (the leading pixel)
   * @param period      time for one bolt to traverse the segment
   * @param pulseLength bolt length in LEDs, including the fading tail
   * @return the pattern
   */
  public static LEDPattern laser(Color color, Time period, int pulseLength) {
    double periodSecs = period.in(Seconds);
    return (reader, writer) -> {
      int length = reader.getLength();
      if (length == 0) return;
      double t = (Timer.getFPGATimestamp() % periodSecs) / periodSecs; // 0..1 per bolt
      // Head runs past the end by pulseLength so the tail fully exits before the bolt repeats.
      double head = t * (length + pulseLength);
      for (int i = 0; i < length; i++) {
        double behind = head - i;
        double brightness =
            (behind >= 0.0 && behind < pulseLength) ? 1.0 - behind / pulseLength : 0.0;
        writer.setLED(i, dim(color, brightness));
      }
    };
  }

  private static Color dim(Color color, double brightness) {
    return new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
  }
}
