package org.frc5010.common.subsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import org.frc5010.common.leds.LedStripSegments;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/**
 * Layer 2 — unit tests for {@link LedStripSegments}.
 *
 * <p>Renders patterns into the strip's buffer via {@code periodic()} and asserts the
 * resulting colours directly through {@link LedStripSegments#getColor(int)}. No physics
 * or scheduler involvement; the AddressableLED runs against the HAL sim.
 */
class LedStripSegmentsTest extends SimTestBase {

  private static final int LENGTH = 30;

  private LedStripSegments strip;

  @AfterEach
  @Override
  public void simTeardown() {
    // Free the PWM port so later tests (and RobotContainer constructions) can
    // create their own AddressableLED.
    if (strip != null) strip.close();
    super.simTeardown();
  }

  @Test
  void segmentsRenderIndependently() {
    strip = new LedStripSegments(9, LENGTH);
    LedStripSegments.Segment lower = strip.addSegment(0, 14);
    LedStripSegments.Segment upper = strip.addSegment(15, 29);

    lower.setPattern(LEDPattern.solid(Color.kRed));
    upper.setPattern(LEDPattern.solid(Color.kBlue));
    strip.periodic();

    for (int i = 0; i < 15; i++) {
      assertEquals(Color.kRed, strip.getColor(i), "lower segment LED " + i + " should be red");
    }
    for (int i = 15; i < 30; i++) {
      assertEquals(Color.kBlue, strip.getColor(i), "upper segment LED " + i + " should be blue");
    }
  }

  @Test
  void segmentsStartOff() {
    strip = new LedStripSegments(9, LENGTH);
    strip.addSegment(0, 29);
    strip.periodic();

    for (int i = 0; i < LENGTH; i++) {
      assertEquals(Color.kBlack, strip.getColor(i), "unset segment LED " + i + " should be off");
    }
  }

  @Test
  void overrideCoversWholeStripThenSegmentsResume() {
    strip = new LedStripSegments(9, LENGTH);
    LedStripSegments.Segment lower = strip.addSegment(0, 14);
    LedStripSegments.Segment upper = strip.addSegment(15, 29);
    lower.setPattern(LEDPattern.solid(Color.kRed));
    upper.setPattern(LEDPattern.solid(Color.kBlue));

    strip.setOverride(LEDPattern.solid(Color.kYellow));
    strip.periodic();
    for (int i = 0; i < LENGTH; i++) {
      assertEquals(Color.kYellow, strip.getColor(i), "override should cover LED " + i);
    }

    strip.clearOverride();
    strip.periodic();
    assertEquals(Color.kRed, strip.getColor(0), "lower segment should resume after override");
    assertEquals(Color.kBlue, strip.getColor(29), "upper segment should resume after override");
  }

  @Test
  void reversedSegmentRendersBackToFront() {
    strip = new LedStripSegments(9, LENGTH);
    LedStripSegments.Segment reversed = strip.addSegment(0, 9, true);

    // Pattern lights only its local index 0; on a reversed view that is the segment's
    // last physical LED.
    reversed.setPattern((reader, writer) -> {
      for (int i = 0; i < reader.getLength(); i++) {
        writer.setLED(i, i == 0 ? Color.kWhite : Color.kBlack);
      }
    });
    strip.periodic();

    assertEquals(Color.kWhite, strip.getColor(9), "local index 0 should map to physical LED 9");
    for (int i = 0; i < 9; i++) {
      assertEquals(Color.kBlack, strip.getColor(i), "physical LED " + i + " should be off");
    }
  }
}
