package org.frc5010.examples;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.util.Color;
import java.util.HashSet;
import java.util.Set;
import org.frc5010.common.util.SimTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Layer 2 — unit tests for the {@link DemoLeds} state → pattern mapping.
 *
 * <p>Drives the robot state (enable/disable, alliance station, pose, intake flag, shot
 * notification) through the HAL sim and stubbed suppliers, calls {@code periodic()}
 * directly, and asserts the rendered colours per segment. Strip layout: left side 0–9,
 * middle 10–19, right side 20–29.
 */
class DemoLedsTest extends SimTestBase {

  /** In the Blue alliance zone (x &lt; 3.952 m). */
  private static final Pose2d IN_BLUE_ZONE = new Pose2d(1.0, 4.0, Rotation2d.kZero);
  /** Mid-field — outside either alliance zone. */
  private static final Pose2d MID_FIELD = new Pose2d(8.0, 4.0, Rotation2d.kZero);

  private DemoLeds leds;
  private Pose2d pose = Pose2d.kZero;
  private boolean intakeExtended = false;

  @BeforeEach
  @Override
  public void simSetup() {
    super.simSetup();
    pose = Pose2d.kZero;
    intakeExtended = false;
    leds = new DemoLeds(9, () -> pose, () -> intakeExtended);
  }

  @AfterEach
  @Override
  public void simTeardown() {
    leds.close();
    // Restore the HAL-sim default so alliance state doesn't leak into other tests.
    setAlliance(AllianceStationID.Unknown);
    super.simTeardown();
  }

  private void setAlliance(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private void assertSidesAll(Color expected, String message) {
    for (int i = 0; i < 10; i++) {
      assertEquals(expected, leds.getColor(i), message + " (left LED " + i + ")");
    }
    for (int i = 20; i < 30; i++) {
      assertEquals(expected, leds.getColor(i), message + " (right LED " + i + ")");
    }
  }

  // ── startup (disabled, never enabled) ──────────────────────────────────────

  @Test
  void startupShowsGreenWhileAllianceUnknown() {
    setAlliance(AllianceStationID.Unknown);
    leds.periodic();
    for (int i = 0; i < DemoLeds.LENGTH; i++) {
      assertEquals(Color.kGreen, leds.getColor(i),
          "whole strip should default to green before the alliance is known, LED " + i);
    }
  }

  @Test
  void startupShowsAllianceColorOnceDetected() {
    setAlliance(AllianceStationID.Red1);
    leds.periodic();
    for (int i = 0; i < DemoLeds.LENGTH; i++) {
      assertEquals(Color.kRed, leds.getColor(i),
          "whole strip should show red once the Red alliance is detected, LED " + i);
    }

    // Alliance is re-read every cycle, so a late DS connection updates the strip.
    setAlliance(AllianceStationID.Blue1);
    leds.periodic();
    for (int i = 0; i < DemoLeds.LENGTH; i++) {
      assertEquals(Color.kBlue, leds.getColor(i),
          "whole strip should follow an alliance change while disabled, LED " + i);
    }
  }

  // ── disabled after having been enabled ─────────────────────────────────────

  @Test
  void disabledAfterEnableShowsRainbowAcrossWholeStrip() {
    setAlliance(AllianceStationID.Blue1);
    enableTeleop();
    leds.periodic(); // latches everEnabled
    disable();
    leds.periodic();

    Set<String> distinct = new HashSet<>();
    for (int i = 0; i < DemoLeds.LENGTH; i++) {
      distinct.add(leds.getColor(i).toHexString());
    }
    assertTrue(distinct.size() >= 5,
        "rainbow should render many distinct colours across the strip, saw " + distinct.size());
  }

  // ── middle segment: intake state ───────────────────────────────────────────

  @Test
  void intakeDeployedShowsLarsonInMiddle() {
    setAlliance(AllianceStationID.Blue1);
    enableTeleop();
    intakeExtended = true;
    leds.periodic();

    int lit = 0;
    int dark = 0;
    for (int i = 10; i < 20; i++) {
      Color c = leds.getColor(i);
      if (c.equals(Color.kBlack)) {
        dark++;
      } else {
        assertTrue(c.red > 0 && c.green == 0 && c.blue == 0,
            "Larson pixels must be red, LED " + i + " was " + c.toHexString());
        lit++;
      }
    }
    assertTrue(lit >= 1, "Larson eye should light at least one middle LED");
    assertTrue(dark >= 1, "Larson scanner should leave LEDs outside the eye dark");

    // Sides are unaffected by the intake: solid alliance colour while not shooting.
    assertSidesAll(Color.kBlue, "sides should stay solid alliance colour");
  }

  @Test
  void intakeRetractedShowsSolidGreenInMiddle() {
    setAlliance(AllianceStationID.Blue1);
    enableTeleop();
    intakeExtended = false;
    leds.periodic();

    for (int i = 10; i < 20; i++) {
      assertEquals(Color.kGreen, leds.getColor(i),
          "middle should be solid green with the intake retracted, LED " + i);
    }
  }

  // ── side segments: shooting state ──────────────────────────────────────────

  @Test
  void notShootingSidesShowSolidAllianceColor() {
    setAlliance(AllianceStationID.Red1);
    enableTeleop();
    leds.periodic();
    assertSidesAll(Color.kRed, "sides should be solid alliance colour when not shooting");
  }

  @Test
  void shootingInAllianceZoneFiresAllianceColorLaser() {
    setAlliance(AllianceStationID.Blue1);
    enableTeleop();
    pose = IN_BLUE_ZONE;
    leds.notifyShot();

    boolean sawBolt = scanSidesForLaser(c ->
        c.blue > 0 && c.red == 0 && c.green == 0);
    assertTrue(sawBolt, "in-zone shot should render a blue (alliance) laser bolt on the sides");
  }

  @Test
  void shootingOutsideZoneFiresGreenLaser() {
    setAlliance(AllianceStationID.Blue1);
    enableTeleop();
    pose = MID_FIELD;
    leds.notifyShot();

    boolean sawBolt = scanSidesForLaser(c ->
        c.green > 0 && c.red == 0 && c.blue == 0);
    assertTrue(sawBolt, "out-of-zone shot should render a green laser bolt on the sides");
  }

  @Test
  void shotEffectExpiresBackToSolidAllianceColor() {
    setAlliance(AllianceStationID.Blue1);
    enableTeleop();
    pose = IN_BLUE_ZONE;
    leds.notifyShot();
    leds.periodic();

    runFor(1.2); // shot effect lasts 1.0 s
    leds.periodic();
    assertSidesAll(Color.kBlue, "sides should return to solid alliance colour after the shot");
  }

  /**
   * Runs several animation cycles and checks every lit side pixel against the expected
   * colour predicate. Returns whether any lit pixel was seen (the bolt briefly leaves the
   * segment entirely dark while it travels off the end, so a single-cycle check is flaky).
   */
  private boolean scanSidesForLaser(java.util.function.Predicate<Color> expectedChannels) {
    boolean sawBolt = false;
    for (int cycle = 0; cycle < 10; cycle++) {
      leds.periodic();
      for (int i = 0; i < 30; i++) {
        if (i >= 10 && i < 20) continue; // middle segment is not part of the laser
        Color c = leds.getColor(i);
        if (c.equals(Color.kBlack)) continue;
        assertTrue(expectedChannels.test(c),
            "laser pixel has wrong colour channels at LED " + i + ": " + c.toHexString());
        sawBolt = true;
      }
      stepOneCycle();
    }
    return sawBolt;
  }
}
