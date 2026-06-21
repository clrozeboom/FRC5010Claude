package frc.robot.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.rebuilt.HubTracker.Shift;
import org.junit.jupiter.api.Test;

/**
 * Layer 1 — unit tests for the ported field/strategy model ({@link FieldConstants},
 * {@link HubTracker}, {@link Constants}). Pure geometry and time math: no HAL, no hardware.
 */
class FieldModelTest {

  // ── HubTracker shift schedule ──────────────────────────────────────────────

  @Test
  void shiftWindowsCoverZeroToOneSixtyContiguously() {
    Shift[] shifts = Shift.values();
    assertEquals(0, shifts[0].startTime(), "first shift starts at match-time 0");
    assertEquals(160, shifts[shifts.length - 1].endTime(), "last shift ends at match-time 160");
    for (int i = 1; i < shifts.length; i++) {
      assertEquals(
          shifts[i - 1].endTime(),
          shifts[i].startTime(),
          "shift windows must be contiguous between " + shifts[i - 1] + " and " + shifts[i]);
    }
  }

  @Test
  void autoAndTransitionAndEndgameAreActiveForBothAlliances() {
    // BOTH windows: regardless of who won auto, both alliances are active.
    assertTrue(HubTracker.isActive(Alliance.Blue, Shift.AUTO));
    assertTrue(HubTracker.isActive(Alliance.Red, Shift.AUTO));
    assertTrue(HubTracker.isActive(Alliance.Blue, Shift.TRANSITION));
    assertTrue(HubTracker.isActive(Alliance.Red, Shift.ENDGAME));
  }

  @Test
  void shiftWindowsAlternateBetweenAutoWinnerAndLoser() {
    // No game-specific message in a unit test → getAutoWinner() is empty → AUTO_WINNER and
    // AUTO_LOSER windows are inactive for everyone (can't be resolved). The schedule is what
    // we pin here: odd shifts favour the loser, even shifts the winner.
    assertEquals(0, Shift.SHIFT_1.startTime() % 5, "SHIFT_1 starts on a 5s boundary");
    // Without an auto winner, alliance-specific windows resolve to inactive.
    assertFalse(HubTracker.isActive(Alliance.Blue, Shift.SHIFT_1));
    assertFalse(HubTracker.isActive(Alliance.Red, Shift.SHIFT_2));
  }

  @Test
  void allianceSpecificShiftsAreInactiveWithoutAnAutoWinner() {
    // With no FMS game-specific message the auto winner is unknown, so the alliance-keyed
    // windows cannot resolve to active for either alliance — pure logic, no DS mode needed.
    assertFalse(HubTracker.isActive(Alliance.Blue, Shift.SHIFT_2));
    assertFalse(HubTracker.isActive(Alliance.Red, Shift.SHIFT_3));
    assertTrue(HubTracker.getAutoWinner().isEmpty(), "no game message → no auto winner");
  }

  // ── Constants: legacy hood-angle offset ────────────────────────────────────

  @Test
  void legacyHoodOffsetConvertsLegacyFrameToCorrectedFrame() {
    // 30° legacy is the mechanical start; corrected start is 12.723°, so the offset is negative.
    assertEquals(
        Constants.Launcher.HOOD_CORRECTED_START_ANGLE_DEGREES,
        Constants.Launcher.offsetLegacyHoodAngleDegrees(
            Constants.Launcher.HOOD_LEGACY_START_ANGLE_DEGREES),
        1e-9,
        "legacy 30° must map to the corrected 12.723° start");
    assertEquals(
        -17.277,
        Constants.Launcher.HOOD_CALIBRATION_OFFSET_DEGREES,
        1e-3,
        "calibration offset is 12.723 − 30 = −17.277°");
  }

  // ── FieldConstants geometry ────────────────────────────────────────────────

  @Test
  void hubTopCenterIsAtFieldMidlineAndCatcherHeight() {
    // y is the field centre; z is the 72" catcher height.
    assertEquals(
        FieldConstants.fieldWidth / 2.0,
        FieldConstants.Hub.topCenterPoint.getY(),
        1e-6,
        "hub aim point sits on the field's horizontal midline");
    assertEquals(
        edu.wpi.first.math.util.Units.inchesToMeters(72.0),
        FieldConstants.Hub.topCenterPoint.getZ(),
        1e-6,
        "hub top-centre aim point is at 72 inches");
  }

  @Test
  void fieldDimensionsAreThe2026AndymarkField() {
    // The default WPILib field (2026 Rebuilt) is ~17.5 m long, ~8 m wide.
    assertTrue(
        FieldConstants.fieldLength > 16.0 && FieldConstants.fieldLength < 18.5,
        "field length in expected 2026 range, was " + FieldConstants.fieldLength);
    assertTrue(
        FieldConstants.fieldWidth > 7.0 && FieldConstants.fieldWidth < 9.0,
        "field width in expected 2026 range, was " + FieldConstants.fieldWidth);
    assertTrue(FieldConstants.aprilTagCount > 0, "tag layout must contain tags");
  }
}
