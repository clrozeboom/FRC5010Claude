package frc.robot.tigershark;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.frc5010.common.leds.LedAnimations;
import org.frc5010.common.leds.LedStripSegments;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/**
 * LED state display for TigerShark. The generic segment/animation engine lives in
 * {@link LedStripSegments}; this class only maps robot states to patterns.
 *
 * <h3>State → pattern</h3>
 * <ul>
 *   <li><b>Disabled at startup</b> — solid alliance colour (green if alliance unknown).</li>
 *   <li><b>Disabled after enabled</b> — scrolling rainbow.</li>
 *   <li><b>Enabled, elevator moving</b> — red Larson scanner.</li>
 *   <li><b>Enabled, elevator idle</b> — solid alliance colour.</li>
 * </ul>
 */
public class TigerSharkLeds extends LedStripSegments {

  /** Number of LEDs on the strip. */
  public static final int LENGTH = 28;

  private static final LEDPattern RAINBOW =
      LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(0.5));
  private static final LEDPattern SOLID_GREEN = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern SOLID_RED = LEDPattern.solid(Color.kRed);
  private static final LEDPattern SOLID_BLUE = LEDPattern.solid(Color.kBlue);
  private static final LEDPattern LARSON =
      LedAnimations.larson(Color.kRed, Seconds.of(1.0), 3.0);

  private final Segment strip;
  private final BooleanSupplier elevatorMoving;
  private boolean everEnabled = false;

  /**
   * @param pwmPort        roboRIO PWM header the strip is plugged into
   * @param elevatorMoving true when the elevator is in motion (drives the Larson pattern)
   */
  public TigerSharkLeds(int pwmPort, BooleanSupplier elevatorMoving) {
    super(pwmPort, LENGTH);
    this.elevatorMoving = elevatorMoving;
    strip = addSegment(0, LENGTH - 1);
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) everEnabled = true;

    if (DriverStation.isDisabled()) {
      setOverride(everEnabled ? RAINBOW : allianceSolid());
    } else {
      clearOverride();
      strip.setPattern(elevatorMoving.getAsBoolean() ? LARSON : allianceSolid());
    }
    super.periodic();
  }

  /** Solid alliance colour, or green while the alliance is unknown. */
  private static LEDPattern allianceSolid() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return SOLID_GREEN;
    return alliance.get() == Alliance.Red ? SOLID_RED : SOLID_BLUE;
  }
}
