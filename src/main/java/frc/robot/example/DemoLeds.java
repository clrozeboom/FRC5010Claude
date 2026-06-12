package frc.robot.example;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.frc5010.common.leds.LedAnimations;
import org.frc5010.common.leds.LedStripSegments;

/**
 * Example LED state display for the 2026 Rebuilt demo robot — the team-code half of the
 * LED stack. The generic segment/animation engine lives in
 * {@link org.frc5010.common.leds.LedStripSegments}; this class only encodes which robot
 * states map to which patterns.
 *
 * <p>A 30-LED strip split into three 10-LED segments: left side (reversed so directional
 * effects travel outward from the centre), middle, right side.
 *
 * <h3>State → pattern mapping</h3>
 * <ul>
 *   <li><b>Disabled at startup</b> (never yet enabled) — solid alliance colour across the
 *       whole strip once the alliance is known, solid green until then.</li>
 *   <li><b>Disabled after having been enabled</b> — scrolling rainbow across the whole strip.</li>
 *   <li><b>Enabled, middle segment</b> — red Larson (Knight Rider) scanner while the intake
 *       is deployed; solid green when retracted.</li>
 *   <li><b>Enabled, side segments</b> — while shooting (~1 s after each shot), an outward
 *       laser-bolt effect: alliance colour when the robot is inside its alliance zone, green
 *       outside it. Solid alliance colour (green if unknown) when not shooting.</li>
 * </ul>
 */
public class DemoLeds extends LedStripSegments {

  /** Total LEDs on the demo strip. */
  public static final int LENGTH = 30;
  /** LEDs per side segment; the middle segment gets the remainder. */
  private static final int SIDE_LENGTH = 10;
  /** How long the laser effect plays after each shot. */
  private static final double SHOT_EFFECT_SECS = 1.0;

  private static final LEDPattern RAINBOW =
      LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(0.5));
  private static final LEDPattern SOLID_GREEN = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern SOLID_RED = LEDPattern.solid(Color.kRed);
  private static final LEDPattern SOLID_BLUE = LEDPattern.solid(Color.kBlue);
  private static final LEDPattern LARSON =
      LedAnimations.larson(Color.kRed, Seconds.of(1.0), 3.0);
  private static final LEDPattern LASER_GREEN =
      LedAnimations.laser(Color.kGreen, Seconds.of(0.4), 4);
  private static final LEDPattern LASER_RED =
      LedAnimations.laser(Color.kRed, Seconds.of(0.4), 4);
  private static final LEDPattern LASER_BLUE =
      LedAnimations.laser(Color.kBlue, Seconds.of(0.4), 4);

  private final Segment leftSide;
  private final Segment middle;
  private final Segment rightSide;
  private final Supplier<Pose2d> poseSupplier;
  private final BooleanSupplier intakeExtended;

  private boolean everEnabled = false;
  private double shootingUntil = 0.0;

  /**
   * @param pwmPort        roboRIO PWM header the strip is plugged into
   * @param poseSupplier   current robot pose (for the alliance-zone laser colour)
   * @param intakeExtended whether the intake is currently deployed
   */
  public DemoLeds(int pwmPort, Supplier<Pose2d> poseSupplier, BooleanSupplier intakeExtended) {
    super(pwmPort, LENGTH);
    this.poseSupplier = poseSupplier;
    this.intakeExtended = intakeExtended;
    leftSide = addSegment(0, SIDE_LENGTH - 1, true); // reversed: bolts travel centre → end
    middle = addSegment(SIDE_LENGTH, LENGTH - SIDE_LENGTH - 1);
    rightSide = addSegment(LENGTH - SIDE_LENGTH, LENGTH - 1);
  }

  /** Starts the side-segment laser effect for {@value #SHOT_EFFECT_SECS} s. Call when firing. */
  public void notifyShot() {
    shootingUntil = Timer.getFPGATimestamp() + SHOT_EFFECT_SECS;
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) everEnabled = true;

    if (DriverStation.isDisabled()) {
      setOverride(everEnabled ? RAINBOW : allianceSolid());
    } else {
      clearOverride();
      middle.setPattern(intakeExtended.getAsBoolean() ? LARSON : SOLID_GREEN);
      LEDPattern sides = isShooting()
          ? (DemoIntake.isInAllianceZone(poseSupplier.get()) ? allianceLaser() : LASER_GREEN)
          : allianceSolid();
      leftSide.setPattern(sides);
      rightSide.setPattern(sides);
    }
    super.periodic();
  }

  private boolean isShooting() {
    return Timer.getFPGATimestamp() < shootingUntil;
  }

  /** Solid alliance colour, or green while the alliance is unknown. */
  private static LEDPattern allianceSolid() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return SOLID_GREEN;
    return alliance.get() == Alliance.Red ? SOLID_RED : SOLID_BLUE;
  }

  /** Alliance-coloured laser, or green while the alliance is unknown. */
  private static LEDPattern allianceLaser() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return LASER_GREEN;
    return alliance.get() == Alliance.Red ? LASER_RED : LASER_BLUE;
  }
}
