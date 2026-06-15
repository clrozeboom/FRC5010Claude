package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.BooleanSupplier;
import org.frc5010.common.leds.LedAnimations;
import org.frc5010.common.leds.LedStripSegments;

/**
 * Status lights for the Fuel handler: a 30-LED strip split into one 10-LED segment per device.
 *
 * <p>Built on the library {@link LedStripSegments}, which divides one physical strip into
 * independently animated {@link Segment}s. Each cycle this subsystem reads the device states
 * (passed in as small {@link BooleanSupplier} lambdas) and chooses a pattern for each segment:
 *
 * <ul>
 *   <li><b>Arm / intake</b> — solid green when stowed, a red Larson "scanner" while deployed.</li>
 *   <li><b>Roller</b> — off when idle, solid orange while spinning.</li>
 *   <li><b>Shooter</b> — off when idle, yellow while spinning up, green when at speed, and a
 *       laser-bolt flash for a moment after each shot.</li>
 * </ul>
 *
 * <p>While the robot is disabled the whole strip shows a single status colour (a rainbow once
 * it has been enabled, solid green before that).
 */
public class StatusLeds extends LedStripSegments {

  private static final int LENGTH = 30;
  private static final int SEG = 10;
  private static final double SHOT_FLASH_SECS = 1.0;

  private static final LEDPattern OFF = LEDPattern.kOff;
  private static final LEDPattern GREEN = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern ORANGE = LEDPattern.solid(Color.kOrange);
  private static final LEDPattern YELLOW = LEDPattern.solid(Color.kYellow);
  private static final LEDPattern RAINBOW =
      LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(0.5));
  private static final LEDPattern ARM_DEPLOYED =
      LedAnimations.larson(Color.kRed, Seconds.of(1.0), 3.0);
  private static final LEDPattern SHOT_FLASH =
      LedAnimations.laser(Color.kWhite, Seconds.of(0.3), 4);

  private final Segment armSeg;
  private final Segment rollerSeg;
  private final Segment shooterSeg;

  private final BooleanSupplier deployed;
  private final BooleanSupplier rollerSpinning;
  private final BooleanSupplier shooterSpinning;
  private final BooleanSupplier shooterAtSpeed;

  private boolean everEnabled = false;
  private double flashUntil = 0.0;

  /**
   * @param pwmPort         roboRIO PWM header the strip is plugged into
   * @param deployed        whether the intake arm is deployed
   * @param rollerSpinning  whether the intake roller is spinning
   * @param shooterSpinning whether the shooter is spinning
   * @param shooterAtSpeed  whether the shooter has reached firing speed
   */
  public StatusLeds(
      int pwmPort,
      BooleanSupplier deployed,
      BooleanSupplier rollerSpinning,
      BooleanSupplier shooterSpinning,
      BooleanSupplier shooterAtSpeed) {
    super(pwmPort, LENGTH);
    this.deployed = deployed;
    this.rollerSpinning = rollerSpinning;
    this.shooterSpinning = shooterSpinning;
    this.shooterAtSpeed = shooterAtSpeed;
    armSeg = addSegment(0, SEG - 1);
    rollerSeg = addSegment(SEG, 2 * SEG - 1);
    shooterSeg = addSegment(2 * SEG, 3 * SEG - 1);
  }

  /** Start the shooter "shot" flash. Call this when you fire. */
  public void notifyShot() {
    flashUntil = Timer.getFPGATimestamp() + SHOT_FLASH_SECS;
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      everEnabled = true;
    }

    if (DriverStation.isDisabled()) {
      setOverride(everEnabled ? RAINBOW : GREEN);
    } else {
      clearOverride();
      armSeg.setPattern(deployed.getAsBoolean() ? ARM_DEPLOYED : GREEN);
      rollerSeg.setPattern(rollerSpinning.getAsBoolean() ? ORANGE : OFF);
      shooterSeg.setPattern(shooterPattern());
    }
    super.periodic(); // renders the chosen patterns to the strip
  }

  private LEDPattern shooterPattern() {
    if (Timer.getFPGATimestamp() < flashUntil) {
      return SHOT_FLASH;
    }
    if (shooterAtSpeed.getAsBoolean()) {
      return GREEN;
    }
    return shooterSpinning.getAsBoolean() ? YELLOW : OFF;
  }
}
