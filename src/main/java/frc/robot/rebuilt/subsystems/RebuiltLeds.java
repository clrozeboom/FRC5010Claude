package frc.robot.rebuilt.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

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
import frc.robot.rebuilt.subsystems.RebuiltLauncher.LauncherState;

/**
 * LED state display for the ported 2026 "Rebuilt" robot — the team-code mapping of robot
 * states to patterns. The generic segment/animation engine lives in
 * {@link LedStripSegments}.
 *
 * <p>State → pattern (ported from the source {@code ConfigConstants.ALL_LEDS} mapping):
 * <ul>
 *   <li><b>Disabled</b> — solid green once the turret has been zeroed, otherwise the alliance
 *       colour; a turret-zero "burst" plays a brief rainbow.</li>
 *   <li><b>Launcher PREP</b> or <b>indexer FEED</b> — scrolling rainbow (firing).</li>
 *   <li><b>Launcher LOW_SPEED</b> — solid green (ready to aim).</li>
 *   <li><b>Otherwise enabled</b> — middle segment red Larson while the intake is deployed,
 *       sides solid alliance colour.</li>
 * </ul>
 */
public class RebuiltLeds extends LedStripSegments {

  public static final int LENGTH = 30;
  private static final int SIDE_LENGTH = 10;
  private static final double ZERO_BURST_SECS = 1.0;

  private static final LEDPattern RAINBOW =
      LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(0.5));
  private static final LEDPattern SOLID_GREEN = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern SOLID_RED = LEDPattern.solid(Color.kRed);
  private static final LEDPattern SOLID_BLUE = LEDPattern.solid(Color.kBlue);
  private static final LEDPattern LARSON = LedAnimations.larson(Color.kRed, Seconds.of(1.0), 3.0);

  private final Segment leftSide;
  private final Segment middle;
  private final Segment rightSide;

  private final Supplier<LauncherState> launcherState;
  private final BooleanSupplier indexerFeeding;
  private final BooleanSupplier intakeExtended;
  private final BooleanSupplier turretZeroed;

  private double burstUntil = 0.0;

  public RebuiltLeds(
      int pwmPort,
      Supplier<LauncherState> launcherState,
      BooleanSupplier indexerFeeding,
      BooleanSupplier intakeExtended,
      BooleanSupplier turretZeroed) {
    super(pwmPort, LENGTH);
    this.launcherState = launcherState;
    this.indexerFeeding = indexerFeeding;
    this.intakeExtended = intakeExtended;
    this.turretZeroed = turretZeroed;
    leftSide = addSegment(0, SIDE_LENGTH - 1, true);
    middle = addSegment(SIDE_LENGTH, LENGTH - SIDE_LENGTH - 1);
    rightSide = addSegment(LENGTH - SIDE_LENGTH, LENGTH - 1);
  }

  /** Plays a brief rainbow burst — call when the turret is zeroed (operator Start). */
  public void notifyTurretZeroed() {
    burstUntil = Timer.getFPGATimestamp() + ZERO_BURST_SECS;
  }

  @Override
  public void periodic() {
    boolean burst = Timer.getFPGATimestamp() < burstUntil;
    LauncherState ls = launcherState.get();

    if (DriverStation.isDisabled()) {
      setOverride(burst ? RAINBOW : (turretZeroed.getAsBoolean() ? SOLID_GREEN : allianceSolid()));
    } else if (burst || ls == LauncherState.PREP || indexerFeeding.getAsBoolean()) {
      setOverride(RAINBOW);
    } else if (ls == LauncherState.LOW_SPEED) {
      setOverride(SOLID_GREEN);
    } else {
      clearOverride();
      middle.setPattern(intakeExtended.getAsBoolean() ? LARSON : SOLID_GREEN);
      leftSide.setPattern(allianceSolid());
      rightSide.setPattern(allianceSolid());
    }
    super.periodic();
  }

  private static LEDPattern allianceSolid() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return SOLID_GREEN;
    return alliance.get() == Alliance.Red ? SOLID_RED : SOLID_BLUE;
  }
}
