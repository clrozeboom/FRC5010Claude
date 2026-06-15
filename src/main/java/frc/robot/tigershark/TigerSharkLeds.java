package frc.robot.tigershark;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import org.frc5010.common.leds.LedAnimations;
import org.frc5010.common.leds.LedStripSegments;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class TigerSharkLeds extends LedStripSegments {
    private static final LEDPattern RAINBOW = LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(0.5));
    protected static final LEDPattern SOLID_GREEN = LEDPattern.solid(Color.kGreen);
    protected static final LEDPattern SOLID_YELLOW = LEDPattern.solid(Color.kYellow);
    private static final LEDPattern LARSON = LedAnimations.larson(Color.kRed, Seconds.of(1.0), 3.0);

    protected final Segment rightSide;
    private final BooleanSupplier moving, nearTop, nearBottom;

    private boolean everEnabled = false;

    public TigerSharkLeds(BooleanSupplier moving, BooleanSupplier nearTop, BooleanSupplier nearBottom) {
        super(1, 28);
        this.moving = moving;
        this.nearTop = nearTop;
        this.nearBottom = nearBottom;
        rightSide = addSegment(0, 27);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) everEnabled = true;

        if (DriverStation.isDisabled()) {
            setOverride(everEnabled ? RAINBOW : SOLID_GREEN);
        } else {
            clearOverride();
            if (moving.getAsBoolean()) {
                rightSide.setPattern(LARSON);
            } else if (nearTop.getAsBoolean()) {
                rightSide.setPattern(SOLID_YELLOW);
            } else if (nearBottom.getAsBoolean()) {
                rightSide.setPattern(SOLID_GREEN);
            } else {
                rightSide.setPattern(LEDPattern.kOff);
            }
        }
        super.periodic();
    }
}
