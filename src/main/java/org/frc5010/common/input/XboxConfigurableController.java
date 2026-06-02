package org.frc5010.common.input;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

/**
 * A {@link ConfigurableController} pre-mapped for a standard Xbox controller,
 * providing named button and axis accessors alongside the index-based API.
 *
 * <p>Example — named bindings:
 * <pre>{@code
 * XboxConfigurableController driver = new XboxConfigurableController(0);
 *
 * JoystickAxis forward = driver.leftY().negate().deadzone(0.05).power(2.0);
 * JoystickAxis strafe  = driver.leftX().negate().deadzone(0.05).power(2.0);
 * JoystickAxis rotate  = driver.rightX().negate().deadzone(0.10);
 *
 * DriveVector translate = DriveVector.of(forward, strafe).unitCircle();
 *
 * driver.a().onTrue(Commands.runOnce(drive::zeroHeading));
 * driver.leftBumper().whileTrue(slowModeCommand);
 * }</pre>
 */
public class XboxConfigurableController extends ConfigurableController {

  // Maps WPILib 1-based button index → 0-based web UI button index; -1 = no web equivalent.
  private static final int[] WPILIB_TO_WEB = {
      -1,  // 0: unused
       0,  // 1: A
       1,  // 2: B
       2,  // 3: X
       3,  // 4: Y
       4,  // 5: LB
       5,  // 6: RB
      -1,  // 7: Back
      -1,  // 8: Start
      -1,  // 9: LeftStick
      -1,  // 10: RightStick
  };

  private BooleanSupplier[] webInputs = null;

  /**
   * Creates an Xbox controller on the specified Driver Station port.
   *
   * @param port WPILib joystick port (0–5)
   */
  public XboxConfigurableController(int port) {
    super(port);
  }

  /**
   * Injects web UI button suppliers so that {@link #button(int)} automatically OR-s the
   * physical button with its web equivalent. Called by
   * {@link org.frc5010.common.profiles.SwerveRobotContainer} after the web controller starts.
   * Pass {@code null} to disable web OR-ing (e.g., on real hardware).
   */
  public void setWebInputs(BooleanSupplier[] inputs) {
    this.webInputs = inputs;
  }

  /**
   * Returns a {@link Trigger} for button {@code index}. When web inputs have been injected
   * via {@link #setWebInputs} and {@code index} maps to a web UI button, the returned trigger
   * automatically OR-s the physical button with the corresponding web button. All named
   * accessors ({@link #a()}, {@link #leftBumper()}, etc.) route through this method.
   */
  @Override
  public Trigger button(int index) {
    Trigger physical = super.button(index);
    if (webInputs != null && index >= 1 && index < WPILIB_TO_WEB.length) {
      int webIdx = WPILIB_TO_WEB[index];
      if (webIdx >= 0 && webIdx < webInputs.length) {
        return physical.or(new Trigger(webInputs[webIdx]));
      }
    }
    return physical;
  }

  // ---- Enum-based accessors ----

  /**
   * Returns a {@link Trigger} for the given Xbox button.
   *
   * @param button the Xbox button to bind
   */
  public Trigger button(XboxController.Button button) {
    return button(button.value);
  }

  /**
   * Returns a {@link JoystickAxis} for the given Xbox axis.
   *
   * @param axis the Xbox axis to read
   */
  public JoystickAxis axis(XboxController.Axis axis) {
    return axis(axis.value);
  }

  // ---- Named button accessors ----

  /** Returns a Trigger for the A button. */
  public Trigger a() { return button(XboxController.Button.kA); }

  /** Returns a Trigger for the B button. */
  public Trigger b() { return button(XboxController.Button.kB); }

  /** Returns a Trigger for the X button. */
  public Trigger x() { return button(XboxController.Button.kX); }

  /** Returns a Trigger for the Y button. */
  public Trigger y() { return button(XboxController.Button.kY); }

  /** Returns a Trigger for the left bumper. */
  public Trigger leftBumper() { return button(XboxController.Button.kLeftBumper); }

  /** Returns a Trigger for the right bumper. */
  public Trigger rightBumper() { return button(XboxController.Button.kRightBumper); }

  /** Returns a Trigger for the Back button. */
  public Trigger back() { return button(XboxController.Button.kBack); }

  /** Returns a Trigger for the Start button. */
  public Trigger start() { return button(XboxController.Button.kStart); }

  /** Returns a Trigger for the left stick click. */
  public Trigger leftStick() { return button(XboxController.Button.kLeftStick); }

  /** Returns a Trigger for the right stick click. */
  public Trigger rightStick() { return button(XboxController.Button.kRightStick); }

  // ---- Named axis accessors ----

  /** Returns a JoystickAxis for the left stick X axis (left = negative). */
  public JoystickAxis leftX() { return axis(XboxController.Axis.kLeftX); }

  /** Returns a JoystickAxis for the left stick Y axis (up = negative). */
  public JoystickAxis leftY() { return axis(XboxController.Axis.kLeftY); }

  /** Returns a JoystickAxis for the right stick X axis (left = negative). */
  public JoystickAxis rightX() { return axis(XboxController.Axis.kRightX); }

  /** Returns a JoystickAxis for the right stick Y axis (up = negative). */
  public JoystickAxis rightY() { return axis(XboxController.Axis.kRightY); }

  /** Returns a JoystickAxis for the left trigger (0 = released, 1 = fully pressed). */
  public JoystickAxis leftTrigger() { return axis(XboxController.Axis.kLeftTrigger); }

  /** Returns a JoystickAxis for the right trigger (0 = released, 1 = fully pressed). */
  public JoystickAxis rightTrigger() { return axis(XboxController.Axis.kRightTrigger); }
}
