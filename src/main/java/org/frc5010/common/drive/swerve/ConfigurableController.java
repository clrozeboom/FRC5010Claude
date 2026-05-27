package org.frc5010.common.drive.swerve;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A joystick or game controller on a WPILib Driver Station port, providing configurable axes
 * and button triggers.
 *
 * <p>Each call to {@link #axis} returns a fresh {@link JoystickAxis} ready for transform
 * chaining. Multiple controllers can be instantiated on different ports to model a two-driver
 * station setup or separate throttle/flight-stick configurations.
 *
 * <p>Example — two controllers, different response curves:
 * <pre>{@code
 * ConfigurableController driver   = new ConfigurableController(0);
 * ConfigurableController operator = new ConfigurableController(1);
 *
 * JoystickAxis forward = driver.axis(1).negate().deadzone(0.05).power(2.0);
 * JoystickAxis strafe  = driver.axis(0).negate().deadzone(0.05).power(2.0);
 * JoystickAxis rotate  = driver.axis(2).negate().deadzone(0.10);
 *
 * DriveVector translate = DriveVector.of(forward, strafe).unitCircle();
 * }</pre>
 */
public class ConfigurableController {

  private final GenericHID hid;

  /**
   * Creates a controller on the specified Driver Station port.
   *
   * @param port WPILib joystick port (0–5)
   */
  public ConfigurableController(int port) {
    this.hid = new GenericHID(port);
  }

  /**
   * Returns a {@link JoystickAxis} for the specified axis index.
   * Each call returns a new instance; stack transforms on the returned object.
   *
   * @param index WPILib axis index (0 = X, 1 = Y, 2 = Z for most HID devices)
   */
  public JoystickAxis axis(int index) {
    return JoystickAxis.of(() -> hid.getRawAxis(index));
  }

  /**
   * Returns a {@link Trigger} that is active while the specified button is held.
   *
   * @param index WPILib button index (1-based for most HID devices)
   */
  public Trigger button(int index) {
    return new Trigger(() -> hid.getRawButton(index));
  }

  /** Returns the Driver Station port this controller is bound to. */
  public int getPort() {
    return hid.getPort();
  }
}
