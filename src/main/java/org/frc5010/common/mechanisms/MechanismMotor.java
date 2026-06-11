package org.frc5010.common.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Creates YAMS {@link SmartMotorController} wrappers from a vendor enum + CAN ID,
 * so mechanism settings stay declarative and vendor-agnostic.
 *
 * <p>This is the single place where vendor motor objects are constructed for the
 * YAMS mechanism wrappers ({@link YamsElevator}, {@link YamsArm}, {@link YamsPivot},
 * {@link YamsFlywheel}, ...). Teams switch vendors by changing one enum value in
 * their settings — no other code changes.
 */
public final class MechanismMotor {

  /** Supported motor controller vendors for YAMS mechanisms. */
  public enum Vendor {
    /** CTRE TalonFX (Falcon 500, Kraken X60/X44). Preferred for examples. */
    TALON_FX,
    /** CTRE TalonFXS (brushed or NEO-class motors on a TalonFXS controller). */
    TALON_FXS,
    /** REV SparkMax (NEO, NEO 550), brushless mode. */
    SPARK_MAX,
    /** REV SparkFlex (NEO Vortex), brushless mode. */
    SPARK_FLEX
  }

  private MechanismMotor() {}

  /**
   * Builds a YAMS smart motor controller wrapper for the given vendor.
   *
   * @param vendor  motor controller vendor
   * @param canId   CAN ID of the controller
   * @param model   physics model of the attached motor (e.g. {@code DCMotor.getKrakenX60(1)})
   * @param config  fully-populated YAMS motor config (applied by the wrapper constructor)
   * @return wired {@link SmartMotorController}
   */
  public static SmartMotorController create(
      Vendor vendor, int canId, DCMotor model, SmartMotorControllerConfig config) {
    return switch (vendor) {
      case TALON_FX -> new TalonFXWrapper(new TalonFX(canId), model, config);
      case TALON_FXS -> new TalonFXSWrapper(new TalonFXS(canId), model, config);
      case SPARK_MAX -> new SparkWrapper(new SparkMax(canId, MotorType.kBrushless), model, config);
      case SPARK_FLEX -> new SparkWrapper(new SparkFlex(canId, MotorType.kBrushless), model, config);
    };
  }

  /**
   * Closes the underlying vendor motor object (frees the CAN ID for unit tests).
   * The YAMS wrapper's own {@link SmartMotorController#close()} stops the RIO
   * closed-loop Notifier but does not close the device handle.
   *
   * @param motor wrapper whose vendor device should be closed
   */
  public static void closeDevice(SmartMotorController motor) {
    Object device = motor.getMotorController();
    if (device instanceof AutoCloseable closeable) {
      try {
        closeable.close();
      } catch (Exception e) {
        // Best-effort cleanup for tests; nothing actionable at runtime.
      }
    }
  }
}
