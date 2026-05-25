package org.frc5010.common.drive.swerve;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

/**
 * Immutable configuration record for a swerve drivetrain.
 *
 * <p>Students fill this out once per robot in their Constants.java and pass it
 * to SwerveFactory. All dimensions accept arbitrary WPILib units — the library
 * converts internally.
 *
 * <p>Example usage:
 * <pre>{@code
 * public static final SwerveConstants DRIVE = new SwerveConstants.Builder()
 *     .trackWidth(Inches.of(22.75))
 *     .wheelBase(Inches.of(22.75))
 *     .wheelRadius(Inches.of(2.0))
 *     .maxLinearSpeed(MetersPerSecond.of(4.5))
 *     .maxAngularSpeed(RadiansPerSecond.of(2 * Math.PI))
 *     .moduleType(ModuleType.TALON_FX)
 *     .gyroType(GyroType.PIGEON2)
 *     .gyroCanId(0)
 *     .frontLeftIds(1, 2, 3)
 *     .frontRightIds(4, 5, 6)
 *     .backLeftIds(7, 8, 9)
 *     .backRightIds(10, 11, 12)
 *     .build();
 * }</pre>
 */
public final class SwerveConstants {

  /** Supported swerve module hardware configurations. */
  public enum ModuleType {
    TALON_FX,        // Falcon 500 or Kraken X60 drive + steer
    SPARK_MAX,       // NEO drive + steer via SparkMax
    SPARK_TALON,     // NEO drive (SparkMax) + Falcon steer (TalonFX)
    SIM              // Simulation only — selected automatically in SIM mode
  }

  /** Supported gyro hardware. */
  public enum GyroType {
    PIGEON2,
    NAVX,
    SIM
  }

  // --- Physical geometry ---
  public final Distance trackWidth;
  public final Distance wheelBase;
  public final Distance wheelRadius;

  // --- Performance limits ---
  public final LinearVelocity maxLinearSpeed;
  public final AngularVelocity maxAngularSpeed;

  // --- Hardware selection ---
  public final ModuleType moduleType;
  public final GyroType gyroType;
  public final int gyroCanId;

  // --- CAN IDs: [driveId, steerId, encoderId] per module ---
  public final int[] frontLeftIds;
  public final int[] frontRightIds;
  public final int[] backLeftIds;
  public final int[] backRightIds;

  // --- CAN bus and odometry ---
  /** Phoenix CANivore bus name. Use {@code ""} for the default RIO bus. */
  public final String canBusName;
  /** High-frequency odometry update rate (e.g., {@code Hertz.of(100)} for typical CAN, {@code Hertz.of(250)} for CANivore). */
  public final Frequency odometryFrequency;

  // --- Physics simulation parameters ---
  /**
   * Total robot mass including bumpers and battery.
   * Used by the IronMaple physics engine in SIM mode. Valid range: 10–80 kg.
   */
  public final Mass robotMass;
  /**
   * Bumper length — full outside-to-outside dimension from front to back.
   * Used by the IronMaple physics engine for collision geometry in SIM mode.
   * Valid range: 0.5–1.5 m.
   */
  public final Distance bumperLength;
  /**
   * Bumper width — full outside-to-outside dimension from left to right.
   * Used by the IronMaple physics engine for collision geometry in SIM mode.
   * Valid range: 0.5–1.5 m.
   */
  public final Distance bumperWidth;

  // --- Derived geometry (computed once at construction) ---
  public final Translation2d[] moduleTranslations;

  private SwerveConstants(Builder b) {
    this.trackWidth      = b.trackWidth;
    this.wheelBase       = b.wheelBase;
    this.wheelRadius     = b.wheelRadius;
    this.maxLinearSpeed  = b.maxLinearSpeed;
    this.maxAngularSpeed = b.maxAngularSpeed;
    this.moduleType      = b.moduleType;
    this.gyroType        = b.gyroType;
    this.gyroCanId       = b.gyroCanId;
    this.frontLeftIds    = b.frontLeftIds;
    this.frontRightIds   = b.frontRightIds;
    this.backLeftIds     = b.backLeftIds;
    this.backRightIds    = b.backRightIds;
    this.canBusName      = b.canBusName;
    this.odometryFrequency = b.odometryFrequency;
    this.robotMass       = b.robotMass;
    this.bumperLength    = b.bumperLength;
    this.bumperWidth     = b.bumperWidth;

    // Compute module positions relative to robot center.
    // Order matches WPILib convention: FL, FR, BL, BR.
    double x = wheelBase.in(Meters) / 2.0;
    double y = trackWidth.in(Meters) / 2.0;
    this.moduleTranslations = new Translation2d[] {
      new Translation2d( x,  y),  // Front Left
      new Translation2d( x, -y),  // Front Right
      new Translation2d(-x,  y),  // Back Left
      new Translation2d(-x, -y),  // Back Right
    };
  }

  /**
   * Validates that this configuration is internally consistent.
   * Called by the Builder before constructing the object.
   *
   * @throws IllegalArgumentException if any value is out of range
   */
  private void validate() {
    if (trackWidth.in(Meters) <= 0)
      throw new IllegalArgumentException("trackWidth must be > 0, got: " + trackWidth);
    if (wheelBase.in(Meters) <= 0)
      throw new IllegalArgumentException("wheelBase must be > 0, got: " + wheelBase);
    if (wheelRadius.in(Meters) <= 0)
      throw new IllegalArgumentException("wheelRadius must be > 0, got: " + wheelRadius);
    if (maxLinearSpeed.in(MetersPerSecond) <= 0)
      throw new IllegalArgumentException("maxLinearSpeed must be > 0, got: " + maxLinearSpeed);
    if (maxAngularSpeed.in(RadiansPerSecond) <= 0)
      throw new IllegalArgumentException("maxAngularSpeed must be > 0, got: " + maxAngularSpeed);
    if (moduleType == null)
      throw new IllegalArgumentException("moduleType must not be null");
    if (gyroType == null)
      throw new IllegalArgumentException("gyroType must not be null");
    if (frontLeftIds == null || frontLeftIds.length < 2)
      throw new IllegalArgumentException("frontLeftIds must have at least [driveId, steerId]");
    if (frontRightIds == null || frontRightIds.length < 2)
      throw new IllegalArgumentException("frontRightIds must have at least [driveId, steerId]");
    if (backLeftIds == null || backLeftIds.length < 2)
      throw new IllegalArgumentException("backLeftIds must have at least [driveId, steerId]");
    if (backRightIds == null || backRightIds.length < 2)
      throw new IllegalArgumentException("backRightIds must have at least [driveId, steerId]");
    double massKg = robotMass.in(Kilograms);
    if (massKg < 10 || massKg > 80)
      throw new IllegalArgumentException(
          "robotMass must be 10–80 kg (FRC weight limits), got: " + robotMass);
    double bumpLenM = bumperLength.in(Meters);
    if (bumpLenM < 0.5 || bumpLenM > 1.5)
      throw new IllegalArgumentException(
          "bumperLength must be 0.5–1.5 m, got: " + bumperLength);
    double bumpWidM = bumperWidth.in(Meters);
    if (bumpWidM < 0.5 || bumpWidM > 1.5)
      throw new IllegalArgumentException(
          "bumperWidth must be 0.5–1.5 m, got: " + bumperWidth);
  }

  /** Fluent builder for SwerveConstants. */
  public static class Builder {
    private Distance trackWidth      = Inches.of(24);
    private Distance wheelBase       = Inches.of(24);
    private Distance wheelRadius     = Inches.of(2);
    private LinearVelocity maxLinearSpeed  = MetersPerSecond.of(4.5);
    private AngularVelocity maxAngularSpeed = RadiansPerSecond.of(2 * Math.PI);
    private ModuleType moduleType    = ModuleType.SIM;
    private GyroType gyroType        = GyroType.SIM;
    private int gyroCanId            = 0;
    private int[] frontLeftIds       = {1, 2, 3};
    private int[] frontRightIds      = {4, 5, 6};
    private int[] backLeftIds        = {7, 8, 9};
    private int[] backRightIds       = {10, 11, 12};
    private String canBusName        = "";
    private Frequency odometryFrequency = Hertz.of(100.0);
    // Physics simulation — defaults match IronMaple's DriveTrainSimulationConfig.Default()
    private Mass robotMass           = Kilograms.of(45.0);
    private Distance bumperLength    = Meters.of(0.76);
    private Distance bumperWidth     = Meters.of(0.76);

    public Builder trackWidth(Distance v)       { trackWidth = v; return this; }
    public Builder wheelBase(Distance v)        { wheelBase = v; return this; }
    public Builder wheelRadius(Distance v)      { wheelRadius = v; return this; }
    public Builder maxLinearSpeed(LinearVelocity v)  { maxLinearSpeed = v; return this; }
    public Builder maxAngularSpeed(AngularVelocity v) { maxAngularSpeed = v; return this; }
    public Builder moduleType(ModuleType v)     { moduleType = v; return this; }
    public Builder gyroType(GyroType v)         { gyroType = v; return this; }
    public Builder gyroCanId(int v)             { gyroCanId = v; return this; }

    /** Set drive, steer, and encoder CAN IDs for front-left module. */
    public Builder frontLeftIds(int drive, int steer, int encoder) {
      frontLeftIds = new int[]{drive, steer, encoder}; return this;
    }
    /** Set drive and steer CAN IDs (no separate encoder) for front-left module. */
    public Builder frontLeftIds(int drive, int steer) {
      frontLeftIds = new int[]{drive, steer}; return this;
    }
    public Builder frontRightIds(int drive, int steer, int encoder) {
      frontRightIds = new int[]{drive, steer, encoder}; return this;
    }
    public Builder frontRightIds(int drive, int steer) {
      frontRightIds = new int[]{drive, steer}; return this;
    }
    public Builder backLeftIds(int drive, int steer, int encoder) {
      backLeftIds = new int[]{drive, steer, encoder}; return this;
    }
    public Builder backLeftIds(int drive, int steer) {
      backLeftIds = new int[]{drive, steer}; return this;
    }
    public Builder backRightIds(int drive, int steer, int encoder) {
      backRightIds = new int[]{drive, steer, encoder}; return this;
    }
    public Builder backRightIds(int drive, int steer) {
      backRightIds = new int[]{drive, steer}; return this;
    }
    /** Phoenix CANivore bus name. Use {@code ""} for the default RIO bus. */
    public Builder canBusName(String v) { canBusName = v; return this; }
    /** High-frequency odometry rate (e.g., {@code Hertz.of(100)} for standard CAN, {@code Hertz.of(250)} for CANivore). */
    public Builder odometryFrequency(Frequency v) { odometryFrequency = v; return this; }

    /**
     * Total robot mass including bumpers and battery (10–80 kg).
     * Passed to the IronMaple physics engine so the sim reflects actual robot weight.
     * Default: 45 kg. Example: {@code Kilograms.of(55)} or {@code Pounds.of(120)}.
     */
    public Builder robotMass(Mass v) { robotMass = v; return this; }
    /**
     * Full bumper length, front to back outside-to-outside (0.5–1.5 m).
     * Used for physics collision geometry. Default: 0.76 m (~30 in).
     * Example: {@code Inches.of(30)} or {@code Meters.of(0.76)}.
     */
    public Builder bumperLength(Distance v) { bumperLength = v; return this; }
    /**
     * Full bumper width, left to right outside-to-outside (0.5–1.5 m).
     * Used for physics collision geometry. Default: 0.76 m (~30 in).
     * Example: {@code Inches.of(30)} or {@code Meters.of(0.76)}.
     */
    public Builder bumperWidth(Distance v) { bumperWidth = v; return this; }

    public SwerveConstants build() {
      SwerveConstants c = new SwerveConstants(this);
      c.validate();
      return c;
    }
  }
}
