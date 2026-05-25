// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.frc5010.common.drive.swerve.akit;

import static org.frc5010.common.drive.swerve.akit.util.SparkUtil.ifOk;
import static org.frc5010.common.drive.swerve.akit.util.SparkUtil.sparkStickyFault;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.akit.util.PhoenixUtil;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSparkTalon implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final SparkBase turnSpark;

  private final AbsoluteEncoder turnEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController turnController;

  // TalonFX Request
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;

  private final Queue<Double> turnPositionQueue;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  // Inlined turn motor defaults (originally from DriveConstants)
  private static final boolean TURN_INVERTED              = false;
  private static final boolean TURN_ENCODER_INVERTED      = true;
  private static final double  TURN_ENCODER_POS_FACTOR    = 2 * Math.PI;      // rotations → rad
  private static final double  TURN_ENCODER_VEL_FACTOR    = (2 * Math.PI) / 60.0; // RPM → rad/s
  private static final int     TURN_CURRENT_LIMIT         = 20;
  private static final double  TURN_KP                    = 2.0;
  private static final double  TURN_KD                    = 0.0;
  private static final double  TURN_PID_MIN               = 0.0;
  private static final double  TURN_PID_MAX               = 2 * Math.PI;

  public ModuleIOSparkTalon(
      SwerveConstants swerveConfig,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {

    this.constants = constants;
    zeroRotation = new Rotation2d();
    driveTalon = new TalonFX(constants.DriveMotorId, new CANBus(swerveConfig.canBusName));
    turnSpark = new SparkMax(constants.SteerMotorId, MotorType.kBrushless);

    turnEncoder = turnSpark.getAbsoluteEncoder();

    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor

    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(TURN_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TURN_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(TURN_ENCODER_INVERTED)
        .positionConversionFactor(TURN_ENCODER_POS_FACTOR)
        .velocityConversionFactor(TURN_ENCODER_VEL_FACTOR)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(TURN_PID_MIN, TURN_PID_MAX)
        .p(TURN_KP)
        .d(TURN_KD);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / swerveConfig.odometryFrequency.in(edu.wpi.first.units.Units.Hertz)))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue = TalonFXOdometryThread.getInstance().registerSignal(drivePosition.clone());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;

    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnAbsolutePosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), TURN_PID_MIN, TURN_PID_MAX);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }
}
