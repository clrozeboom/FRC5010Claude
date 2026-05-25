// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.frc5010.common.drive.swerve.akit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.frc5010.common.drive.swerve.SwerveConstants;

/**
 * Legacy Phoenix odometry thread (pre-TalonFX). Prefer {@link TalonFXOdometryThread} for new
 * projects. Kept for compatibility with older swerve configurations.
 */
public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock = new ReentrantLock();
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();
  private final SwerveConstants swerveConfig;

  private static boolean isCANFD;
  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    return instance;
  }

  public static void createInstance(SwerveConstants constants) {
    if (instance == null) {
      instance = new PhoenixOdometryThread(constants);
    }
  }

  private PhoenixOdometryThread(SwerveConstants constants) {
    this.swerveConfig = constants;
    isCANFD = new CANBus(constants.canBusName).isNetworkFD();
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (!timestampQueues.isEmpty() && RobotBase.isReal()) {
      super.start();
    }
  }

  /** Registers a Phoenix signal to be read from the thread. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    OdometryThread.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      OdometryThread.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    OdometryThread.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      OdometryThread.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    OdometryThread.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      OdometryThread.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      signalsLock.lock();
      try {
        if (isCANFD && phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / swerveConfig.odometryFrequency.in(edu.wpi.first.units.Units.Hertz), phoenixSignals);
        } else {
          Thread.sleep((long) (1000.0 / swerveConfig.odometryFrequency.in(edu.wpi.first.units.Units.Hertz)));
          if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      OdometryThread.odometryLock.lock();
      try {
        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : phoenixSignals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (phoenixSignals.length > 0) {
          timestamp -= totalLatency / phoenixSignals.length;
        }

        for (int i = 0; i < phoenixSignals.length; i++) {
          phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        OdometryThread.odometryLock.unlock();
      }
    }
  }
}
