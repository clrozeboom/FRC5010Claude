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
import org.frc5010.common.drive.swerve.SwerveConstants;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 */
public class TalonFXOdometryThread extends OdometryThread {
  private final Lock signalsLock = new ReentrantLock();
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final SwerveConstants swerveConfig;

  private static boolean isCANFD;
  private static TalonFXOdometryThread instance = null;

  public static TalonFXOdometryThread getInstance() {
    return instance;
  }

  public static void createInstance(SwerveConstants constants) {
    if (instance == null) {
      instance = new TalonFXOdometryThread(constants);
    }
  }

  private TalonFXOdometryThread(SwerveConstants constants) {
    super("TalonFXOdometryThread");
    this.swerveConfig = constants;
    isCANFD = new CANBus(constants.canBusName).isNetworkFD();
    commonInstance = this;
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
  public void runThreadLogic() {
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
