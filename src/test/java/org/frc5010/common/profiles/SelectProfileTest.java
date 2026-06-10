package org.frc5010.common.profiles;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/**
 * Layer 1 — unit tests for {@link SwerveRobotContainer#selectProfile(String)}.
 *
 * <p>These tests verify the three selection branches without touching any
 * subsystem, CommandScheduler, or hardware abstraction layer.
 */
class SelectProfileTest {

  @AfterEach
  void clearProperties() {
    System.clearProperty("testSim");
  }

  @Test
  void testSimPropertyReturnsSimRobotProfile() {
    System.setProperty("testSim", "true");
    RobotProfile profile = SwerveRobotContainer.selectProfile("frc.robot.rebuilt.RealRobotProfile");
    assertInstanceOf(SimRobotProfile.class, profile,
        "testSim=true must substitute SimRobotProfile regardless of the class name argument");
  }

  @Test
  void noPropertyInstantiatesNamedClassReflectively() {
    // testSim not set → selectProfile must reflectively instantiate RealRobotProfile.
    // We check the class name rather than importing frc.robot types into the common package.
    RobotProfile profile = SwerveRobotContainer.selectProfile("frc.robot.rebuilt.RealRobotProfile");
    assertEquals("frc.robot.rebuilt.RealRobotProfile", profile.getClass().getName(),
        "Without testSim, selectProfile must reflectively instantiate the named class");
  }

  @Test
  void subclassOfRealRobotProfileInstantiatedByName() {
    // A team-supplied subclass of RealRobotProfile can be passed without changing library code.
    // We use SimRobotProfile as a stand-in (it has a no-arg constructor and extends RobotProfile).
    RobotProfile profile = SwerveRobotContainer.selectProfile(
        "org.frc5010.common.profiles.SimRobotProfile");
    assertEquals("org.frc5010.common.profiles.SimRobotProfile", profile.getClass().getName(),
        "selectProfile must instantiate any RobotProfile subclass by fully-qualified name");
  }

  @Test
  void unknownClassNameThrowsRuntimeException() {
    assertThrows(RuntimeException.class,
        () -> SwerveRobotContainer.selectProfile("com.nonexistent.BogusProfile"),
        "An unresolvable class name must wrap the ReflectiveOperationException in RuntimeException");
  }
}
