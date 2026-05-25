package org.frc5010.common.unit;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.*;

import org.frc5010.common.drive.swerve.SwerveConstants;
import org.frc5010.common.drive.swerve.SwerveConstants.GyroType;
import org.frc5010.common.drive.swerve.SwerveConstants.ModuleType;
import org.junit.jupiter.api.Test;

/**
 * Layer 1 unit tests for SwerveConstants.
 * No HAL, no WPILib sim — pure Java logic only.
 */
class SwerveConstantsTest {

  /** A valid baseline config used across multiple tests. */
  private SwerveConstants.Builder validBuilder() {
    return new SwerveConstants.Builder()
        .trackWidth(Inches.of(22.75))
        .wheelBase(Inches.of(22.75))
        .wheelRadius(Inches.of(2.0))
        .maxLinearSpeed(MetersPerSecond.of(4.5))
        .maxAngularSpeed(RadiansPerSecond.of(2 * Math.PI))
        .moduleType(ModuleType.SIM)
        .gyroType(GyroType.SIM)
        .frontLeftIds(1, 2, 3)
        .frontRightIds(4, 5, 6)
        .backLeftIds(7, 8, 9)
        .backRightIds(10, 11, 12);
  }

  @Test
  void validConfigBuildsSuccessfully() {
    assertDoesNotThrow(() -> validBuilder().build());
  }

  @Test
  void moduleTranslationsHaveCorrectCount() {
    SwerveConstants c = validBuilder().build();
    assertEquals(4, c.moduleTranslations.length);
  }

  @Test
  void moduleTranslationsReflectGeometry() {
    double trackWidthM = Inches.of(22.75).in(Meters);
    double wheelBaseM  = Inches.of(22.75).in(Meters);
    SwerveConstants c = validBuilder()
        .trackWidth(Inches.of(22.75))
        .wheelBase(Inches.of(22.75))
        .build();

    double expectedX = wheelBaseM / 2.0;
    double expectedY = trackWidthM / 2.0;

    // Front Left: (+x, +y)
    assertEquals( expectedX, c.moduleTranslations[0].getX(), 1e-6);
    assertEquals( expectedY, c.moduleTranslations[0].getY(), 1e-6);
    // Front Right: (+x, -y)
    assertEquals( expectedX, c.moduleTranslations[1].getX(), 1e-6);
    assertEquals(-expectedY, c.moduleTranslations[1].getY(), 1e-6);
    // Back Left: (-x, +y)
    assertEquals(-expectedX, c.moduleTranslations[2].getX(), 1e-6);
    assertEquals( expectedY, c.moduleTranslations[2].getY(), 1e-6);
    // Back Right: (-x, -y)
    assertEquals(-expectedX, c.moduleTranslations[3].getX(), 1e-6);
    assertEquals(-expectedY, c.moduleTranslations[3].getY(), 1e-6);
  }

  @Test
  void zeroTrackWidthThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().trackWidth(Meters.of(0)).build());
  }

  @Test
  void negativeWheelBaseThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().wheelBase(Meters.of(-0.1)).build());
  }

  @Test
  void zeroWheelRadiusThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().wheelRadius(Meters.of(0)).build());
  }

  @Test
  void zeroMaxSpeedThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().maxLinearSpeed(MetersPerSecond.of(0)).build());
  }

  @Test
  void nullModuleTypeThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().moduleType(null).build());
  }

  @Test
  void nullGyroTypeThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().gyroType(null).build());
  }

  @Test
  void defaultBuilderIsValid() {
    // Default builder should produce a working sim config out of the box
    assertDoesNotThrow(() -> new SwerveConstants.Builder().build());
  }

  // --- Physics simulation fields ---

  @Test
  void physicsFieldsHaveCorrectDefaults() {
    SwerveConstants c = new SwerveConstants.Builder().build();
    assertEquals(45.0, c.robotMass.in(Kilograms),    1e-6, "default robot mass");
    assertEquals(0.76, c.bumperLength.in(Meters),     1e-6, "default bumper length");
    assertEquals(0.76, c.bumperWidth.in(Meters),      1e-6, "default bumper width");
  }

  @Test
  void customPhysicsFieldsAreStored() {
    SwerveConstants c = validBuilder()
        .robotMass(Kilograms.of(60.0))
        .bumperLength(Meters.of(0.9))
        .bumperWidth(Meters.of(0.85))
        .build();
    assertEquals(60.0, c.robotMass.in(Kilograms),   1e-6);
    assertEquals(0.9,  c.bumperLength.in(Meters),    1e-6);
    assertEquals(0.85, c.bumperWidth.in(Meters),     1e-6);
  }

  @Test
  void robotMassBelowMinThrows() {
    // IronMaple lower bound is 10 kg
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().robotMass(Kilograms.of(9.9)).build());
  }

  @Test
  void robotMassAboveMaxThrows() {
    // IronMaple upper bound is 80 kg (FRC weight limit with bumpers is ~68 kg)
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().robotMass(Kilograms.of(80.1)).build());
  }

  @Test
  void bumperTooSmallThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().bumperLength(Meters.of(0.49)).build());
  }

  @Test
  void bumperTooLargeThrows() {
    assertThrows(IllegalArgumentException.class,
        () -> validBuilder().bumperWidth(Meters.of(1.51)).build());
  }
}
