// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /* ----- Elevator ID's ----- */
  public static final int kElevatorMotor1ID = 26;
  public static final int kElevatorMotor2ID = 27;

  /* ----- Intake System ----- */
  public static final int kAngleMotorID = 28;
  public static final int kIntakeMotorID = 20;
  public static final int kLaserCanID = 20000;
  public static final double kSpeedUpTime = 1.0; // in seconds

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
