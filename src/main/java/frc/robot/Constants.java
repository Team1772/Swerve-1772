// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13;
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  public static final boolean DEV_MODE = true;

  public static class HIDConstants {
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class SwerveConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class IntakeConstants {
    public static final int MOTOR_CAN_ID = 12;
    public static final double INTAKE = 0.3;
    public static final double OUTTAKE = -0.6;
  }

  public static class JointConstants {
    public static final int LEFT_MOTOR_CAN_ID = 13;
    public static final int RIGHT_MOTOR_CAN_ID = 14;
    public static final int RIGHT_ENCODER_DIO_PORT = 3;
    public static final double ASCEND = -0.85;
    public static final double DESCEND = 0.7;
  }

  public static class PuncherConstants {
    public static final int LEFT_MOTOR_CAN_ID = 10;
    public static final int RIGHT_MOTOR_CAN_ID = 11;
    public static final int RELEASE_MOTOR_CAN_ID = 16;
    public static final int SENSOR_DIO_PORT = 4;
    public static final double POSITION = 45;
    public static final double RELEASE = 0.3;
    public static final double TIGHTEN = 0.31;
  }

  public static class ElevatorConstants {
    public static final int MOTOR_CAN_ID = 15;
    public static final int LIMIT_SWITCH_DIO_PORT = 0;
    public static final double ASCEND = -0.3;
    public static final double DESCEND = 0.3;
  }
}