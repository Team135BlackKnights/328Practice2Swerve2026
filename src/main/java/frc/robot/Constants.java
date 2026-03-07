// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kManipulatorControllerPort = 0;
  }
  public static class SwerveConstants {
    public static final Translation2d moduleLocationFrontLeft = new Translation2d(-0.1875, 0.377);
    public static final Translation2d moduleLocationFrontRight = new Translation2d(0.1875, 0.377);
    public static final Translation2d moduleLocationBackLeft = new Translation2d(-0.1875, -0.377);
    public static final Translation2d moduleLocationBackRight = new Translation2d(0.1875, -0.377);

    //if turn is x locking: add Math.toRadians(-90) to offsets
    public static final int frontLeftTurnID = 9;
    public static final int frontLeftDriveID = 6;
        public static final int frontLeftEncoderID = 1;
    public static final double frontLeftOffsetRadians = Math.toRadians(-90);

    public static final int frontRightTurnID = 8;
    public static final int frontRightDriveID = 11;
    public static final int frontRightEncoderID = 4;
    public static final double frontRightOffsetRadians = Math.toRadians(-90);

    public static final int backLeftTurnID = 5;
    public static final int backLeftDriveID = 12;
    public static final int backLeftEncoderID = 2;
    public static final double backLeftOffsetRadians = Math.toRadians(-90);

    public static final int backRightTurnID = 7;
    public static final int backRightDriveID = 10;
    public static final int backRightEncoderID = 3;
    public static final double backRightOffsetRadians =  Math.toRadians(-90);

    public static final InvertedValue turnInversion = InvertedValue.Clockwise_Positive;
    public static final InvertedValue driveLeftInversion = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue driveRightInversion = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue turnNeutralMode = NeutralModeValue.Coast; 
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake; 

    public static final double[] turnPID = new double[] {5,0,0.1};
    public static final double[] drivePID = new double[] {1,0,0,.3,2.1};

    public static final double gearRatioSpeed = (1/5.9); 
    public static final double wheelRadius = 2 * 0.0254;

    public static final double maxLinearSpeedMPS = 6;
    public static final double maxRadSpeedRPS = 10;
  }

  public static final double robotMassKG = 50;

  public static class IntakeConstants {
    public static final double[] intakePID = new double[] {0.6,0,0.05};
    public static final int intakeVertMotorID = 30;
    public static final int intakeVertEncoderID = 21;
    public static final double downPositionP = 0; 
    public static final double upPositionP = -6;
  }

  public static class HoodConstants {
    public static final int hoodMotorID = 36;
    public static final int hoodEncoderID = 37;
    public static final double minHoodRangeRotations = 0;
    public static final double maxHoodRangeRotations = 35;
    public static final double hoodAngleVoltagePositive = 10.0;
    public static final double hoodAngleVoltageNegative = -10.0;
    public static final double[] hoodPID = new double[]{5,0,0};
  }
  
  public static class ShooterConstants {
    public static final int shooterMotorID = 24;//kicker
    public static final int shooterMotor2ID = 25;//flywheel
    public static final double shooter1Voltage = 7;
    public static final double shooter2voltage = -5;
  } 

  public static class IntakeRollerConstants {
    public static final int rollerMotorID = 31;
    public static final double rollerVoltage = 7;
  }

  public static class PigeonConstants {
    public static final int pigeonID = 0;
  }
  public static class HangConstants{
    public static final int hangMotorID = 27;
    public static final double hangVoltage = 8;
  }

  public static class IndexerConstants {
    public static final int indexerMotorID = 20;
    public static final double indexerVoltage = -3.5;
    public static final boolean indexerRunValue = true; 
  }
}

