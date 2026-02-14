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

    public static final int frontLeftTurnID = 9;
    public static final int frontLeftDriveID = 6;
    public static final int frontLeftEncoderID = 1;
    public static final double frontLeftOffsetRadians = -Math.PI/5;

    public static final int frontRightTurnID = 8;
    public static final int frontRightDriveID = 11;
    public static final int frontRightEncoderID = 4;
    public static final double frontRightOffsetRadians = 8*Math.PI/9; //this is good

    public static final int backLeftTurnID = 5;
    public static final int backLeftDriveID = 12;
    public static final int backLeftEncoderID = 2;
    public static final double backLeftOffsetRadians = Math.PI/2; //this is good

    public static final int backRightTurnID = 7;
    public static final int backRightDriveID = 10;
    public static final int backRightEncoderID = 3;
    public static final double backRightOffsetRadians =  6*Math.PI/9;

    public static final InvertedValue turnInversion = InvertedValue.Clockwise_Positive;
    public static final InvertedValue driveInversion = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue turnNeutralMode = NeutralModeValue.Coast; 
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake; 

    public static final double[] turnPID = new double[] {3,0,0};
    public static final double[] drivePID = new double[] {1,0,0};

    public static final double gearRatioSpeed = (1/5.9); 
    public static final double wheelRadius = 2 * 0.0254;

    public static final double maxLinearSpeedMPS = 0;
    public static final double maxRadSpeedRPS = 0;
  }

  public static final double robotMassKG = 100;

  public static class IntakeConstants {
    public static final double[] intakePID = new double[] {1,0,0};
    public static final int intakeVertMotorID = 20;
    public static final int intakeVertEncoderID = 21;
    public static final double desiredPositionP = 0.25; //this is the P in PID for lowering intake angle. in constants because easy place to change
    public static final double upPositionP = 0; //depending on how PIDs go this could be different(tm)
  }

  public static class HoodConstants {
    public static final int hoodMotorID = 22;
    public static final int hoodEncoderID = 23;
    public static final double minHoodRange = 0;
    public static final double maxHoodRange = 3.5;
    public static final double hoodAngleVoltagePositive = 5.0;
    public static final double hoodAngleVoltageNegative = -5.0;
  }
  
  public static class ShooterConstants {
    public static final int shooterMotorID = 24;
    public static final int shooterMotor2ID = 25;
    public static final double shooter1Voltage = -7;
    public static final int shooter2voltage = -7;
  }

  public static class IntakeRollerConstants {
    public static final int rollerMotorID = 26;
    public static final double rollerVoltage = -6;
  }

  public static class PigeonConstants {
    public static final int pigeonID = 0;
  }
  public static class HangConstants{
    public static final int hangMotorID = 27;
    public static final double hangVoltage = 8;
  }

  public static class IndexerConstants {
    public static final int indexerMotorID = 28;
    public static final double indexerVoltage = 3;
    public static final boolean indexerRunValue = true;
  }
}

