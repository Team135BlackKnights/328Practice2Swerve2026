package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase{
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.moduleLocationFrontLeft,SwerveConstants.moduleLocationFrontRight,
    SwerveConstants.moduleLocationBackLeft, SwerveConstants.moduleLocationBackRight);
    SwerveModule frontLeftModule = new SwerveModule(SwerveConstants.frontLeftTurnID, SwerveConstants.frontLeftDriveID, SwerveConstants.frontLeftEncoderID);
    SwerveModule frontRightModule = new SwerveModule(SwerveConstants.frontRightTurnID, SwerveConstants.frontRightDriveID, SwerveConstants.frontRightEncoderID);
    SwerveModule backLeftModule = new SwerveModule(SwerveConstants.backLeftTurnID, SwerveConstants.backLeftDriveID, SwerveConstants.backLeftEncoderID);
    SwerveModule backRightModule = new SwerveModule(SwerveConstants.backRightTurnID, SwerveConstants.backRightDriveID, SwerveConstants.backRightEncoderID);



    public void setFrontLeftDrive(){
        frontLeftModule.setDriveVoltage(8);
    }

    public void setSpeed(double xSpeed, double ySpeed, double rotSpeed){
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontLeft = moduleStates[0];//TODO optimize
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backLeft = moduleStates[2];
        SwerveModuleState backRight = moduleStates[3];
        
    }

    // public SwerveModuleState optimizeWithCosineCompensation(
    //         SwerveModuleState desiredState,
    //         Rotation2d currentAngle) {
    //     // Standard WPILib optimization (handles 180° flips)
    //     SwerveModuleState optimized =
    //             SwerveModuleState.optimize(desiredState, currentAngle);
    //     // Calculate angle error
    //     Rotation2d angleError =
    //             optimized.angle.minus(currentAngle);
    //     // Apply cosine compensation
    //  optimized.speedMetersPerSecond *= angleError.getCos();

    //     return optimized;
    // }


}


