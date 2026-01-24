package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase{
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.moduleLocationFrontLeft,SwerveConstants.moduleLocationFrontRight,
    SwerveConstants.moduleLocationBackLeft, SwerveConstants.moduleLocationBackRight);
    CANBus bus = new CANBus("E13B8EB250374E5320202047380C10FF");
    //public static CANBus bus = new CANBus("canivore", "./logs/example.hoot");
    SwerveModule frontLeftModule = new SwerveModule(SwerveConstants.frontLeftTurnID, SwerveConstants.frontLeftDriveID, SwerveConstants.frontLeftEncoderID, SwerveConstants.frontLeftOffsetRadians, bus);
    SwerveModule frontRightModule = new SwerveModule(SwerveConstants.frontRightTurnID, SwerveConstants.frontRightDriveID, SwerveConstants.frontRightEncoderID, SwerveConstants.frontRightOffsetRadians, bus);
    SwerveModule backLeftModule = new SwerveModule(SwerveConstants.backLeftTurnID, SwerveConstants.backLeftDriveID, SwerveConstants.backLeftEncoderID, SwerveConstants.backLeftOffsetRadians, bus);
    SwerveModule backRightModule = new SwerveModule(SwerveConstants.backRightTurnID, SwerveConstants.backRightDriveID, SwerveConstants.backRightEncoderID, SwerveConstants.backRightOffsetRadians, bus);


    @Override
    public void periodic(){
        frontLeftModule.updateStatePID();
        frontRightModule.updateStatePID();
        backLeftModule.updateStatePID();
        backRightModule.updateStatePID();
        // System.out.println("FR:" + frontLeftModule.getTurnPosition());
        // System.out.println("FL:" + frontRightModule.getTurnPosition());
        // System.out.println("BL:" + backLeftModule.getTurnPosition());
        // System.out.println("BR:" + backRightModule.getTurnPosition());
    
    }

    public void setSpeed(double xSpeed, double ySpeed, double rotSpeed){
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backLeft = moduleStates[2];
        SwerveModuleState backRight = moduleStates[3];

        System.out.println("FR:" + frontLeft);
        System.out.println("FL:" + frontRight);
        System.out.println("BL:" + backLeft);
        System.out.println("BR:" + backRight);
        
        frontLeftModule.setDesiredModuleState(frontLeft);
        frontRightModule.setDesiredModuleState(frontRight);
        backLeftModule.setDesiredModuleState(backLeft);
        backRightModule.setDesiredModuleState(backRight);

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


