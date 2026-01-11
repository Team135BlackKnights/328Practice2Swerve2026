package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class Swerve {
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.moduleLocationFrontLeft,SwerveConstants.moduleLocationFrontRight,
    SwerveConstants.moduleLocationBackLeft, SwerveConstants.moduleLocationBackRight);



    public void setSpeed(double xSpeed, double ySpeed, double rotSpeed){
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontLeft = moduleStates[0];//TODO optimize
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backLeft = moduleStates[2];
        SwerveModuleState backRight = moduleStates[3];
    }
















}


