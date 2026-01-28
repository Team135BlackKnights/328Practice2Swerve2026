package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveS;

public class XLock extends Command{
    boolean isFinished = false;
    final SwerveS m_Swerve;

    final SwerveModuleState frontLeftModuleX = new SwerveModuleState(0, new Rotation2d(7*Math.PI/4));
    final SwerveModuleState frontRightModuleX = new SwerveModuleState(0, new Rotation2d(Math.PI/4));
    final SwerveModuleState backLeftModuleX = new SwerveModuleState(0, new Rotation2d(Math.PI/4));
    final SwerveModuleState backRightModuleX = new SwerveModuleState(0, new Rotation2d(7*Math.PI/4));

    public XLock(SwerveS subsystem){
        addRequirements(subsystem);
        m_Swerve = subsystem;
    }
    
    @Override
    public void execute(){
        m_Swerve.setModuleStates(frontLeftModuleX, frontRightModuleX, backLeftModuleX, backRightModuleX);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}