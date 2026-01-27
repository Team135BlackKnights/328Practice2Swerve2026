package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveS;
import frc.robot.RobotContainer;

public class SwerveC extends Command{
    boolean isFinished = false;
    final SwerveS m_Swerve;

    public SwerveC(SwerveS subsystem){
        addRequirements(subsystem);
        m_Swerve = subsystem;
    }
    
    @Override
    public void execute(){
        //TODO units are wrong here
        m_Swerve.setSpeed(-RobotContainer.m_driverController.getLeftX(), RobotContainer.m_driverController.getLeftY(), RobotContainer.m_driverController.getRightX());
        
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
