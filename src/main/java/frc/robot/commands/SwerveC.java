package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule;

public class SwerveC extends Command{
    boolean isFinished = false;
    final Swerve m_Swerve;

    public SwerveC(Swerve subsystem){
        addRequirements(subsystem);
        m_Swerve = subsystem;
    }
    
    @Override
    public void execute(){
        //TODO units are wrong here
        m_Swerve.setSpeed(-0*RobotContainer.m_driverController.getLeftX(), 0*RobotContainer.m_driverController.getLeftY(), 1+0*RobotContainer.m_driverController.getRightX());
        
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
