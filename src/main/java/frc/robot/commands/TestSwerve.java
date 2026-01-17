package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TestSwerve extends Command{
    boolean isFinished = false;
    final Swerve m_Swerve;

    public TestSwerve(Swerve subsystem){
        addRequirements(subsystem);
        m_Swerve = subsystem;
    }

    @Override
    public void initialize() {
        m_Swerve.setFrontLeftDrive();
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
