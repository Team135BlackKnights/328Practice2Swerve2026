package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterS;

public class ShooterC extends Command{
    boolean isFinished = false;
    final ShooterS m_ShooterS;

    public ShooterC(ShooterS subsystem){
        addRequirements(subsystem);
        m_ShooterS = subsystem;
    }
    
    @Override
    public void execute(){
       m_ShooterS.fireSpeed(10);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

