package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterS;

public class ShooterC extends Command{
    boolean isFinished = false;
    final ShooterS m_ShooterS;
    final double v;

    public ShooterC(ShooterS subsystem, double voltage){
        addRequirements(subsystem);
        m_ShooterS = subsystem;
        v = voltage;
    }
    
    @Override
    public void execute(){
       m_ShooterS.fireSpeed(v);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

