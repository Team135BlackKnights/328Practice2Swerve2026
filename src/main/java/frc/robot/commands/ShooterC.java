package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterS;

public class ShooterC extends Command{
    boolean isFinished = false;
    final ShooterS m_ShooterS;
    final double v;
    final double v2;

    public ShooterC(ShooterS subsystem, double voltage, double voltage2){
        addRequirements(subsystem);
        m_ShooterS = subsystem;
        v = -voltage;
        v2 = -voltage2;
    }
    
    @Override
    public void execute(){
       m_ShooterS.fire(v,v2);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

