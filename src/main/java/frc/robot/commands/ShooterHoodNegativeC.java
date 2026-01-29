package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodAngleS;

public class ShooterHoodNegativeC extends Command{
    boolean isFinished = false;
    final HoodAngleS m_HoodAngleS;

    public ShooterHoodNegativeC(HoodAngleS subsystem){
        addRequirements(subsystem);
        m_HoodAngleS = subsystem;
    }
    
    @Override
    public void execute(){
       m_HoodAngleS.moveRange(-5);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
