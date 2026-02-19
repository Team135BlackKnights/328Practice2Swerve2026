package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodAngleS;

public class ShooterHoodC extends Command{
    boolean isFinished = false;
    final HoodAngleS m_HoodAngleS;
    final double voltage;

    public ShooterHoodC(HoodAngleS subsystem, double v){
        addRequirements(subsystem);
        m_HoodAngleS = subsystem;
        voltage = v;
    }
    
    @Override
    public void execute(){
       m_HoodAngleS.moveHood(voltage);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
