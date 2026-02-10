package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangS;

public class HangC extends Command {
    boolean isFinished = false;
    final HangS m_HangS;
    final double v;

    public HangC(HangS subsystem, double voltage){
        addRequirements(subsystem);
        m_HangS = subsystem;
        v = voltage;
    }
    
    @Override
    public void execute(){
       m_HangS.hangPower(v);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
