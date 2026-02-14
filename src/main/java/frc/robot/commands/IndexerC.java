package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerS;

public class IndexerC extends Command{
    boolean isFinished = false;
    final IndexerS m_IndexerS;
    final double v;

    public IndexerC(IndexerS subsystem, double indexerVoltage){
        addRequirements(subsystem);
        m_IndexerS = subsystem;
        v = indexerVoltage;
    }
    
    @Override
    public void execute(){
       m_IndexerS.setVoltage(v);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
