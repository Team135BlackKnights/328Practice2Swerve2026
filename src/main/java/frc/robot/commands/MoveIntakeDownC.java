package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeS;

public class MoveIntakeDownC extends Command{
    boolean isFinished = false;
    final IntakeS m_IntakeS;

    public MoveIntakeDownC(IntakeS subsystem){
        addRequirements(subsystem);
        m_IntakeS = subsystem;
    }
    
    @Override
    public void execute(){
       m_IntakeS.moveTo(0);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
