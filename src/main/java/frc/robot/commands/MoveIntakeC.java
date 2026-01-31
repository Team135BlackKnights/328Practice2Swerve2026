package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MoveIntakeS;

public class MoveIntakeC extends Command{
    boolean isFinished = false;
    final MoveIntakeS m_IntakeS;
    final double position;

    public MoveIntakeC(MoveIntakeS subsystem, double p){
        addRequirements(subsystem);
        m_IntakeS = subsystem;
        position = p;
    }
    
    @Override
    public void execute(){
       m_IntakeS.moveTo(position);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
