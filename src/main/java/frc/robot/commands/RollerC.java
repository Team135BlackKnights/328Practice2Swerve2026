package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerS;

public class RollerC extends Command{
    boolean isFinished = false;
    final IntakeRollerS m_IntakeRollerS;

    public RollerC(IntakeRollerS subsystem){
        addRequirements(subsystem);
        m_IntakeRollerS = subsystem;
    }
    
    @Override
    public void execute(){
       m_IntakeRollerS.rollerSpeed(10);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

