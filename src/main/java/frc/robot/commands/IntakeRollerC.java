package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerS;

public class IntakeRollerC extends Command{
    boolean isFinished = false;
    final IntakeRollerS m_IntakeRollerS;
    final double voltage;

    public IntakeRollerC(IntakeRollerS subsystem, double v){
        addRequirements(subsystem);
        m_IntakeRollerS = subsystem;
        voltage = v;
    }
    
    @Override
    public void execute(){
       m_IntakeRollerS.rollerSpeed(voltage);
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

