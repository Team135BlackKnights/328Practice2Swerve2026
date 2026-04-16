package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeRollerS;
import frc.robot.subsystems.MoveIntakeS;

public class intakeA extends Command{
    final MoveIntakeS moveintake;
    final IntakeRollerS rollers;

    public intakeA(MoveIntakeS m_MoveIntakeS, IntakeRollerS m_IntakeRollerS){
        moveintake = m_MoveIntakeS;
        rollers = m_IntakeRollerS;
    }

    public void execute(){
        moveintake.moveTo(Constants.IntakeConstants.downPositionSetpoint);
        if (moveintake.getEncoderPositionWithOffset() < 0.2 && moveintake.getEncoderPositionWithOffset() > 0.5){
            rollers.setVoltage(Constants.IntakeRollerConstants.rollerVoltage);
        } else {
            rollers.setVoltage(0);
        }
    }
}
