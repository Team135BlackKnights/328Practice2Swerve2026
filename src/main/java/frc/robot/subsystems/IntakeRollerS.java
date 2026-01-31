package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollerS extends SubsystemBase{
    private final Spark m_motor = new Spark(Constants.IntakeRollerConstants.rollerMotorID);

    public void rollerSpeed(double rollerVoltage){
            m_motor.setVoltage(rollerVoltage); 
    }
}
