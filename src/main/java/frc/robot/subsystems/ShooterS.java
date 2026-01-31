package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterS extends SubsystemBase{
    private final Spark m_motor = new Spark(Constants.ShooterConstants.shooterHoodID);

    public void fireSpeed(double shooterVoltage){
            m_motor.setVoltage(shooterVoltage); 
        }
}
