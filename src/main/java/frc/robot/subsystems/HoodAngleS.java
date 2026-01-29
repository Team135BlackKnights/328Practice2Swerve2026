package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodAngleS extends SubsystemBase {
    private final Spark m_motor = new Spark(Constants.HoodConstants.hoodMotorID);
    private final DutyCycleEncoder m_Encoder = new DutyCycleEncoder(Constants.HoodConstants.hoodEncoderID);

    public void moveRange(double hoodVoltage){
        double currentPos = m_Encoder.get();
        if ((currentPos > Constants.HoodConstants.maxHoodRange) && hoodVoltage > 0 ) {
            m_motor.setVoltage(0);
        } else if ((currentPos < Constants.HoodConstants.minHoodRange) && hoodVoltage < 0) {
            m_motor.setVoltage(0);
        } else {
            m_motor.setVoltage(hoodVoltage); 
        }
    }
}
