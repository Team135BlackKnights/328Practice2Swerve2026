package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangS extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(Constants.HangConstants.hangMotorID, MotorType.kBrushless);    
    
    public void hangPower(double hangVoltage){
        m_motor.setVoltage(hangVoltage); 
    }
}
