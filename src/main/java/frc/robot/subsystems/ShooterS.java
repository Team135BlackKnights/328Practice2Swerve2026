package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterS extends SubsystemBase{
    private final SparkMax m_motor = new SparkMax(Constants.ShooterConstants.shootermotorID, MotorType.kBrushless);
    //private final SparkMax m_motor2 = new SparkMax(Constants.ShooterConstants.shootermotor2ID, MotorType.kBrushless);
    public void fire(double shooter1Voltage, double shooter2Voltage){
            m_motor.setVoltage(shooter1Voltage);
            //m_motor2.setVoltage(shooter2Voltage);
        }
}
