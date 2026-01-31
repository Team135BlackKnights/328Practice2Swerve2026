package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollerS extends SubsystemBase{
    private final SparkMax m_motor = new SparkMax(Constants.IntakeRollerConstants.rollerMotorID, MotorType.kBrushless);

    public void rollerSpeed(double rollerVoltage){
            m_motor.setVoltage(rollerVoltage); 
    }
}
