package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodAngleS extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(Constants.HoodConstants.hoodMotorID, MotorType.kBrushless);
    private final DutyCycleEncoder m_Encoder = new DutyCycleEncoder(Constants.HoodConstants.hoodEncoderID);

    private PIDController intakeController = new PIDController(Constants.HoodConstants.hoodPID[0], Constants.HoodConstants.hoodPID[1], Constants.HoodConstants.hoodPID[2]);

    /*
    public void moveRange(double hoodVoltage, double setpoint){
        double voltage = intakeController.calculate(hoodVoltage, setpoint);

        voltage = MathUtil.clamp(voltage, -0.5, 0.5);
        double currentPos = m_Encoder.get();

        if ((currentPos > Constants.HoodConstants.maxHoodRange) && hoodVoltage > 0 ) {
            m_motor.setVoltage(0);
        } else if ((currentPos < Constants.HoodConstants.minHoodRange) && hoodVoltage < 0) {
            m_motor.setVoltage(0);
        } else {
            m_motor.setVoltage(hoodVoltage); 
        }
    }
    */

    // same as moveintake subsystem
    
    public void moveHood(double setpoint){
        double voltage = intakeController.calculate(m_Encoder.get(), setpoint);
        voltage = MathUtil.clamp(voltage, Constants.HoodConstants.hoodAngleVoltageNegative, Constants.HoodConstants.hoodAngleVoltagePositive);
        m_motor.setVoltage(voltage); 
    }

    public void setVoltage(double voltage){
        m_motor.setVoltage(voltage);
    }
}
