package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangS extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(Constants.HoodConstants.hoodMotorID, MotorType.kBrushless);
    private final DutyCycleEncoder m_Encoder = new DutyCycleEncoder(Constants.HoodConstants.hoodEncoderID);

    private PIDController controller = new PIDController(Constants.HoodConstants.hoodPID[0], Constants.HoodConstants.hoodPID[1], Constants.HoodConstants.hoodPID[2]);
 
    
    public void hangPower(double setpoint){
        double voltage = controller.calculate(m_Encoder.get(), setpoint);
        voltage = MathUtil.clamp(voltage, Constants.HangConstants.hangMinVoltage, Constants.HangConstants.hangMaxVoltage);
        m_motor.setVoltage(voltage); 
    }
}
