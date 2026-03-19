package frc.robot.subsystems;

//import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.LoggableTunedNumber;
//import frc.robot.Robot;
import frc.robot.RobotContainer;

public class HoodAngleS extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(Constants.HoodConstants.hoodMotorID, MotorType.kBrushed);
    private RelativeEncoder m_Encoder = m_motor.getEncoder();
    //private boolean zeroing = false;
    //private double zeroSpikeStart = Double.NaN;
    // do not set to zero
    private PIDController controller = new PIDController(Constants.HoodConstants.hoodPID[0], Constants.HoodConstants.hoodPID[1], Constants.HoodConstants.hoodPID[2]);
    //private final LoggableTunedNumber kP = new LoggableTunedNumber("Hood/kP",Constants.HoodConstants.hoodPID[0],true);
    //private final LoggableTunedNumber kI = new LoggableTunedNumber("Hood/kI",Constants.HoodConstants.hoodPID[1],true);
    //private final LoggableTunedNumber kD = new LoggableTunedNumber("Hood/kD",Constants.HoodConstants.hoodPID[2],true);
    public void moveRange(double setpoint){
        double currentPos = m_Encoder.getPosition();
        double hoodVoltage = controller.calculate(currentPos, setpoint);
        
        // if ((currentPos > Constants.HoodConstants.maxHoodRangeRotations) && hoodVoltage > 0 ) {
        //     setVoltage(0);
        // } else if ((currentPos < Constants.HoodConstants.minHoodRangeRotations) && hoodVoltage < 0) {
        //     setVoltage(0);
        // } else {
        //     setVoltage(hoodVoltage);
        // }

        setVoltage(hoodVoltage);
    }
    public void setVoltage(double volts){
        MathUtil.clamp(volts, Constants.HoodConstants.hoodAngleVoltageNegative, Constants.HoodConstants.hoodAngleVoltagePositive);
        m_motor.setVoltage(volts);
    }

    // public void zero(){
    //     zeroing = true;
    //     zeroSpikeStart = Double.NaN;
    // }

    @Override
    public void periodic(){
        // zeroing = false;
        // if (zeroing){
        //     double now = Timer.getFPGATimestamp();
        //     setVoltage(-4);
        //     double observedAmps = Math.abs(m_motor.getOutputCurrent());
        //     if(observedAmps > 20){
        //         if (Double.isNaN(zeroSpikeStart)){
        //             zeroSpikeStart = now;
        //             System.out.println("Spike Detected");
        //         }
        //     } else {
        //         zeroSpikeStart = Double.NaN;
        //     }
        //     if (!Double.isNaN(zeroSpikeStart) && (now - zeroSpikeStart) >= 0.1){
        //         m_Encoder.setPosition(0);
        //         zeroSpikeStart = Double.NaN;
        //         zeroing = false;
        //     }
        // }else 
        if (Math.abs(RobotContainer.m_manipulatorController.getLeftY())>0.1){
            //moveRange(-0.3);
            setVoltage(25*RobotContainer.m_manipulatorController.getLeftY());
        } else {
            setVoltage(0);
        }
    
    }
}