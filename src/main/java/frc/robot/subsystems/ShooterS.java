package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterS extends SubsystemBase{
    private final SparkMax kickMotor = new SparkMax(Constants.ShooterConstants.shooterMotorID, MotorType.kBrushless);
    private final SparkMax shootMotor = new SparkMax(Constants.ShooterConstants.shooterMotor2ID, MotorType.kBrushless);
    private final PIDController flyController = new PIDController(1, 0, 0);
    private final PIDController kickController = new PIDController(1, 0, 0);
    //dio channel
    private final RelativeEncoder kickupEncoder = kickMotor.getEncoder();
    private final DutyCycleEncoder shooterEncoder = new DutyCycleEncoder(1);
    double flyPreviousPosition = shooterEncoder.get();
    //double kickPreviousPosition = kickupEncoder.get();
    double previousTime;
    public double flyVelocity;
    //public double kickVelocity;
    // rpms in volts / rpms
    // so if 1 volt gives 300 rpm the number is 300, as 1/300 volts would then give 1 rpm
    private double flyvoltageconstant = 1000;
    //private double kickvoltageconstant = 500;

    public ShooterS(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(12);
        //make higher if motor no work
        config.smartCurrentLimit(65);
        kickMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shootMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void fire(double kickupVoltage, double desiredFlyVelocity){
        double flywheelVoltage = flyController.calculate(flyVelocity, desiredFlyVelocity) / flyvoltageconstant;
        //double kickupVoltage = kickController.calculate(kickVelocity, desiredKickVelocity) / kickvoltageconstant;
        //kickupVoltage = kickupController.calculate(kickupVoltage);
        //flywheelVoltage = shooterController.calculate(flywheelVoltage);
        //kickupVoltage = MathUtil.clamp(kickupVoltage, -1*Math.abs(Constants.ShooterConstants.shooter1Voltage), Math.abs(Constants.ShooterConstants.shooter1Voltage));
        //flywheelVoltage = MathUtil.clamp(flywheelVoltage, -1*Math.abs(Constants.ShooterConstants.shooter2voltage), Math.abs(Constants.ShooterConstants.shooter2voltage));
        kickMotor.setVoltage(kickupVoltage);
        shootMotor.setVoltage(flywheelVoltage);
    }

    public void idle(double idleVoltage){
        kickMotor.setVoltage(0);
        //
        idleVoltage = MathUtil.clamp(idleVoltage, -0.5*Math.abs(Constants.ShooterConstants.flywheelRPM), 0.5*Math.abs(Constants.ShooterConstants.flywheelRPM));
        shootMotor.setVoltage(idleVoltage);
    }

    public void stop(){
        kickMotor.setVoltage(0);
        shootMotor.setVoltage(0);
    }

    public void getPos(){
        
    }

    public void periodic(){
        double flyCurrentPosition = shooterEncoder.get(); 
        //double kickCurrentPosition = kickupEncoder.get();
        double time = Timer.getFPGATimestamp();
        // delta position over delta time
        
        flyCurrentPosition = shooterEncoder.get();
        int updatedRegion = (int) (flyCurrentPosition * 3);
        int oldRegion = (int) (flyPreviousPosition * 3);

        if(oldRegion == 0 && updatedRegion == 2){
            flyCurrentPosition --;
        } else if(oldRegion == 2 && updatedRegion == 0){
            flyCurrentPosition ++;
        }

        flyVelocity = ((flyCurrentPosition - flyPreviousPosition) / (time - previousTime));

        //kickVelocity = ((kickCurrentPosition - kickPreviousPosition) / (time - previousTime));
        //Note: the kickup and shooter RPMs lie to you, put an absolute encoder on the flywheel plz
        // there should be one on the flywheel and kicker now, ignore previous note
        Logger.recordOutput("Shooter/KickupRPM", Math.abs(kickupEncoder.getVelocity()));
        Logger.recordOutput("Shooter/ShooterRPM", Math.abs(flyVelocity));
        Logger.recordOutput("Shooter/ShooterRotations", flyPreviousPosition);
        Logger.recordOutput("Shooter/ShooterAmps", shootMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/KickupAmps", kickMotor.getOutputCurrent());
        flyPreviousPosition = flyCurrentPosition;
        //kickPreviousPosition = kickCurrentPosition; 
        previousTime = time;
    }
}

