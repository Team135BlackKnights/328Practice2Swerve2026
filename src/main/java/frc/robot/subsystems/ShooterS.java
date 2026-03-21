package frc.robot.subsystems;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.QuadratureConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggableTunedNumber;
import frc.robot.RobotContainer;

public class ShooterS extends SubsystemBase{
    private final SparkMax kickMotor = new SparkMax(Constants.ShooterConstants.shooterMotorID, MotorType.kBrushless);
    private final SparkMax shootMotor = new SparkMax(Constants.ShooterConstants.shooterMotor2ID, MotorType.kBrushless);
    private PIDController flyController = new PIDController(1, 0, 0);
    private final LoggableTunedNumber kP = new LoggableTunedNumber("Shooter/kP",Constants.ShooterConstants.flyPID[0],true);
    private final LoggableTunedNumber kI = new LoggableTunedNumber("Shooter/kI",Constants.ShooterConstants.flyPID[1],true);
    private final LoggableTunedNumber kD = new LoggableTunedNumber("Shooter/kD",Constants.ShooterConstants.flyPID[2],true);
    //dio channel
    private final RelativeEncoder kickupEncoder = kickMotor.getEncoder();
    private final Encoder shooterencoder = new Encoder(1, 2);
    private final DutyCycleEncoder shooterEncoder = new DutyCycleEncoder(1);
    double flyPreviousPosition = shooterEncoder.get();
    double flyCumulativeRotations = 0;
    double DesiredFlyVelocity;
    double[] flyVelocities = new double[10];
    double flymean = 0;
    int period = 0;

    //double kickPreviousPosition = kickupEncoder.get();
    double previousTime;
    public double flyVelocity;
    //public double kickVelocity;
    // rpms in volts / rpms
    // so if 1 volt gives 300 rpm the number is 300, as 1/300 volts would then give 1 rpm
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
        // converts to rad per sec
        DesiredFlyVelocity = desiredFlyVelocity;
        // double desiredFlyVelocityRadPS = Math.toRadians(desiredFlyVelocity*6);
        // double flyVelocityRadPS = Math.toRadians(flyVelocity*6);
        double flywheelVoltage = MathUtil.clamp(flyController.calculate(flyVelocity, desiredFlyVelocity)*12, -12, 12);
        kickMotor.setVoltage(kickupVoltage);
        shootMotor.setVoltage(flywheelVoltage);
    }

    public void idle(double idleVoltage){
        kickMotor.setVoltage(0);
        //idleVoltage = MathUtil.clamp(idleVoltage, Math.abs(Constants.ShooterConstants.flywheelRPM)/Constants.ShooterConstants.flyvoltageconstant, Math.abs(Constants.ShooterConstants.flywheelRPM)/Constants.ShooterConstants.flyvoltageconstant);
        shootMotor.setVoltage(idleVoltage);
    }

    public void stop(){
        kickMotor.setVoltage(0);
        shootMotor.setVoltage(0);
    }

    //this is to try and make the shooter shoot with more power the further it is from the hub
    //since changes in x would result in not hitting the hub we only consider a y value (distance)
    public double getShooterProportionalControlSpeed(){
        double distM = 1 * RobotContainer.vis.getHubDistanceFieldRelative();
        double desiredSpeed = MathUtil.clamp(( -1100 * distM - 176.6), -3000, -500);
        System.out.println("Desired Speed:" + desiredSpeed);
        return desiredSpeed;
    }


    public void periodic(){
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            flyController = new PIDController(kP.get(), kI.get(), kD.get());
        }, kP, kI, kD);
        double flyCurrentPosition = shooterEncoder.get(); 
        //double kickCurrentPosition = kickupEncoder.get();
        double time = Timer.getFPGATimestamp();
        // delta position over delta time
        
        flyCurrentPosition = shooterEncoder.get();

        // if (flyCurrentPosition - flyPreviousPosition < 0){
        //     flyPreviousPosition -= 1;
        // } else if (flyPreviousPosition - flyCurrentPosition < 0){
        //     flyCurrentPosition -= 1 ;
        // }

        if(flyCurrentPosition > 0.75 && flyPreviousPosition < 0.25){
            flyPreviousPosition++;
        } else if(flyPreviousPosition > 0.75 && flyCurrentPosition < 0.25) {
            flyPreviousPosition--;
        }

        // int updatedRegion = (int) (flyCurrentPosition * 3);
        // int oldRegion = (int) (flyPreviousPosition * 3);

        // if(oldRegion == 0 && updatedRegion == 2){
        //     flyCurrentPosition --;
        // } else if(oldRegion == 2 && updatedRegion == 0){
        //     flyCurrentPosition ++;
        // }

        // if (shooterEncoder.get() > 0.99){
        //     flyCumulativeRotations = flyCumulativeRotations + 1;
        // } else if (shooterEncoder.get() < 0.001){
        //     flyCumulativeRotations = flyCumulativeRotations + 1;
        // }

        flyVelocity = (flyCurrentPosition - flyPreviousPosition) / (time - previousTime);
        for(int i = 0; i < flyVelocities.length-1; i++){
            flyVelocities[i] = flyVelocities[i+1];
        }
        flyVelocities[flyVelocities.length - 1] = -1*flyVelocity;

        double[] flyVelocitiesCopy = Arrays.copyOf(flyVelocities, flyVelocities.length);
        Arrays.sort(flyVelocitiesCopy);
        double medianVel = flyVelocitiesCopy[(int) (flyVelocitiesCopy.length/2)];
        
        

        //kickVelocity = ((kickCurrentPosition - kickPreviousPosition) / (time - previousTime));
        //Note: the kickup and shooter RPMs lie to you, put an absolute encoder on the flywheel plz
        // there should be one on the flywheel and kicker now, ignore previous note
        Logger.recordOutput("Shooter/CumulativePos", flyCumulativeRotations);
        Logger.recordOutput("Shooter/KickupRPM", Math.abs(kickupEncoder.getVelocity()));
        Logger.recordOutput("Shooter/ShooterRPM", Math.abs(flyVelocity*60));
        Logger.recordOutput("Shooter/ShooterRotations", flyPreviousPosition);
        Logger.recordOutput("Shooter/ShooterAmps", shootMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/KickupAmps", kickMotor.getOutputCurrent());

        Logger.recordOutput("Shooter/flyCurrentPos", flyCurrentPosition);
        Logger.recordOutput("Shooter/flyPrevPos", flyPreviousPosition);
        Logger.recordOutput("Shooter/time", time);
        Logger.recordOutput("Shooter/prevtime", previousTime);
        Logger.recordOutput("Shooter/flymedvel", medianVel*60);

        Logger.recordOutput("Shooter/flySetSpeed", DesiredFlyVelocity);

        flyPreviousPosition = flyCurrentPosition;
        //kickPreviousPosition = kickCurrentPosition; 
        previousTime = time;
    }
}

