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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggableTunedNumber;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ShooterS extends SubsystemBase{
    private final SparkMax kickMotor = new SparkMax(Constants.ShooterConstants.shooterMotorID, MotorType.kBrushless);
    private final SparkMax shootMotor = new SparkMax(Constants.ShooterConstants.shooterMotor2ID, MotorType.kBrushless);
    private PIDController flyController = new PIDController(1, 0, 0);
    private final LoggableTunedNumber kP = new LoggableTunedNumber("Shooter/kP",Constants.ShooterConstants.flyPID[0],true);
    private final LoggableTunedNumber kI = new LoggableTunedNumber("Shooter/kI",Constants.ShooterConstants.flyPID[1],true);
    private final LoggableTunedNumber kD = new LoggableTunedNumber("Shooter/kD",Constants.ShooterConstants.flyPID[2],true);
    private final LoggableTunedNumber kff = new LoggableTunedNumber("Shooter/kF",0,true);

    private final LoggableTunedNumber volts = new LoggableTunedNumber("Shooter/volts",0,true);
    private final LoggableTunedNumber rpm = new LoggableTunedNumber("Shooter/rpm",0,true);


    //dio
    private final RelativeEncoder kickupEncoder = kickMotor.getEncoder();
    private final Encoder shooterencoder = new Encoder(2, 3, false, EncodingType.k1X);
    public static final SlewRateLimiter limiter = new SlewRateLimiter(2);
    // private final DutyCycleEncoder shooterEncoder = new DutyCycleEncoder(1);
    // double flyPreviousPosition = shooterEncoder.get();
    // double flyCumulativeRotations = 0;
    // double DesiredFlyVelocity;
    // double[] flyVelocities = new double[10];
    // double flymean = 0;
    // int period = 0;

    // double kickPreviousPosition = kickupEncoder.get();
    // double previousTime;
    // public double flyVelocity;
    // public double kickVelocity;
    // rpms in volts / rpms
    // so if 1 volt gives 300 rpm the number is 300, as 1/300 volts would then give 1 rpm
    // private double kickvoltageconstant = 500;

    public ShooterS(){
        SparkMaxConfig config = new SparkMaxConfig();   
        config.voltageCompensation(12);
        //make higher if motor no work
        config.smartCurrentLimit(50);
        config.idleMode(IdleMode.kCoast);
        kickMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shootMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterencoder.setDistancePerPulse(1);
        shooterencoder.setSamplesToAverage(127);
    }

    
    /**
    * if you are using this something has gone horribly wrong
    */
    public void setVoltage(double kickupVoltage, double flyVolts){
        kickupVoltage = MathUtil.clamp(kickupVoltage, -12, 12);
        flyVolts = MathUtil.clamp(flyVolts, -12, 12);
        kickMotor.setVoltage(kickupVoltage);
        shootMotor.setVoltage(flyVolts);
    }

    public void fire(double kickupVoltage, double flySpeed){
        Robot.firing = true;
        if (getShooterRPM() > flySpeed - 300 && getShooterRPM() < flySpeed + 300){
            double flywheelVoltage = MathUtil.clamp(flyController.calculate(getShooterRPM(), flySpeed) + Constants.ShooterConstants.flyFF * flySpeed, -12, 3);
            flywheelVoltage = limiter.calculate(flywheelVoltage);
            kickMotor.setVoltage(kickupVoltage); 
            shootMotor.setVoltage(flywheelVoltage);
        } else {
            double flywheelVoltage = MathUtil.clamp(flyController.calculate(getShooterRPM(), flySpeed) + Constants.ShooterConstants.flyFF * flySpeed, -12, 3);
            flywheelVoltage = limiter.calculate(flywheelVoltage);
            shootMotor.setVoltage(flywheelVoltage);
        }
    }

    public void fireControlledSpeed(double kickupVoltage){
        Robot.firing = true;
        if (volts.get() == 0 && rpm.get() == 0 && getShooterRPM() > getShooterSetpointRPM() - 300 && getShooterRPM() < getShooterSetpointRPM() + 300){
            double flywheelVoltage = MathUtil.clamp(flyController.calculate(getShooterRPM(), getShooterSetpointRPM()) + Constants.ShooterConstants.flyFF * getShooterSetpointRPM(), -12, 3);
            flywheelVoltage = limiter.calculate(flywheelVoltage);
            kickMotor.setVoltage(kickupVoltage); 
            shootMotor.setVoltage(flywheelVoltage);
        } else if (volts.get() > 0){
            shootMotor.setVoltage(volts.get());
        } else if (rpm.get() > 0){
            double grpm = -1 * Math.abs(rpm.get());
            double flywheelVoltage = MathUtil.clamp(flyController.calculate(getShooterRPM(), grpm) + Constants.ShooterConstants.flyFF * grpm, -12, 3);
            flywheelVoltage = limiter.calculate(flywheelVoltage);
            shootMotor.setVoltage(flywheelVoltage);
            kickMotor.setVoltage(kickupVoltage);
        } else {
            spinup();
        }
    }

    public void spinup(){
        double flywheelVoltage = MathUtil.clamp(flyController.calculate(getShooterRPM(), getShooterSetpointRPM()) + Constants.ShooterConstants.flyFF * getShooterSetpointRPM(), -12, 3);
        kickMotor.setVoltage(0); 
        shootMotor.setVoltage(flywheelVoltage);
    }
    public void idle(double idleVoltage){
        Robot.firing = false;
        kickMotor.setVoltage(0);
        //idleVoltage = MathUtil.clamp(idleVoltage, Math.abs(Constants.ShooterConstants.flywheelRPM)/Constants.ShooterConstants.flyvoltageconstant, Math.abs(Constants.ShooterConstants.flywheelRPM)/Constants.ShooterConstants.flyvoltageconstant);
        idleVoltage = -1 * Math.abs(idleVoltage);
        shootMotor.setVoltage(idleVoltage);
    }

    public void stop(){
        Robot.firing = false;
        kickMotor.setVoltage(0);
        shootMotor.setVoltage(0);
    }

    //this is to try and make the shooter shoot with more power the further it is from the hub
    //since changes in x would result in not hitting the hub we only consider a y value (distance)
    public double getShooterSetpointRPM(){
        double distM = 1 * RobotContainer.vis.getHubDistanceFieldRelative();
        double desiredSpeed = MathUtil.clamp(( -652.4 * distM - 1615.5), -5000, -500);
        return desiredSpeed;
    }

    public double getShooterRPM(){
        return -1 * (shooterencoder.getRate() / 2048) * 60;
    }

    public void shootAtHub(){
        if (getShooterRPM() > -2850 && getShooterRPM() < -2250){
            double flywheelVoltage = MathUtil.clamp(flyController.calculate(getShooterRPM(), -2550) + Constants.ShooterConstants.flyFF * -2550, -12, 3);
            flywheelVoltage = limiter.calculate(flywheelVoltage);
            kickMotor.setVoltage(Constants.ShooterConstants.constantKickupVoltage); 
            shootMotor.setVoltage(flywheelVoltage);
        } else {
            double flywheelVoltage = MathUtil.clamp(flyController.calculate(getShooterRPM(), -2550) + Constants.ShooterConstants.flyFF * -2550, -12, 3);
            flywheelVoltage = limiter.calculate(flywheelVoltage);
            shootMotor.setVoltage(flywheelVoltage);
        }
    }

    public void periodic(){
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            flyController = new PIDController(kP.get(), kI.get(), kD.get());
        }, kP, kI, kD);

        Logger.recordOutput("Shooter/KickupRPM", Math.abs(kickupEncoder.getVelocity()));
        Logger.recordOutput("Shooter/ShooterRPM", getShooterRPM());
        Logger.recordOutput("Shooter/ShooterRate", Math.abs(shooterencoder.getRate()));
        Logger.recordOutput("Shooter/ShooterRaw", Math.abs(shooterencoder.getRaw()));
        Logger.recordOutput("Shooter/Distance", shooterencoder.getDistance());
        Logger.recordOutput("Shooter/ShooterAmps", shootMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/KickupAmps", kickMotor.getOutputCurrent());

        Logger.recordOutput("Shooter/flySetSpeed", getShooterSetpointRPM());

        // double flyCurrentPosition = shooterEncoder.get(); 
        //double kickCurrentPosition = kickupEncoder.get();
        //double time = Timer.getFPGATimestamp();
        // delta position over delta time
        
        //flyCurrentPosition = shooterEncoder.get();

        // if (flyCurrentPosition - flyPreviousPosition < 0){
        //     flyPreviousPosition -= 1;
        // } else if (flyPreviousPosition - flyCurrentPosition < 0){
        //     flyCurrentPosition -= 1 ;
        // }

        // if(flyCurrentPosition > 0.75 && flyPreviousPosition < 0.25){
        //     flyPreviousPosition++;
        // } else if(flyPreviousPosition > 0.75 && flyCurrentPosition < 0.25) {
        //     flyPreviousPosition--;
        // }

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

        // flyVelocity = (flyCurrentPosition - flyPreviousPosition) / (time - previousTime);
        // for(int i = 0; i < flyVelocities.length-1; i++){
        //     flyVelocities[i] = flyVelocities[i+1];
        // }
        // flyVelocities[flyVelocities.length - 1] = -1*flyVelocity;

        // double[] flyVelocitiesCopy = Arrays.copyOf(flyVelocities, flyVelocities.length);
        // Arrays.sort(flyVelocitiesCopy);
        // double medianVel = flyVelocitiesCopy[(int) (flyVelocitiesCopy.length/2)];
        
        

        //kickVelocity = ((kickCurrentPosition - kickPreviousPosition) / (time - previousTime));
        //Note: the kickup and shooter RPMs lie to you, put an absolute encoder on the flywheel plz
        // there should be one on the flywheel and kicker now, ignore previous note

        // flyPreviousPosition = flyCurrentPosition;
        // kickPreviousPosition = kickCurrentPosition; 
        // previousTime = time;
    }
}

