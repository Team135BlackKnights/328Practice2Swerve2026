package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
//import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LoggableTunedNumber;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final TalonFX turnMotor; 
    private final TalonFX driveMotor; 
    private final CANcoder turnEncoder; 
    private final double offsetRadians;
    private SwerveModuleState desiredState;
    public double turnVoltage; 
    private int currentRotation = 0;
    private double currentPos = 0;
    
    private PIDController turnController = new PIDController(SwerveConstants.turnPID[0], SwerveConstants.turnPID[1], SwerveConstants.turnPID[2]);
    private PIDController driveController = new PIDController(SwerveConstants.drivePID[0], SwerveConstants.drivePID[1], SwerveConstants.drivePID[2]);
    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(Constants.SwerveConstants.drivePID[3], Constants.SwerveConstants.drivePID[4]);
    private final LoggableTunedNumber turnkP = new LoggableTunedNumber("turn/kP",Constants.SwerveConstants.turnPID[0],true);
    private final LoggableTunedNumber turnkI = new LoggableTunedNumber("turn/kI",Constants.SwerveConstants.turnPID[1],true);
    private final LoggableTunedNumber turnkD = new LoggableTunedNumber("turn/kD",Constants.SwerveConstants.turnPID[2],true);
    private final LoggableTunedNumber drivekP = new LoggableTunedNumber("drive/kP",Constants.SwerveConstants.drivePID[0],true);
    private final LoggableTunedNumber drivekI = new LoggableTunedNumber("drive/kI",Constants.SwerveConstants.drivePID[1],true);
    private final LoggableTunedNumber drivekD = new LoggableTunedNumber("drive/kD",Constants.SwerveConstants.drivePID[2],true);
    private final LoggableTunedNumber drivekS = new LoggableTunedNumber("drive/kS",Constants.SwerveConstants.drivePID[3],true);
    private final LoggableTunedNumber drivekV = new LoggableTunedNumber("drive/kV",Constants.SwerveConstants.drivePID[4],true);
    
    public void periodic(){
        double updatedPos = getTurnPosition();
        int updatedRegion = (int) (updatedPos * 3);
        int oldRegion = (int) (currentPos * 3);

        if(oldRegion == 0 && updatedRegion == 2){
            currentRotation --;
        } else if(oldRegion == 2 && updatedRegion == 0){
            currentRotation ++;
        }

        currentPos = updatedPos;
    }   

    public SwerveModule(int turnID, InvertedValue flip, int driveID, int encoderID, double offset, CANBus bus){
        turnMotor = new TalonFX(turnID, bus);
        driveMotor = new TalonFX(driveID, bus);
        turnEncoder = new CANcoder(encoderID, bus);
        offsetRadians = offset;

        turnController.enableContinuousInput(-Math.PI, Math.PI);

        final TalonFXConfiguration turnConfigs = new TalonFXConfiguration().withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(SwerveConstants.turnNeutralMode)
                .withInverted(SwerveConstants.turnInversion)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(80))
                .withStatorCurrentLimitEnable(true)
        );

        TalonFXConfigurator turnConfigurator = turnMotor.getConfigurator();
        turnConfigurator.apply(turnConfigs); 

        final TalonFXConfiguration driveConfigs = new TalonFXConfiguration().withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(SwerveConstants.driveNeutralMode)
                .withInverted(flip)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(80))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(30))
                .withSupplyCurrentLimitEnable(true)
        );

        TalonFXConfigurator driveConfigurator = driveMotor.getConfigurator();
        driveConfigurator.apply(driveConfigs); 

        desiredState = new SwerveModuleState(0, new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - offsetRadians));
    }

    public void setDriveVoltage(double voltage){
        driveMotor.setVoltage(voltage);
    }

    public void setTurnVoltage(double voltage){
        this.turnVoltage = voltage;
        turnMotor.setVoltage(voltage);
    }

    public double getTurnPosition(){
        return turnEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public Rotation2d getTurnPositionRotation2D(){
        return new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
    }
    
    public double getPositionMeters(){
        return driveMotor.getPosition().getValueAsDouble() * 2 * Math.PI * SwerveConstants.gearRatioSpeed * Constants.SwerveConstants.wheelRadiusMeters;
    }

    public double getPositionCumulative(){
        return currentPos + currentRotation;
    }

    public double getDriveSpeedMetersPerSecond(){
        return driveMotor.getVelocity().getValueAsDouble() * 2 * Math.PI * SwerveConstants.gearRatioSpeed * SwerveConstants.wheelRadiusMeters;
    }

    public void setDesiredModuleState(SwerveModuleState moduleState){
        desiredState = moduleState;
    }

    // public void updateStateBangBang(double driveVoltage, double turnVoltage){
    //     double currentDriveVelocity = driveMotor.getVelocity().getValueAsDouble();
    //     double currentTurnPosition = turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    //     double driveVelocityError = desiredState.speedMetersPerSecond - currentDriveVelocity;
    //     double angularError = desiredState.angle.getRadians() - currentTurnPosition; 
    //     if(angularError < -0.01){
    //         turnMotor.setVoltage(turnVoltage);
    //     } else if(angularError > 0.01){
    //         turnMotor.setVoltage(-turnVoltage);
    //     } else {
    //         turnMotor.setVoltage(0);
    //     }
    //       if(driveVelocityError < -0.01){
    //         driveMotor.setVoltage(driveVoltage);
    //     } else if(driveVelocityError > 0.01){
    //         driveMotor.setVoltage(-driveVoltage);
    //     } else {
    //         driveMotor.setVoltage(0);
    //     }
    // }

    public void updateStatePID(){
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            turnController = new PIDController(turnkP.get(), turnkI.get(), turnkD.get());
            turnController.enableContinuousInput(-Math.PI, Math.PI);
        }, turnkP, turnkI, turnkD);
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            driveController = new PIDController(drivekP.get(), drivekI.get(), drivekD.get());
            driveFF = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());
        }, drivekP, drivekI, drivekD, drivekS,drivekV);


        double currentDriveVelocity = getDriveSpeedMetersPerSecond();
        double currentTurnPosition = turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - offsetRadians;


        desiredState.optimize(new Rotation2d(currentTurnPosition));
        double speed = desiredState.speedMetersPerSecond;
        double angle = desiredState.angle.getRadians();
        speed = speed * Math.cos(currentTurnPosition-angle);

        SwerveModuleState optimizedDesiredState = new SwerveModuleState(speed, new Rotation2d(angle));
        
        // Rotation2d currentRotation =new Rotation2d(currentTurnPosition);
        // System.out.println(currentRotation);
        // System.out.println(optimizedDesiredState.angle);
        // System.out.println("hiiiiiii");
        // optimizedDesiredState.optimize(currentRotation);
        turnController.setSetpoint(optimizedDesiredState.angle.getRadians());//yo this is wrong
        driveController.setSetpoint(optimizedDesiredState.speedMetersPerSecond);
        double ffVolts = driveFF.calculateWithVelocities(currentDriveVelocity, optimizedDesiredState.speedMetersPerSecond);
        setTurnVoltage(turnController.calculate(currentTurnPosition));
        setDriveVoltage(driveController.calculate(currentDriveVelocity) + ffVolts);
        Logger.recordOutput("turn encoder", turnEncoder.getAbsolutePosition().getValueAsDouble());
        Logger.recordOutput("turn voltage", turnController.calculate(currentTurnPosition));
        Logger.recordOutput("drive voltage", driveController.calculate(currentDriveVelocity));
        Logger.recordOutput("turn setpoint", optimizedDesiredState.angle.getRadians());
        Logger.recordOutput("drive setpoint", optimizedDesiredState.speedMetersPerSecond);
        Logger.recordOutput("drive vel", currentDriveVelocity);
        Logger.recordOutput("drive ff", ffVolts);

    }



	public static double closerAngleToZero(Rotation2d angle) {
		// Normalize the angle to be within the range of -180 to 180 degrees
		double angleDegrees = angle.getDegrees();
		double normalizedAngle = MathUtil.inputModulus(angleDegrees, -180, 180);
		return normalizedAngle;
	}
}
