package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.CAN;
import frc.robot.Constants.SwerveConstants;


public class SwerveModule {
    private final TalonFX turnMotor; 
    private final TalonFX driveMotor; 
    private final CANcoder turnEncoder; 
    private final double offsetRadians;
    private SwerveModuleState desiredState; 
    
    private PIDController turnController = new PIDController(SwerveConstants.turnPID[0], SwerveConstants.turnPID[1], SwerveConstants.turnPID[2]);
    private PIDController driveController = new PIDController(SwerveConstants.drivePID[0], SwerveConstants.drivePID[1], SwerveConstants.drivePID[2]);

    public SwerveModule(int turnID, int driveID, int encoderID, double offset, CANBus bus){
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
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
        );

        TalonFXConfigurator turnConfigurator = turnMotor.getConfigurator();
        turnConfigurator.apply(turnConfigs); 

        final TalonFXConfiguration driveConfigs = new TalonFXConfiguration().withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(SwerveConstants.driveNeutralMode)
                .withInverted(SwerveConstants.driveInversion)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
        );

        TalonFXConfigurator driveConfigurator = driveMotor.getConfigurator();
        driveConfigurator.apply(driveConfigs); 

        desiredState = new SwerveModuleState(0, new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - offsetRadians));
    }

    public void setDriveVoltage(double voltage){
        driveMotor.setVoltage(voltage);
    }

    public void setTurnVoltage(double voltage){
        turnMotor.setVoltage(voltage);
    }

    public double getTurnPosition(){
        return turnEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setDesiredModuleState(SwerveModuleState moduleState){
        desiredState = moduleState;
    }

    // public void updateStateBangBang(double driveVoltage, double turnVoltage){
    //     double currentDriveVelocity = driveMotor.getVelocity().getValueAsDouble();
    //     double currentTurnPosition = turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    //     double driveVelocityError = desiredState.speedMetersPerSecond - currentDriveVelocity;//TODO fix +- so it dont go backwards also units
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
        double currentDriveVelocity = driveMotor.getVelocity().getValueAsDouble() * 2 * Math.PI * SwerveConstants.gearRatioSpeed * SwerveConstants.wheelRadius;//5.9:1 and 2 in
        double currentTurnPosition = turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - offsetRadians;
        SwerveModuleState optimizedDesiredState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        optimizedDesiredState.optimize(new Rotation2d(currentTurnPosition));

        turnController.setSetpoint(optimizedDesiredState.angle.getRadians());
        driveController.setSetpoint(optimizedDesiredState.speedMetersPerSecond);

        turnMotor.setVoltage(turnController.calculate(currentTurnPosition));
        driveMotor.setVoltage(driveController.calculate(currentDriveVelocity));


    }



}
