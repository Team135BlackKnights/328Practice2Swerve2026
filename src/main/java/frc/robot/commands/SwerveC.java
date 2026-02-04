package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

//import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
//import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveS;



public class SwerveC extends Command{
    boolean isFinished = false;
    final SwerveS m_Swerve;
    

    public SwerveC(SwerveS subsystem){
        addRequirements(subsystem);
        m_Swerve = subsystem;
    }
    
    @Override
    public void execute(){
        //i dont think units are wrong here, they're in m/s and that's cool

        double x = -RobotContainer.m_driverController.getLeftX();
        double y = RobotContainer.m_driverController.getLeftY();
        double angle = Math.atan2(y,x);
        double magnitude = Math.hypot(x, y);
        double a = 0.6;
        magnitude = a*Math.pow(magnitude,5)+magnitude*(1-a);
        x = Math.cos(angle) * magnitude;
        y = Math.sin(angle) * magnitude;

        double rot = RobotContainer.m_driverController.getRightX();
        rot = a*Math.pow(rot,5)+rot*(1-a);

        // m_Swerve.setSpeed(-3*x, 3*y, 10*rot);
        final ChassisSpeeds cspeeds = RobotContainer.fieldOrientedDrive(x,y,rot);
        m_Swerve.setSpeed(Constants.SwerveConstants.linearSpeedMultiplier*cspeeds.vxMetersPerSecond, Constants.SwerveConstants.linearSpeedMultiplier*cspeeds.vyMetersPerSecond, Constants.SwerveConstants.radianSpeedMultiplier*cspeeds.omegaRadiansPerSecond);

    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
