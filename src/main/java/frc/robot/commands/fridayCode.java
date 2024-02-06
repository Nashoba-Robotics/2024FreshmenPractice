package frc.robot.commands;

import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.drive.DriveSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class fridayCode extends Command{
    public DriveSubsystem drive = RobotContainer.drive;
    public PIDController controller; 
    public double lastKnownGyro;
    public double currentGyro;
    public double targetAngle;
    public fridayCode(){
        controller = new PIDController(3.7, 0, 0.003);
        addRequirements(drive);
    }

    public void initialize(){
        controller.setSetpoint(0);
        lastKnownGyro = drive.getGyroAngle().getRadians();
    
    }

    public void execute(){
        ChassisSpeeds somthingAboutSpeed = new ChassisSpeeds();
        double PIDoutput;
        if(AprilTagManager.hasTarget()) {
            controller.setSetpoint(0);
            lastKnownGyro = drive.getGyroAngle().getRadians();
            targetAngle = AprilTagManager.getTargetYaw();
            PIDoutput = controller.calculate(targetAngle);
            somthingAboutSpeed.omegaRadiansPerSecond = PIDoutput;
        }

        else{
            controller.setSetpoint(-targetAngle);
            currentGyro = drive.getGyroAngle().getRadians();
            PIDoutput = controller.calculate(currentGyro - lastKnownGyro);
            somthingAboutSpeed.omegaRadiansPerSecond = PIDoutput;
        }

        drive.set(somthingAboutSpeed);
         Logger.recordOutput ("Freshmen Angle Thing", PIDoutput);
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds noMoreSpeed = new ChassisSpeeds();
        noMoreSpeed.omegaRadiansPerSecond = 0;
        drive.set (noMoreSpeed);
    }
    
    @Override
    public boolean isFinished() {
        // double angle = AprilTagManager.getTargetYaw();
        // if (angle==0) return true;
        // else return false;

        return false;
    }
    
}
