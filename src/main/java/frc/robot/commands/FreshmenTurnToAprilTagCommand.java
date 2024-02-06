package frc.robot.commands;

import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class FreshmenTurnToAprilTagCommand extends Command {
    public DriveSubsystem drive = RobotContainer.drive;
    public PIDController controller;
    public double currentGyro;
    public double targetAngle;
    public double lastKnownGyro;
    public FreshmenTurnToAprilTagCommand(){
        controller = new PIDController(3.7, 0, 0);
        controller.setTolerance(0.03);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive); 
    }


public void initialize(){
    controller.setSetpoint(0);
}

public void execute(){
    ChassisSpeeds speed = new ChassisSpeeds();
    double PIDoutput;
    if(AprilTagManager.hasTarget()) {
        lastKnownGyro = drive.getGyroAngle().getRadians();
        targetAngle = AprilTagManager.getTargetYaw();
        PIDoutput = controller.calculate(targetAngle);
        speed.omegaRadiansPerSecond = PIDoutput;
    }
  
    else {
        currentGyro = drive.getGyroAngle().getRadians();
        PIDoutput = controller.calculate(currentGyro - lastKnownGyro);
        speed.omegaRadiansPerSecond = PIDoutput; 
    }
        
    drive.set(speed);
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
    return false;
}

}