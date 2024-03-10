package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToNote extends Command{
    public DriveSubsystem drive= RobotContainer.drive;
    public PIDController controller;
    public ChassisSpeeds speed;
    public PhotonCamera camera = new PhotonCamera("HD_USB_Camera");
    public double yaw;
    public PhotonPipelineResult result;
    public DriveToNote(){
        controller = new PIDController(0, 0, 0);
        controller.setTolerance(0.02);
        addRequirements(drive);
    
    }
    @Override
    public void initialize() {
        controller.setSetpoint(0);
        
    }
    @Override
    public void execute() {
        result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        yaw = target.getYaw(); 
        boolean hasNote = result.hasTargets();
        if (hasNote){  
            Logger.recordOutput("Yaws", yaw);
            double x = 0.08 * Math.sin(yaw);
            double y = 0.08 * Math.cos(yaw);
            speed = new ChassisSpeeds(x, y, 0);
            drive.set(speed);
        }
        
        
        
    }
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds stop = new ChassisSpeeds();
        stop.omegaRadiansPerSecond=0;
        drive.set(stop);
    }
    
       
}

    

