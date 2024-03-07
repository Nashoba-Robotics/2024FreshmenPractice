package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class noteDetection extends Command{
    public PhotonCamera camera = new PhotonCamera("HD_USB_Camera");
    public PhotonPipelineResult result = camera.getLatestResult();
    public DriveSubsystem drive = RobotContainer.drive;
    public boolean hasNote;
    public double yaw;
    public PhotonTrackedTarget target;
    public PIDController controller;
    public double lastKnownGyro;
    
    public noteDetection(){
        controller= new PIDController(3.7, 0, 0);
        controller.setTolerance(0.03);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }
    @Override
    public void initialize() {
       controller.setSetpoint(0);
       hasNote = result.hasTargets();
    
    }
    @Override
    public void execute() {
        ChassisSpeeds speed = new ChassisSpeeds();
        double PIDoutput;
        if(hasNote){
            lastKnownGyro=drive.getGyroAngle().getRadians();
            target = result.getBestTarget();
            yaw = target.getYaw(); 
            PIDoutput = controller.calculate(yaw);
            speed.omegaRadiansPerSecond=PIDoutput;
            
        }
        
        drive.set(speed);
    }
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds stop = new ChassisSpeeds();
        stop.omegaRadiansPerSecond = 0;
        drive.set(stop);
    }
}
