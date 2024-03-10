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


 /* Yi's instructions:
     *
     * In intialize, set robot's orientation to 0 
     *      Then read the current yaw of the target --
     * 
     * In execute, use a PID controller to set the robot's angle equal to the read yaw. --
     */

public class NoteDetection extends Command{
    public PhotonCamera camera = new PhotonCamera("HD_USB_Camera");
    public PhotonPipelineResult result = camera.getLatestResult();
    public DriveSubsystem drive = RobotContainer.drive;
    public boolean hasNote;
    public double yaw;
    public PhotonTrackedTarget target;
    public PIDController controller;

    
    public NoteDetection(){
        controller= new PIDController(0.0965, 0, 0);
        controller.setTolerance(0.02);
        controller.enableContinuousInput(-180, 180);
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
        result = camera.getLatestResult();
        hasNote = result.hasTargets();

        if(hasNote){
            target = result.getBestTarget();

            // result.getTargets();
            yaw = target.getYaw(); 
            double PIDoutput = controller.calculate(yaw);
            speed.omegaRadiansPerSecond=PIDoutput;
            Logger.recordOutput("Yaw", yaw);  
            Logger.recordOutput("PIDSpeed", PIDoutput);
            drive.set(speed);
        }
        else{
            speed.omegaRadiansPerSecond= 0;
            drive.set(speed);
            
        }

        
        
    }
    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds stop = new ChassisSpeeds();
        stop.omegaRadiansPerSecond = 0;
        drive.set(stop);
    }
}