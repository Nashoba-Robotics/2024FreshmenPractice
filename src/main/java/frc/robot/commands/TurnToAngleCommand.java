package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TurnToAngleCommand extends Command{
    DriveSubsystem drive;
    private PIDController controller;
    private ProfiledPIDController profiledController;

    private double startAngle;

       
    public TurnToAngleCommand(DriveSubsystem drive){
        this.drive = drive;
        controller = new PIDController(0.1, 0, 0);
        profiledController = new ProfiledPIDController(0.2, 0, 0, new Constraints(2, 1));
    }

    @Override
    public void initialize() {
        // startAngle = drive.getGyroAngle().getDegrees();
        // SmartDashboard.putNumber("Start Angel", startAngle);
        drive.setGyro(0);
    }

    @Override
    public void execute() {
        double currentAngle = drive.getGyroAngle().getDegrees();
        SmartDashboard.putNumber("Current Angel", currentAngle);
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
        speeds.omegaRadiansPerSecond = controller.calculate(currentAngle, 90);
        drive.set(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.set(0, 0, 0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}