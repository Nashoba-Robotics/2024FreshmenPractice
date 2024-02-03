package frc.robot.commands;

import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class FreshmenTurnToTargetCommand extends Command {
    public DriveSubsystem drive = RobotContainer.drive;
    public PIDController controller;
    public double lastKnownGyro;
    public double currentGyro;
    public double targetAngle;
    public FreshmenTurnToTargetCommand(double targetAngle){
        this.targetAngle = targetAngle;
        controller = new PIDController(1.6, 0, 0.09);
        controller.setTolerance(0.05);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive); 
    }


public void initialize(){
    controller.setSetpoint(targetAngle);

    }

public void execute(){
    ChassisSpeeds speed = new ChassisSpeeds();
    double PIDoutput;
    if(!controller.atSetpoint()) {
        currentGyro = drive.getGyroAngle().getRadians();
        PIDoutput = controller.calculate(currentGyro);
        speed.omegaRadiansPerSecond = PIDoutput;
    }

    else PIDoutput = 0;

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