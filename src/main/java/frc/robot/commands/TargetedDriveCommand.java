package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.util.JoystickValues;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class TargetedDriveCommand extends Command{
    
    private DriveSubsystem drive;
    DoubleSupplier joystickX, joystickY, joystickOmega;

    private JoystickValues leftJoystickValues;
    private JoystickValues rightJoystickValues;

    private ChassisSpeeds chassisSpeeds;
    private static final Pose2d target = new Pose2d(0, 2, new Rotation2d());
    PIDController controller;

    public TargetedDriveCommand(DriveSubsystem drive, DoubleSupplier joystickX, DoubleSupplier joystickY, DoubleSupplier joystickOmega) {
        this.drive = drive;
        addRequirements(drive);
        chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        this.joystickX = joystickX;
        this.joystickY = joystickY;
        this.joystickOmega = joystickOmega;

        leftJoystickValues = new JoystickValues(joystickX.getAsDouble(), -joystickY.getAsDouble());
        rightJoystickValues = new JoystickValues(joystickOmega.getAsDouble(), 0);

        controller = new PIDController(0.1, 0, 0);
    }

    @Override
    public void initialize() {
        chassisSpeeds.vxMetersPerSecond = 0;
        chassisSpeeds.vyMetersPerSecond = 0;
        chassisSpeeds.omegaRadiansPerSecond = 0;
        drive.set(chassisSpeeds);
    }

    @Override
    public void execute() {
        leftJoystickValues = new JoystickValues(joystickX, joystickY)
            .shape(Constants.Joystick.MOVE_DEAD_ZONE, Constants.Joystick.TURN_SENSITIVITY)
            .swap()
            .applyAngleDeadzone(Constants.Joystick.ANGLE_DEAD_ZONE);

        rightJoystickValues = new JoystickValues(joystickOmega.getAsDouble(), 0)
            .shape(Constants.Joystick.TURN_DEAD_ZONE, Constants.Joystick.TURN_SENSITIVITY);

        chassisSpeeds.vxMetersPerSecond = leftJoystickValues.x * Constants.Drive.MAX_VELOCITY;
        chassisSpeeds.vyMetersPerSecond = leftJoystickValues.y * Constants.Drive.MAX_VELOCITY;

        // chassisSpeeds.omegaRadiansPerSecond = -rightJoystickValues.x * Constants.Drive.MAX_ROTATION_VELOCITY;
        Pose2d currPose = drive.getPose();
        double targetAngle = Math.atan2(currPose.getY()-target.getY(), currPose.getX()-target.getX());
        targetAngle *= 180./Math.PI;
        
        if(Math.abs(rightJoystickValues.x) <= 0.05) chassisSpeeds.omegaRadiansPerSecond = controller.calculate(drive.getGyroAngle().getDegrees(), targetAngle);
        else chassisSpeeds.omegaRadiansPerSecond = -rightJoystickValues.x * Constants.Drive.MAX_ROTATION_VELOCITY;
        
        drive.set(chassisSpeeds);

    }

    @Override
    public void end(boolean interrupted) {
        // drive.set(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
