package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FreshmenTurnToAprilTagCommand;
import frc.robot.commands.FreshmenTurnToTargetCommand;
import frc.robot.commands.fridayCode;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class RobotContainer {

  public static DriveSubsystem drive = new DriveSubsystem();
  public static JoystickSubsystem joysticks = new JoystickSubsystem();

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    SmartDashboard.putData(new DriveCommand(drive, joysticks));
    SmartDashboard.putData(new FreshmenTurnToTargetCommand(0));
    SmartDashboard.putData(new FreshmenTurnToAprilTagCommand());
    SmartDashboard.putData(new fridayCode());

  }

}
