package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
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
  }

}
