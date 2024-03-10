package frc.robot;

import javax.swing.ActionMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.TurnTestCommand;
import frc.robot.commands.TurnTestVisionCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.FreshmenTurnToAprilTagCommand;
import frc.robot.commands.FreshmenTurnToTargetCommand;
import frc.robot.commands.NoteDetection;
import frc.robot.commands.TargetedDriveCommand;
import frc.robot.commands.ridayCode;
import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class RobotContainer {

  public static DriveSubsystem drive = new DriveSubsystem();
  public static ArmSubsystem arm = new ArmSubsystem();
  // public static JoystickSubsystem joysticks = new JoystickSubsystem();
  public static CommandJoystick controller = new CommandJoystick(2);

  // private static TalonFX armPivot = new TalonFX(13);
  // private static TalonFX armPivot2 = new TalonFX(15);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    controller.button(12).whileTrue(new InstantCommand(
      () -> drive.setGyro(0)
    ));
    SmartDashboard.putData(new DriveCommand(drive, controller::getX, controller::getY, controller::getTwist));
    // SmartDashboard.putData(new FreshmenTurnToTargetCommand(0));
    // SmartDashboard.putData(new FreshmenTurnToAprilTagCommand());
    // SmartDashboard.putData(new fridayCode());
    SmartDashboard.putData(new TurnToAngleCommand(drive));
    SmartDashboard.putData(new TargetedDriveCommand(drive, controller::getX, controller::getY, controller::getTwist));
    SmartDashboard.putData(new NoteDetection());
    SmartDashboard.putData(new DriveToNote());
    SmartDashboard.putData(new InstantCommand(() -> {
      drive.resetOdometry(AprilTagManager.getRobotPos().toPose2d());
    }));
  }

}
