package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TurnTestCommand;
import frc.robot.commands.TurnTestVisionCommand;
import frc.robot.commands.FreshmenTurnToAprilTagCommand;
import frc.robot.commands.FreshmenTurnToTargetCommand;
import frc.robot.commands.fridayCode;
import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class RobotContainer {

  public static DriveSubsystem drive = new DriveSubsystem();
  public static JoystickSubsystem joysticks = new JoystickSubsystem();

  private static TalonFX armPivot = new TalonFX(13);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    SmartDashboard.putData(new DriveCommand(drive, joysticks));
    // SmartDashboard.putData(new FreshmenTurnToTargetCommand(0));
    SmartDashboard.putData(new FreshmenTurnToAprilTagCommand());
    // SmartDashboard.putData(new fridayCode());

    SmartDashboard.putData(new InstantCommand(() -> {
      drive.resetOdometry(AprilTagManager.getRobotPos().toPose2d());
    }));

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armPivot.getConfigurator().apply(config);
  }

}
