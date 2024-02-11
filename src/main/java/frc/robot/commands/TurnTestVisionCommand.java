package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.apriltags.AprilTagManager;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TurnTestVisionCommand extends Command{
    
    private DriveSubsystem drive;

    TrapezoidProfile control;
    PIDController pidController;

    Timer t;

    State startState;
    // State lastState;

    public TurnTestVisionCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);

        control = new TrapezoidProfile(new Constraints(2, 4));
        pidController = new PIDController(0.09, 0, 0.008);
        pidController.setTolerance(0.05);
        t = new Timer();
    }

    @Override
    public void initialize() {
        startState = new State(drive.getGyroAngle().getRadians(), 0);
        // lastState = startState;
        t.restart();
    }

    @Override
    public void execute() {
        State state = control.calculate(t.get(), startState, new State(0, 0));
        state.velocity += pidController.calculate(drive.getYaw().getRadians(), state.position);
        Logger.recordOutput("GoalPosition", state.position);
        Logger.recordOutput("GoalSpeed", state.velocity);

        if(!pidController.atSetpoint())drive.set(0, 0, state.velocity);

        // lastState = state;
    }


}
