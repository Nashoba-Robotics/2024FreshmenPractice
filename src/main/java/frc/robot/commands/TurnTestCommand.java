package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TurnTestCommand extends Command{
    
    private DriveSubsystem drive;

    TrapezoidProfile control;
    PIDController pidController;

    Timer t;

    State startState;

    public TurnTestCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);

        control = new TrapezoidProfile(new Constraints(2, 4));
        pidController = new PIDController(0.09, 0, 0.008);
        pidController.setTolerance(0.05);
        t = new Timer();
    }

    @Override
    public void initialize() {
        startState = new State(drive.getYaw().getRadians(), 0);
        t.restart();
    }

    @Override
    public void execute() {
        State state = control.calculate(t.get(), startState, new State(startState.position + Constants.TAU/2, 0));
        state.velocity += pidController.calculate(drive.getYaw().getRadians(), state.position);
        Logger.recordOutput("GoalPosition", state.position);
        Logger.recordOutput("GoalSpeed", state.velocity);

        if(!pidController.atSetpoint())drive.set(0, 0, state.velocity);
    }


}
