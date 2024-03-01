package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LilBuddyAzul extends Command {
    public TalonFX roller;

    @Override
    public void initSendable(SendableBuilder builder) {
        roller = new TalonFX(14);
        SmartDashboard.putNumber("Set Speed", 0);

    }

    @Override
    public void end(boolean interrupted) {
        double speed = SmartDashboard.getNumber("Default Value", 0);
        roller.set(speed);

    }
}

