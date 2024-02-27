package frc.robot.commands;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class LilBuddy  extends Command{
    public TalonFX roller;
    
    @Override
    public void initialize() {
        roller = new TalonFX(14);
        roller.set(.1);
        SmartDashboard.putNumber("Set Speed", 0);

    }
    @Override
    public void execute() {
        double speed=SmartDashboard.getNumber("Set Speed", 0);
        roller.set(speed);
    }
    @Override
    public void end(boolean interrupted) {
        roller.set(0);
    }
}