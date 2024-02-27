package frc.robot.commands;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;


public class LilBuddyit  extends Command{
    public TalonFX roller;
    
    @Override
    public void initialize() {
        roller = new TalonFX(14);
        roller.set(.1);
    }
    @Override
    public void end(boolean interrupted) {
        roller.set(0);
    }
}