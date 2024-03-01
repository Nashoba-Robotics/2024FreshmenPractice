package frc.robot.commands;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;


public class SashaTriesToLightACANdle extends Command{

CANdle candle;
Animation frontL;
Animation frontR;
Animation backL;
Animation backR;    
    
public SashaTriesToLightACANdle(){
    candle = new CANdle(0, "drivet");
    frontL = new RainbowAnimation(0.75, 0, 30, isFinished(), 8);
    frontR = new RainbowAnimation(0.1, 0, 30, isFinished(), 38);
    backR = new RainbowAnimation(0.1, 0, 30, isFinished(), 68);
    backL = new RainbowAnimation(0.75, 0, 30, isFinished(), 98);
}

public void fLRainbow(){
    candle.animate(frontL);
}

public void fRRainbow(){
    candle.animate(frontR);
}

@Override
public void execute() {
    fLRainbow();
}

@Override
public void end(boolean interrupted) {
    candle.clearAnimation(0);
}
@Override
public boolean isFinished() {
    return false;
}

}
