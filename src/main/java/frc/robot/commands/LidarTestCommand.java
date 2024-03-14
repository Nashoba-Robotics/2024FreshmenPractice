package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LidarTestCommand extends Command {
    
    public Counter counter;
    public DigitalInput source;

    public Timer timer;

    public double lastDist;
    public double currentDist;
    public double lastTime;
    public double currentTime;
    public LinearFilter filter;

    public double velocity;

    public LidarTestCommand(){
        source = new DigitalInput(0);
        counter = new Counter(source);
        counter.setMaxPeriod(1);
        counter.setSemiPeriodMode(true);
        counter.reset();

        filter = LinearFilter.movingAverage(20);

        timer = new Timer();
        timer.reset();
    }

    public double getDist(){
        if(counter.get() < 1){
            return 0;
        }
        else{
        return filter.calculate((counter.getPeriod()*1000000 / 10) / 100);
        }
    }
    
    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {

        lastDist = currentDist;
        currentDist = getDist();

        lastTime = currentTime;
        currentTime = timer.get();

        velocity = Math.abs((currentDist - lastDist) / (currentTime - lastTime));

        SmartDashboard.putNumber("Object Dist", getDist());
        SmartDashboard.putNumber("Object Velocity", velocity);

    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
