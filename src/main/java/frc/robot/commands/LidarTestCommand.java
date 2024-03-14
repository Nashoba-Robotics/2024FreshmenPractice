package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class LidarTestCommand extends Command {
    
    public DriveSubsystem drive;

    public Counter counter;
    public DigitalInput source;

    public Timer timer;

    public double lastDist;
    public double lastRobotX;
    public double currentDist;
    public double currentRobotX;
    public double lastTime;
    public double currentTime;
    public LinearFilter filter;

    public double lidarVelocity;
    public double robotVelocity;
    public double oponentVelocity;

    public double robotToMidline;
    public double oponentToMidline;

    public boolean doWin;

    public double robotX;
    public double oponentX;
    public Alliance alliance;
    public Optional<Alliance> ally;

    public LidarTestCommand(){
        drive = RobotContainer.drive;
        ally = DriverStation.getAlliance();

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

        lidarVelocity = Math.abs((currentDist - lastDist) / (currentTime - lastTime));

        SmartDashboard.putNumber("Object Dist", getDist());
        SmartDashboard.putNumber("Object lidarVelocity", lidarVelocity);

        lastRobotX = robotX;
        robotX = drive.getPose().getX();

        if(ally.get() == Alliance.Blue) oponentX = robotX + getDist();
        else oponentX = robotX - getDist();

        robotVelocity = Math.abs((robotX - lastRobotX) / (currentTime - lastTime));
        oponentVelocity = lidarVelocity - robotVelocity;

        robotToMidline = Math.abs(8.29 - robotX);
        oponentToMidline = Math.abs(8.29 - oponentX);

        if((oponentToMidline / oponentVelocity) > (robotToMidline / robotVelocity)) doWin = false;
        else doWin = true;

        SmartDashboard.putBoolean("Win Race To Note?", doWin);

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
