package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;
import frc.robot.lib.math.SwerveMath;

public class DriveSubsystem extends SubsystemBase{


    private Module[] modules;

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private boolean fieldCentric;

    public DriveSubsystem() {
        gyroIO = new GyroIOPigeon2();

        fieldCentric = true;

        modules = new Module[] {
            new Module(0, Constants.Drive.CANBUS),
            new Module(1, Constants.Drive.CANBUS),
            new Module(2, Constants.Drive.CANBUS),
            new Module(3, Constants.Drive.CANBUS)
        };

    }

    /*
     * xSpeed: speed of the robot (Forward) (m/s)
     * ySpeed: speed of the robot (Side) (m/s)
     * omegaSpeed: speed of the rotation (Counterclockwise positive) (rad/s)
     */
    public void set(double xSpeed, double ySpeed, double omegaSpeed){
        set(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
    }

    private PIDController angleController = new PIDController(10, 0, 0);
    private double lastJoystickAngle = 0;

    /*
     * chassisSpeeds: Object that contains values for the Chassis Speeds
     */
    public void set(ChassisSpeeds chassisSpeeds) {
        double x = chassisSpeeds.vxMetersPerSecond;
        double y = chassisSpeeds.vyMetersPerSecond;

        double omega = chassisSpeeds.omegaRadiansPerSecond;

        // if(gyroInputs.zVelocity >= 0.10 || omega != 0) lastJoystickAngle = getYaw().getRadians();
        // else omega = Math.abs(lastJoystickAngle - getYaw().getRadians()) < Constants.TAU/10 &&
        //     Math.sqrt(x*x+y*y) > 0.1 ?
        //     angleController.calculate(getYaw().getRadians(), lastJoystickAngle) :
        //     0;

        if(fieldCentric) {
            double angleDiff = Math.atan2(y, x) - getGyroAngle().getRadians(); //difference between input angle and gyro angle gives desired field relative angle
            double r = Math.sqrt(x*x + y*y); //magnitude of translation vector
            x = r * Math.cos(angleDiff);
            y = r * Math.sin(angleDiff);
        }
        
        // //Repeated equations
        double a = omega * Constants.Drive.WIDTH/2;
        double b = omega * Constants.Drive.LENGTH/2;

        //The addition of the movement and rotational vector
        Translation2d[] t = new Translation2d[] {
            new Translation2d(x+b, y+a),
            new Translation2d(x-b, y+a),
            new Translation2d(x-b, y-a),
            new Translation2d(x+b, y-a),
        };

        SwerveModuleState[] setStates = new SwerveModuleState[t.length];
        for(int i = 0; i < t.length; i++) {
            setStates[i] = new SwerveModuleState(t[i].getNorm(), t[i].getAngle());
        }

        setStates = SwerveMath.normalize(setStates);

        setStates(setStates);
    }

    public void setState(SwerveModuleState state, int modIndex) {
        modules[modIndex].set(state);
    }


    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public void setStates(SwerveModuleState[] states) {
        for(int i = 0; i < modules.length; i++) {
            Logger.recordOutput("Velocity/Mod"+i+"Velocity", states[i].speedMetersPerSecond);
            Logger.recordOutput("Velocity/Mod"+i+"Angle", NRUnits.logConstrainRad(states[i].angle.getRadians()+Constants.TAU));
            modules[i].set(states[i]);
        }
    }

    public void setVoltageStates(double voltage){
        for(int i = 2; i < 4; i++){ //Setting only the back 2 motors
            modules[i].setBoltage(voltage);
        }
    }


    // True if the robot is field-centric, false for robot-centric
    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public boolean isFieldCentric() {
        return this.fieldCentric;
    }

    //Sets the angle of the robot in radians
    public void setGyro(double angle) {
        gyroIO.setYaw(angle * 180 / Math.PI);
    }

    //Zeroes the yaw (Rotational direction)
    public void zeroYaw() {
        gyroIO.setYaw(0);
    }

    //Returns the Robot's yaw orientation in radians (Contstrained)
    public Rotation2d getGyroAngle() {
        return Rotation2d.fromRadians(NRUnits.constrainRad(getYaw().getRadians()));
    }

    //Returns the gyro's yaw orientation in radians (Rotation Horizontal)
    public Rotation2d getYaw(){
        return Rotation2d.fromRadians(gyroInputs.yaw);
    }

    //Returns the gyro's pitch (Front/backflips)
    public Rotation2d getPitch(){
        return Rotation2d.fromRadians(gyroInputs.pitch);
    }

    //Returns the gyro's roll (Rolling over)
    public Rotation2d getRoll(){
        return Rotation2d.fromRadians(gyroInputs.roll);
    }



    @Override
    public void periodic(){
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for(Module module : modules) {
            module.periodic();
        }

        Logger.recordOutput("FPGATimestamp", Timer.getFPGATimestamp());


        Logger.recordOutput("GetGyroAngle", getGyroAngle().getRadians());
    }
}
