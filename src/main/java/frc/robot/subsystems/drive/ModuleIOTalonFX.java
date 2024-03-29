package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.math.NRUnits;

public class ModuleIOTalonFX implements ModuleIO {

    private SwerveModule module;

    private SwerveModulePosition position;
    private SwerveModuleState state;
    
    private TalonFX moveMotor;
    private TalonFX turnMotor;
    private CANcoder encoder;

     public ModuleIOTalonFX(SwerveModuleConstants constants, String canbusName) {
        module = new SwerveModule(constants, canbusName);

        position = module.getPosition(true);
        state = module.getCurrentState();

        moveMotor = module.getDriveMotor();
        turnMotor = module.getSteerMotor();
        encoder = module.getCANcoder();
    }    

    public void updateInputs(ModuleIOInputs inputs) {
        position = module.getPosition(true);
        state = module.getCurrentState();

        inputs.movePosition = position.distanceMeters;
        inputs.moveVelocity = Math.abs(module.getDriveMotor().getVelocity().getValueAsDouble() / Constants.Drive.kDriveGearRatio * Constants.TAU * Constants.Drive.WHEEL_RADIUS);
        inputs.moveVoltage = module.getDriveMotor().getMotorVoltage().getValueAsDouble();
        inputs.moveStatorCurrent = moveMotor.getStatorCurrent().getValue();
        inputs.moveSupplyCurrent = moveMotor.getSupplyCurrent().getValue();

        inputs.turnAbsolutePosition = NRUnits.logConstrainRad(encoder.getAbsolutePosition().getValue() * Constants.TAU);
        inputs.turnRotorPosition = NRUnits.logConstrainRad(position.angle.getRadians());
        inputs.turnVelocity = turnMotor.getVelocity().getValue() * Constants.TAU / Constants.Drive.kSteerGearRatio;
        inputs.turnVoltage = turnMotor.getMotorVoltage().getValueAsDouble();
        inputs.turnStatorCurrent = turnMotor.getStatorCurrent().getValue();
        inputs.turnSupplyCurrent = turnMotor.getSupplyCurrent().getValue();
    }

    public void set(SwerveModuleState state) {
        if(state.speedMetersPerSecond == 0) {
            module.apply(
                new SwerveModuleState(0, module.getTargetState().angle),
                DriveRequestType.Velocity,
                SteerRequestType.MotionMagicExpo
            );
        } else
        module.apply(state, DriveRequestType.Velocity, SteerRequestType.MotionMagicExpo);
    }

    public void setBoltage(double voltage){
        module.getDriveMotor().setVoltage(voltage);
    }


    public void disableCurrentLimit(){
        moveMotor.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(false)
        // .withStatorCurrentLimit(10)
        .withSupplyCurrentLimitEnable(false)
        .withSupplyCurrentThreshold(1000)
        );
    }

    public SwerveModulePosition getPosition() {
        return position;
    }

    public SwerveModuleState getState() {
        return state;
    }

}
