package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmSubsystem {
    private TalonFX pivot, pivot2;
    private TalonFXConfiguration config;

    public ArmSubsystem(){
        pivot = new TalonFX(13, "drivet");
        pivot2 = new TalonFX(15, "drivet");

        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivot.getConfigurator().apply(config);
        pivot2.getConfigurator().apply(config);
    }

}
