package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX IntakeMotor;
    private final DutyCycleOut percentOutCycle = new DutyCycleOut(0);

    public Intake() {
        IntakeMotor = new TalonFX(12);  
        IntakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    }

    public void percentOut(double speed) {
        IntakeMotor.setControl(percentOutCycle.withOutput(speed));
    }

    public void stop() {
        IntakeMotor.stopMotor();
    }
}