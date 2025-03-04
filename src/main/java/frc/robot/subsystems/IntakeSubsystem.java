package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX intakeMotor;
    
    public IntakeSubsystem() {
        intakeMotor = new TalonSRX(12);

        TalonSRXConfiguration configs = new TalonSRXConfiguration();

        intakeMotor.configAllSettings(configs);
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.setInverted(false);
    }

    public void percentOut(double speed) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stop() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}