package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joint extends SubsystemBase {
    private final TalonSRX IntakeLeftMotor;
    private final TalonSRX IntakeRightMotor;

    public Joint() {
        IntakeLeftMotor = new TalonSRX(13);
        IntakeRightMotor = new TalonSRX(14);

        TalonSRXConfiguration configs = new TalonSRXConfiguration();

        IntakeLeftMotor.configAllSettings(configs);
        IntakeRightMotor.configAllSettings(configs);

        IntakeLeftMotor.setNeutralMode(NeutralMode.Brake);
        IntakeRightMotor.setNeutralMode(NeutralMode.Brake);

        IntakeLeftMotor.setInverted(false);
        IntakeRightMotor.setInverted(true);

        IntakeRightMotor.follow(IntakeLeftMotor);
    }

    public void percentOut(double speed) {
        IntakeLeftMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stop() {
        IntakeLeftMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
