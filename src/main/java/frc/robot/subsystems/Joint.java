package frc.robot.subsystems;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joint extends SubsystemBase {
    private final TalonSRX intakeLeftMotor;
    private final TalonSRX intakeRightMotor;
    private final DutyCycleEncoder absoluteEncoder;

    public Joint() {
        intakeLeftMotor = new TalonSRX(13);
        intakeRightMotor = new TalonSRX(14);

        absoluteEncoder = new DutyCycleEncoder(new DigitalInput(2));

        intakeLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 0);
        intakeRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        TalonSRXConfiguration configs = new TalonSRXConfiguration();

        intakeLeftMotor.configAllSettings(configs);
        intakeRightMotor.configAllSettings(configs);

        intakeLeftMotor.setNeutralMode(NeutralMode.Brake);
        intakeRightMotor.setNeutralMode(NeutralMode.Brake);

        intakeLeftMotor.setInverted(false);
        intakeRightMotor.setInverted(true);

        intakeRightMotor.follow(intakeLeftMotor);
    }

    public void percentOut(double speed) {
        intakeLeftMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stop() {
        intakeLeftMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
