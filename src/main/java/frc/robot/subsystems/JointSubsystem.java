package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JointSubsystem extends SubsystemBase {
    private final TalonSRX JointLeftMotor;
    private final TalonSRX JointRightMotor;
    private final DutyCycleEncoder absoluteEncoder;

    private boolean absoluteEncoderConnected;
    private int absoluteEncoderFrequency;
    private double absoluteEncoderOutput;
    private double relativeEncoderPosition;

    public JointSubsystem() {
        JointLeftMotor = new TalonSRX(13);
        JointRightMotor = new TalonSRX(14);

        absoluteEncoder = new DutyCycleEncoder(3, 1, 0);
        absoluteEncoder.setAssumedFrequency(975.6);

        JointRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        JointRightMotor.getSensorCollection().setQuadraturePosition(0, 10);

        TalonSRXConfiguration configs = new TalonSRXConfiguration();

        JointLeftMotor.configAllSettings(configs);
        JointRightMotor.configAllSettings(configs);

        JointLeftMotor.setNeutralMode(NeutralMode.Brake);
        JointRightMotor.setNeutralMode(NeutralMode.Brake);

        JointLeftMotor.setInverted(true);
        JointRightMotor.setInverted(false);

        JointRightMotor.follow(JointLeftMotor);
    }

    public void percentOut(double speed) {
        JointLeftMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stop() {
        JointLeftMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        absoluteEncoderConnected = absoluteEncoder.isConnected();
        absoluteEncoderFrequency = absoluteEncoder.getFrequency();
        absoluteEncoderOutput = absoluteEncoder.get();

        relativeEncoderPosition = JointRightMotor.getSelectedSensorPosition(0);

        SmartDashboard.putBoolean("Joint Subsystem/Absolute Encoder/Connected", absoluteEncoderConnected);
        SmartDashboard.putNumber("Joint Subsystem/Absolute Encoder/Frequency", absoluteEncoderFrequency);
        SmartDashboard.putNumber("Joint Subsystem/Absolute Encoder/Output", absoluteEncoderOutput);

        SmartDashboard.putNumber("Joint Subsystem/Relative Encoder/Position", relativeEncoderPosition);
    }
}
