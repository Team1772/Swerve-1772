package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JointSubsystem extends SubsystemBase {
    private final TalonSRX jointLeftMotor;
    private final VictorSPX jointRightMotor;
    private final DutyCycleEncoder absoluteEncoder;
    //private final PIDController pidController;

    private boolean absoluteEncoderConnected;
    private int absoluteEncoderFrequency;
    private double absoluteEncoderOutput;
    private double absoluteEncoderPosition;

    private double relativeEncoderPosition;

    public JointSubsystem() {
        jointLeftMotor = new TalonSRX(13);
        jointRightMotor = new VictorSPX(14);

        absoluteEncoder = new DutyCycleEncoder(3, 1, 0);
        absoluteEncoder.setAssumedFrequency(975.6);

        jointLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        //jointRightMotor.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);

        jointLeftMotor.getSensorCollection().setQuadraturePosition((int) Math.round(absoluteEncoder.get()), 0);

        //pidController = new PIDController(relativeEncoderPosition, absoluteEncoderPosition, absoluteEncoderOutput, absoluteEncoderFrequency);

        TalonSRXConfiguration masterConfigs = new TalonSRXConfiguration();
        VictorSPXConfiguration slaveConfigs = new VictorSPXConfiguration();

        jointLeftMotor.configAllSettings(masterConfigs);
        jointRightMotor.configAllSettings(slaveConfigs);

        jointLeftMotor.setNeutralMode(NeutralMode.Brake);
        jointRightMotor.setNeutralMode(NeutralMode.Brake);

        jointLeftMotor.setInverted(true);
        jointRightMotor.setInverted(false);

        jointRightMotor.follow(jointLeftMotor);
    }

    public void percentOut(double speed) {
        jointLeftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        jointLeftMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        absoluteEncoderConnected = absoluteEncoder.isConnected();
        absoluteEncoderFrequency = absoluteEncoder.getFrequency();
        absoluteEncoderOutput = (((absoluteEncoder.get()*100)-74)*36.7) - 308;
        //absoluteEncoderOutput = absoluteEncoder.get();
        absoluteEncoderPosition = jointRightMotor.getSelectedSensorPosition(0);

        relativeEncoderPosition = jointLeftMotor.getSelectedSensorPosition(0)*(-1);

        SmartDashboard.putBoolean("Joint Subsystem/Absolute Encoder/Connected", absoluteEncoderConnected);
        SmartDashboard.putNumber("Joint Subsystem/Absolute Encoder/Frequency", absoluteEncoderFrequency);
        SmartDashboard.putNumber("Joint Subsystem/Absolute Encoder/Output", absoluteEncoderOutput);
        SmartDashboard.putNumber("Joint Subsystem/Absolute Encoder/Position", absoluteEncoderPosition);

        SmartDashboard.putNumber("Joint Subsystem/Relative Encoder/Position", relativeEncoderPosition);
    }
}
