package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PuncherSubsystem extends SubsystemBase {
    private final TalonFX puncherLeftMotor;
    private final TalonFX puncherRightMotor;
    private final Follower follower;

    private final TalonSRX baixo;

    private final DutyCycleOut percentOutCycle = new DutyCycleOut(0);
    private final PositionDutyCycle positionCycle = new PositionDutyCycle(0);

    public PuncherSubsystem() {
        puncherLeftMotor = new TalonFX(10);
        puncherRightMotor = new TalonFX(11);
        follower = new Follower(10, false);
        baixo = new TalonSRX(16);

        puncherLeftMotor.setPosition(0);
        puncherRightMotor.setPosition(0);

        puncherLeftMotor.getConfigurator().apply(new TalonFXConfiguration());
        puncherRightMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfigurator masterConfig = puncherLeftMotor.getConfigurator();
        TalonFXConfigurator slaveConfig = puncherRightMotor.getConfigurator();

        Slot0Configs positionPIDConfigs = new Slot0Configs();
        positionPIDConfigs.kP = 0.2;
        positionPIDConfigs.kI = 0.005;
        positionPIDConfigs.kD = 0;
        positionPIDConfigs.kA = 0;
        masterConfig.apply(positionPIDConfigs);
        slaveConfig.apply(positionPIDConfigs);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.PeakForwardDutyCycle = 1;
        outputConfigs.PeakReverseDutyCycle = -1;
        outputConfigs.withNeutralMode(NeutralModeValue.Coast);
        //outputConfigs.withInverted(InvertedValue.Clockwise_Positive);
        masterConfig.apply(outputConfigs);
        slaveConfig.apply(outputConfigs);

        SoftwareLimitSwitchConfigs limitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        limitSwitchConfigs.ForwardSoftLimitEnable = false;
        limitSwitchConfigs.ReverseSoftLimitEnable = false;
        limitSwitchConfigs.ForwardSoftLimitThreshold = 0;
        limitSwitchConfigs.ReverseSoftLimitThreshold = 0;
        masterConfig.apply(limitSwitchConfigs);
        slaveConfig.apply(limitSwitchConfigs);
    }

    public double getPosition() {
        return puncherLeftMotor.getPosition().getValueAsDouble();
    }

    public void goToPosition(double position) {
        double clampedPosition = MathUtil.clamp(position, 0, 100);
        puncherLeftMotor.setControl(positionCycle.withPosition(clampedPosition));
        puncherRightMotor.setControl(follower);
    }

    public void percentOut(double speed) {
        puncherLeftMotor.setControl(percentOutCycle.withOutput(speed));
        puncherRightMotor.setControl(follower);
    }

    public void stop() {
        puncherLeftMotor.stopMotor();
        puncherRightMotor.stopMotor();
    }

    public void setBaixo(double speed) {
        baixo.set(TalonSRXControlMode.PercentOutput,speed);
    }

    public void resetEncoders() {
        puncherLeftMotor.setPosition(0);
        puncherRightMotor.setPosition(0);
    }
}