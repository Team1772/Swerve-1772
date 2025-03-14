package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PuncherSubsystem extends SubsystemBase {
    private final TalonFX puncherLeftMotor;
    private final TalonFX puncherRightMotor;
    private final Follower follower;

    private final TalonSRX puncherReleaseMotor;
    private final Encoder puncherReleaseFeedback;

    private final DutyCycleOut percentOutCycle = new DutyCycleOut(0);
    private final PositionDutyCycle positionCycle = new PositionDutyCycle(0);
    private boolean isPuncherReady = false;

    private GenericEntry setpointValue;
    private GenericEntry releaseTimeInValue;
    private GenericEntry releaseTimeOutValue;
    private GenericEntry releaseDutyCycleValue;
    private GenericEntry tightenDutyCycleValue;
    private GenericEntry encoderSetpointValue;

    public PuncherSubsystem() {
        puncherLeftMotor = new TalonFX(Constants.PuncherConstants.LEFT_MOTOR_CAN_ID);
        puncherRightMotor = new TalonFX(Constants.PuncherConstants.RIGHT_MOTOR_CAN_ID);
        follower = new Follower(Constants.PuncherConstants.LEFT_MOTOR_CAN_ID, false);
        puncherReleaseMotor = new TalonSRX(Constants.PuncherConstants.RELEASE_MOTOR_CAN_ID);
        puncherReleaseFeedback = new Encoder(Constants.PuncherConstants.ENCODER_DIO_PORT_B, Constants.PuncherConstants.ENCODER_DIO_PORT_A);

        puncherReleaseMotor.setNeutralMode(NeutralMode.Brake);

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

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = false;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 35;
        currentLimitsConfigs.SupplyCurrentLowerLimit = 30;
        currentLimitsConfigs.SupplyCurrentLowerTime = 1;
        masterConfig.apply(currentLimitsConfigs);
        slaveConfig.apply(currentLimitsConfigs);

        puncherReleaseFeedback.reset();

        if(Constants.DEV_MODE) {
        setpointValue = Shuffleboard.getTab("Puncher Subsystem").add("Setpoint", 45)
                               .withWidget(BuiltInWidgets.kTextView).getEntry();

        releaseTimeInValue = Shuffleboard.getTab("Puncher Subsystem").add("Release: Timer", 0.3)
                               .withWidget(BuiltInWidgets.kTextView).getEntry();

        releaseTimeOutValue = Shuffleboard.getTab("Puncher Subsystem").add("Tighten: Timer", 0.3)
                               .withWidget(BuiltInWidgets.kTextView).getEntry();

        releaseDutyCycleValue = Shuffleboard.getTab("Puncher Subsystem").add("Release: DutyCycleOut", 0.3)
                               .withWidget(BuiltInWidgets.kTextView).getEntry();

        tightenDutyCycleValue = Shuffleboard.getTab("Puncher Subsystem").add("Tighten: DutyCycleOut", 0.31)
                               .withWidget(BuiltInWidgets.kTextView).getEntry();

        encoderSetpointValue = Shuffleboard.getTab("Puncher Subsystem").add("Release: Setpoint", 500)
                               .withWidget(BuiltInWidgets.kTextView).getEntry();
        }
    }

    public boolean isPuncherReady() {
        return isPuncherReady;
    }

    public void setIsPuncherReady(boolean isPuncherReady) {
        this.isPuncherReady = isPuncherReady;
    }

    public double getPosition() {
        return puncherLeftMotor.getPosition().getValueAsDouble();
    }

    public void resetPosition(){
        puncherLeftMotor.setPosition(0);
    }

    public void goToPosition(double position) {
        puncherLeftMotor.setControl(positionCycle.withPosition(position));
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

    public void setRelease(double speed) {
        puncherReleaseMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stopRelease() {
        puncherReleaseMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void resetEncoders() {
        puncherLeftMotor.setPosition(0);
        puncherRightMotor.setPosition(0);
        puncherReleaseFeedback.reset();
    }

    public Command buildUpCommand(DoubleSupplier position) {
        return Commands.startEnd(() -> this.goToPosition(position.getAsDouble()), () -> {this.goToPosition(0);
        this.setIsPuncherReady(true);}, this); //ADICIONAR UNTIL PARA POSICAO ESPECIFICA E TESTAR
    }

    public Command releaseCommand(DoubleSupplier speed1, DoubleSupplier speed2) {
        return Commands.startEnd(() -> this.setRelease(-speed1.getAsDouble()), this::stopRelease, this)
                        .until(() -> puncherReleaseFeedback.get() == 500).andThen
                            (Commands.startEnd(() -> this.setRelease(speed2.getAsDouble()), () -> {this.stopRelease();
                                this.setIsPuncherReady(false);}, this)
                            .until(() -> puncherReleaseFeedback.get() == 0));
    }

    public Command testPrintCommand() {
        return new InstantCommand(() -> System.out.println(setpointValue.getDouble(40)), this).ignoringDisable(true);
    }

    public Command testBuildUpCommand() {
        return Commands.startEnd(() -> this.goToPosition(setpointValue.getDouble(45)), () -> {this.goToPosition(0);
            this.setIsPuncherReady(true);}, this).until(() -> puncherLeftMotor.getPosition().getValueAsDouble() > 20);
    }

    public Command testReleaseCommand() {
        return Commands.startEnd(() -> this.setRelease(-0.3), this::stopRelease, this)
            .until(() -> puncherReleaseFeedback.get() > 440).andThen
                (Commands.startEnd(() -> this.setRelease(0.3), () -> {this.stopRelease();
                    this.setIsPuncherReady(false);}, this)
                .until(() -> puncherReleaseFeedback.get() < 10));
    }

    public void resetState() {
        this.setIsPuncherReady(false);
    }

    public void debug() {
        SmartDashboard.putNumber("Puncher Subsystem/Release/Feedback", puncherReleaseFeedback.get());
        SmartDashboard.putBoolean("Puncher Subsystem/State", isPuncherReady());
        SmartDashboard.putNumber("Puncher Subsystem/Motor", puncherLeftMotor.getPosition().getValueAsDouble());
    }

   @Override
    public void periodic() {
        if(Constants.DEV_MODE) {
            this.debug();
        }
    }
}