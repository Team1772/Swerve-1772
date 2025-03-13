package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class JointSubsystem extends SubsystemBase {
    private final WPI_TalonSRX jointLeftMotor;
    private final WPI_VictorSPX jointRightMotor;
    private final DutyCycleEncoder absoluteEncoder;

    private final PIDController masterController;
    private final PIDController slaveController;

    private boolean absoluteEncoderConnected;
    private int absoluteEncoderFrequency;
    private double absoluteEncoderRaw;
    private double absoluteEncoderAdjusted;

    private double analogEncoderRaw;
    private double analogEncoderAdjusted;

    private GenericEntry absoluteEncoderMultiplierValue;
    private GenericEntry absoluteEncoderOffsetValue;

    private GenericEntry analogEncoderMultiplierValue;
    private GenericEntry analogEncoderOffsetValue;

    private GenericEntry setpointValue;

    private GenericEntry masterControllerProportionalValue;
    private GenericEntry masterControllerIntegralValue;
    private GenericEntry masterControllerDerivativeValue;

    private GenericEntry slaveControllerProportionalValue;
    private GenericEntry slaveControllerIntegralValue;
    private GenericEntry slaveControllerDerivativeValue;

    private GenericEntry dutyCycleOutValue;

    public JointSubsystem() {
        jointLeftMotor = new WPI_TalonSRX(Constants.JointConstants.LEFT_MOTOR_CAN_ID);
        jointRightMotor = new WPI_VictorSPX(Constants.JointConstants.RIGHT_MOTOR_CAN_ID);

        absoluteEncoder = new DutyCycleEncoder(Constants.JointConstants.RIGHT_ENCODER_DIO_PORT, 100, 0);
        absoluteEncoder.setAssumedFrequency(975.6);

        //jointLeftMotor.getSensorCollection().setQuadraturePosition((int) Math.round(absoluteEncoder.get()), 0);

        masterController = new PIDController(0, 0, 0);
        masterController.setTolerance(2);

        slaveController = new PIDController(0, 0, 0);
        slaveController.setTolerance(0.2);

        TalonSRXConfiguration masterConfigs = new TalonSRXConfiguration();
        VictorSPXConfiguration slaveConfigs = new VictorSPXConfiguration();

        jointLeftMotor.configAllSettings(masterConfigs);
        jointRightMotor.configAllSettings(slaveConfigs);

        jointLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        jointLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.None, 1, 0);

        jointLeftMotor.setNeutralMode(NeutralMode.Brake);
        jointRightMotor.setNeutralMode(NeutralMode.Brake);

        jointLeftMotor.setInverted(true);
        jointRightMotor.setInverted(false);

        //jointRightMotor.follow(jointLeftMotor);

        if(Constants.DEV_MODE) {
        absoluteEncoderMultiplierValue = Shuffleboard.getTab("Joint Subsystem").add("Absolute Encoder: Multipler", 1)
                     .withWidget(BuiltInWidgets.kTextView).getEntry();

        absoluteEncoderOffsetValue = Shuffleboard.getTab("Joint Subsystem").add("Absolute Encoder: Offset", 0)
                    .withWidget(BuiltInWidgets.kTextView).getEntry();

        analogEncoderMultiplierValue = Shuffleboard.getTab("Joint Subsystem").add("Analog Encoder: Multipler", 1)
                    .withWidget(BuiltInWidgets.kTextView).getEntry();

        analogEncoderOffsetValue = Shuffleboard.getTab("Joint Subsystem").add("Analog Encoder: Offset", 0)
                   .withWidget(BuiltInWidgets.kTextView).getEntry();

        setpointValue = Shuffleboard.getTab("Joint Subsystem").add("PID Controller: Setpoint", 0)
                   .withWidget(BuiltInWidgets.kTextView).getEntry();

        masterControllerProportionalValue = Shuffleboard.getTab("Joint Subsystem").add("Master: P", 0)
                   .withWidget(BuiltInWidgets.kTextView).getEntry();
        masterControllerIntegralValue = Shuffleboard.getTab("Joint Subsystem").add("Master: I", 0)
                   .withWidget(BuiltInWidgets.kTextView).getEntry();
        masterControllerDerivativeValue = Shuffleboard.getTab("Joint Subsystem").add("Master: D", 0)
                   .withWidget(BuiltInWidgets.kTextView).getEntry();

        slaveControllerProportionalValue = Shuffleboard.getTab("Joint Subsystem").add("Slave: P", 0)
                   .withWidget(BuiltInWidgets.kTextView).getEntry();
        slaveControllerIntegralValue = Shuffleboard.getTab("Joint Subsystem").add("Slave: I", 0)
                   .withWidget(BuiltInWidgets.kTextView).getEntry();
        slaveControllerDerivativeValue = Shuffleboard.getTab("Joint Subsystem").add("Slave: D", 0)
                   .withWidget(BuiltInWidgets.kTextView).getEntry();
        dutyCycleOutValue = Shuffleboard.getTab("Joint Subsystem").add("DutyCycleOut", 0)
                   .withWidget(BuiltInWidgets.kTextView).getEntry();

        Shuffleboard.getTab("Joint Subsystem").add("Closed Loop Command", testClosedLoopCommand());
        Shuffleboard.getTab("Joint Subsystem").add("Update Controllers Parameters Command", testUpdateClosedLoopControllersCommand());
        }
    }

    public void percentOut(double speed) {
        jointLeftMotor.set(ControlMode.PercentOutput, speed);
        jointRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        jointLeftMotor.set(ControlMode.PercentOutput, 0);
        jointRightMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean atSetpoint() {
        return masterController.atSetpoint() && slaveController.atSetpoint();
    }

    public void masterClosedLoop(double setpoint) {
        jointLeftMotor.set(TalonSRXControlMode.Position, masterController.calculate(analogEncoderAdjusted, setpoint));
    }

    public void slaveClosedLoop() {
        jointRightMotor.set(VictorSPXControlMode.PercentOutput, slaveController.calculate(absoluteEncoderAdjusted, analogEncoderAdjusted));
    }

    public void positionClosedLoop(double setpoint) {
        masterClosedLoop(setpoint);
        slaveClosedLoop();
    }

    public Command percentOutCommand(DoubleSupplier speed) {
        return Commands.startEnd(() -> this.percentOut(speed.getAsDouble()), this::stop, this);
    }

    public Command closedLoopCommand(DoubleSupplier speed) {
        return Commands.startEnd(() -> this.positionClosedLoop(speed.getAsDouble()), this::stop, this);
    }

    public Command testClosedLoopCommand() {
        return Commands.startEnd(() -> this.positionClosedLoop(setpointValue.getDouble(0)), this::stop, this).until(this::atSetpoint);
    }

    public void updateClosedLoopControllersParameters() {
        masterController.setPID(masterControllerProportionalValue.getDouble(0), masterControllerIntegralValue.getDouble(0), masterControllerDerivativeValue.getDouble(0));
        slaveController.setPID(slaveControllerProportionalValue.getDouble(0), slaveControllerIntegralValue.getDouble(0), slaveControllerDerivativeValue.getDouble(0));
    }

    public Command testOpenLoopCommand() {
        return this.runEnd(() -> this.percentOut(dutyCycleOutValue.getDouble(0)), this::stop);
    }

    public Command testUpdateClosedLoopControllersCommand() {
        return new InstantCommand(this::updateClosedLoopControllersParameters, this);
    }

    public void debug() {
        absoluteEncoderConnected = absoluteEncoder.isConnected();
        absoluteEncoderFrequency = absoluteEncoder.getFrequency();
        absoluteEncoderAdjusted = ((absoluteEncoder.get()) + absoluteEncoderOffsetValue.getDouble(0))*(absoluteEncoderMultiplierValue.getDouble(1));
        absoluteEncoderRaw = (absoluteEncoder.get());
        analogEncoderAdjusted = (jointLeftMotor.getSelectedSensorPosition(0) + analogEncoderOffsetValue.getDouble(0))*(analogEncoderMultiplierValue.getDouble(1));
        analogEncoderRaw = jointLeftMotor.getSelectedSensorPosition(0);

        SmartDashboard.putBoolean("Joint Subsystem/Absolute Encoder/Connected", absoluteEncoderConnected);
        SmartDashboard.putNumber("Joint Subsystem/Absolute Encoder/Frequency", absoluteEncoderFrequency);
        SmartDashboard.putNumber("Joint Subsystem/Absolute Encoder/Adjusted", absoluteEncoderAdjusted);
        SmartDashboard.putNumber("Joint Subsystem/Absolute Encoder/Raw", absoluteEncoderRaw);
        SmartDashboard.putNumber("Joint Subsystem/Analog Encoder/Adjusted", analogEncoderAdjusted);
        SmartDashboard.putNumber("Joint Subsystem/Analog Encoder/Raw", analogEncoderRaw);
        SmartDashboard.putBoolean("Joint Subsystem/Encoders/Is Matching", analogEncoderAdjusted == absoluteEncoderAdjusted);
        SmartDashboard.putNumber("Joint Subsystem/Encoders/Get Difference", analogEncoderAdjusted - absoluteEncoderAdjusted);
    }

    @Override
    public void periodic() {
        if(Constants.DEV_MODE) {
            this.debug();
        }
    }
}
