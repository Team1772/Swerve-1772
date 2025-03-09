package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX intakeMotor;

    private GenericEntry intakeDutyCycleValue;
    private GenericEntry outtakeDutyCycleValue;
    
    public IntakeSubsystem() {
        intakeMotor = new TalonSRX(12);

        TalonSRXConfiguration configs = new TalonSRXConfiguration();

        intakeMotor.configAllSettings(configs);
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.setInverted(false);

        if(Constants.DEV_MODE) {
        intakeDutyCycleValue = Shuffleboard.getTab("Intake Subsystem").add("Intake: DutyCycleOut", 0.3)
                               .withWidget(BuiltInWidgets.kTextView).getEntry();

        outtakeDutyCycleValue = Shuffleboard.getTab("Intake Subsystem").add("Outtake: DutyCycleOut", 0.6)
                               .withWidget(BuiltInWidgets.kTextView).getEntry();
        }
    }

    public void percentOut(double speed) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void stop() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public Command percentOutCommand(DoubleSupplier speed) {
        return Commands.startEnd(() -> this.percentOut(speed.getAsDouble()), this::stop, this);
    }

    public Command testIntakeCommand() {
        return Commands.startEnd(() -> this.percentOut(-intakeDutyCycleValue.getDouble(0.3)), this::stop, this);
    }

    public Command testOuttakeCommand() {
        return Commands.startEnd(() -> this.percentOut(outtakeDutyCycleValue.getDouble(0.6)), this::stop, this);
    }

    public void debug() {
        
    }

   @Override
    public void periodic() {
        if(Constants.DEV_MODE) {
            this.debug();
        }
    }
}