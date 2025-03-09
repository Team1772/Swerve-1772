package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonSRX motor;
    private final DigitalInput minLimitSwitch;

    private GenericEntry ascendDutyCycleOutValue;
    private GenericEntry descendDutyCycleOutValue;
   
    public ElevatorSubsystem() {
        motor = new TalonSRX(15);
        minLimitSwitch = new DigitalInput(0);

        if(Constants.DEV_MODE) {
        ascendDutyCycleOutValue = Shuffleboard.getTab("Elevator Subsystem").add("Ascend: DutyCycleOut", 0)
                                  .withWidget(BuiltInWidgets.kTextView).getEntry();

        descendDutyCycleOutValue = Shuffleboard.getTab("Elevator Subsystem").add("Descend: DutyCycleOut", 0)
                                  .withWidget(BuiltInWidgets.kTextView).getEntry();
        }
    }
   
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }
   
    public boolean isAtMin() {
        return minLimitSwitch.get();
    }

    public void percentOut(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    public Command percentOutCommand(DoubleSupplier speed) {
        return Commands.startEnd(() -> this.percentOut(speed.getAsDouble()), this::stop, this);
    }

    public Command descendCommand() {
        return Commands.startEnd(() -> this.percentOut(-1), this::stop, this).until(this::isAtMin);
    }

    public Command testDescendCommand() {
        return Commands.startEnd(() -> this.percentOut(descendDutyCycleOutValue.getDouble(0)*(-1)), this::stop, this).until(this::isAtMin);
    }

    public Command testAscendCommand() {
        return Commands.startEnd(() -> this.percentOut(ascendDutyCycleOutValue.getDouble(0)), this::stop, this);
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