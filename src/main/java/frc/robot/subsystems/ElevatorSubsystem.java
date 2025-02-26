package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class ElevatorSubsystem extends SubsystemBase {
    private final VictorSPX motor;
    private final DigitalInput minLimitSwitch;
    private final DigitalInput maxLimitSwitch;
   
    private static final double SPEED_UP = 0.0;
    private static final double SPEED_DOWN = 0.0;
   
    public ElevatorSubsystem() {
        motor = new VictorSPX(15);
        minLimitSwitch = new DigitalInput(0);
        maxLimitSwitch = new DigitalInput(1);
    }
   
    public void moveUp() {
        if (!this.isAtMax()) {
            motor.set(ControlMode.PercentOutput, SPEED_UP);
        } else {
            stop();
        }
    }
   
    public void moveDown() {
        if (!this.isAtMin()) {
            motor.set(ControlMode.PercentOutput, SPEED_DOWN);
        } else {
            stop();
        }
    }
   
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }
   
    public boolean isAtMin() {
        return !minLimitSwitch.get();
    }
   
    public boolean isAtMax() {
        return !maxLimitSwitch.get();
    }
}