package frc.robot.subsystems.swervetank;

import java.nio.file.ClosedFileSystemException;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Drivetrain extends SubsystemBase {
    private final DifferentialDrive differentialDrive;

    private final SparkMax leftFrontDrive;
    private final SparkMax leftBackDrive;
    private final SparkMax rightFrontDrive;
    private final SparkMax rightBackDrive;

    private final SparkMax leftFrontAngle;
    private final SparkMax leftBackAngle;
    private final SparkMax rightFrontAngle;
    private final SparkMax rightBackAngle;

    private final SparkClosedLoopController leftFrontAngleClosedLoopController;
    private final SparkClosedLoopController leftBackAngleClosedLoopController;
    private final SparkClosedLoopController rightFrontAngleClosedLoopController;
    private final SparkClosedLoopController rightBackAngleClosedLoopController;

    private final SparkMaxConfig globalDriveConfig;
    private final SparkMaxConfig globalAngleConfig;

    private final SparkMaxConfig leftFollowerConfig;
    private final SparkMaxConfig rightFollowerConfig;


    public Drivetrain() {
        leftFrontDrive = new SparkMax(1, MotorType.kBrushless);
        leftBackDrive = new SparkMax(7, MotorType.kBrushless);
        rightFrontDrive = new SparkMax(3, MotorType.kBrushless);
        rightBackDrive = new SparkMax(5, MotorType.kBrushless);

        leftFrontAngle = new SparkMax(4, MotorType.kBrushless);
        leftBackAngle = new SparkMax(9, MotorType.kBrushless);
        rightFrontAngle = new SparkMax(2, MotorType.kBrushless);
        rightBackAngle = new SparkMax(6, MotorType.kBrushless);

        leftFrontAngleClosedLoopController = leftFrontAngle.getClosedLoopController();
        leftBackAngleClosedLoopController = leftBackAngle.getClosedLoopController();
        rightFrontAngleClosedLoopController = rightFrontAngle.getClosedLoopController();
        rightBackAngleClosedLoopController = rightBackAngle.getClosedLoopController();

        globalDriveConfig = new SparkMaxConfig();
        globalAngleConfig = new SparkMaxConfig();

        leftFollowerConfig = new SparkMaxConfig();
        rightFollowerConfig = new SparkMaxConfig();

        globalDriveConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        globalAngleConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        globalAngleConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        globalAngleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1);

        leftFollowerConfig
            .apply(globalDriveConfig)
            .follow(leftFrontDrive);

        rightFollowerConfig
            .apply(globalDriveConfig)
            .follow(rightFrontDrive);

        leftFrontDrive.configure(globalDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftBackDrive.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFrontDrive.configure(globalDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBackDrive.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftFrontAngle.configure(globalAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftBackAngle.configure(globalAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightFrontAngle.configure(globalAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBackAngle.configure(globalAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        differentialDrive = new DifferentialDrive(leftFrontDrive::set, rightFrontDrive::set);
    }

    public Command drive(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
        return this.run(() -> differentialDrive.arcadeDrive(-leftSpeed.getAsDouble(), -rightSpeed.getAsDouble()));
    }

    public void setMotorBrake(boolean isBrake) {

    }

    public void zeroGyro() {

    }

    public Command autonomousCommand() {
        return this.runEnd(() -> differentialDrive.tankDrive(-0.8, -0.8), differentialDrive::stopMotor).withTimeout(5);
    }

    public void lockMotors() {
        leftFrontAngleClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        leftBackAngleClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        rightFrontAngleClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        rightBackAngleClosedLoopController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Tank/Position", leftFrontAngle.getEncoder().getPosition());
        lockMotors();
    }
}
