package frc.robot.buttonBindings;

import org.dyn4j.dynamics.joint.Joint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.PuncherSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class DriverButtonBindings {
    private final CommandXboxController driverXbox;

    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final JointSubsystem jointSubsystem;
    private final PuncherSubsystem puncherSubsystem;

    private final SwerveInputStream driveDirectAngle;
    private final SwerveInputStream driveRobotOriented;
    private final SwerveInputStream driveAngularVelocityKeyboard;
    private final SwerveInputStream driveDirectAngleKeyboard;
    private final SwerveInputStream driveAngularVelocity;

    private final Command driveFieldOrientedDirectAngle;
    private final Command driveFieldOrientedAnglularVelocity;
    private final Command driveRobotOrientedAngularVelocity;
    private final Command driveSetpointGen;
    private final Command driveFieldOrientedDirectAngleKeyboard;
    private final Command driveFieldOrientedAnglularVelocityKeyboard;
    private final Command driveSetpointGenKeyboard;


    public DriverButtonBindings(CommandXboxController driverXbox, SwerveSubsystem drivebase, 
                                IntakeSubsystem intake, JointSubsystem joint, PuncherSubsystem puncher) {
        this.driverXbox = driverXbox;

        this.swerveSubsystem = drivebase;
        this.intakeSubsystem = intake;
        this.jointSubsystem = joint;
        this.puncherSubsystem = puncher;

        driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> {
                double leftTrigger = driverXbox.getLeftTriggerAxis();  
                double rightTrigger = driverXbox.getRightTriggerAxis(); 
                double rotation = rightTrigger - leftTrigger;
                return rotation;
            })
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

        driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis
            (driverXbox::getRightX, driverXbox::getRightY).headingWhile(true);

        driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

        driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX())
            .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

        driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() -> Math.sin(
            driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
            () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
            .headingWhile(true);

        driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
        driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);
    }

    public void configureBindings() {
        drivebaseDefaultButtonBindings();
        drivebaseSimulationButtonBindings();
        drivebaseTestButtonBindings();
        intakeButtonBindings();
        jointButtonBindings();
        puncherButtonBindings();
    }

    public void drivebaseDefaultButtonBindings() {
        if (!RobotBase.isSimulation()) {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        if (!DriverStation.isTest()) {
        driverXbox.a().onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
        driverXbox.x().onTrue(Commands.runOnce(swerveSubsystem::addFakeVisionReading));
        driverXbox.b().whileTrue(swerveSubsystem.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
        driverXbox.start().whileTrue(Commands.none());
        driverXbox.back().whileTrue(Commands.none());
        driverXbox.leftBumper().whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
        driverXbox.rightBumper().onTrue(Commands.none());
        }
    }

    public void drivebaseSimulationButtonBindings() {
        if (RobotBase.isSimulation()) {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } 

        if (Robot.isSimulation()) {
        driverXbox.start().onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
        driverXbox.button(1).whileTrue(swerveSubsystem.sysIdDriveMotorCommand());
        }
    }

    public void drivebaseTestButtonBindings() {
        if (DriverStation.isTest()) {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        driverXbox.x().whileTrue(Commands.runOnce(swerveSubsystem::lock, swerveSubsystem).repeatedly());
        driverXbox.y().whileTrue(swerveSubsystem.driveToDistanceCommand(1.0, 0.2));
        driverXbox.start().onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
        driverXbox.back().whileTrue(swerveSubsystem.centerModulesCommand());
        driverXbox.leftBumper().onTrue(Commands.none());
        driverXbox.rightBumper().onTrue(Commands.none());
        }
    }

    public void intakeButtonBindings() {
        driverXbox.povRight().whileTrue(Commands.startEnd(() -> intakeSubsystem.percentOut(0), () -> intakeSubsystem.stop(), intakeSubsystem));
        driverXbox.povLeft().whileTrue(Commands.startEnd(() -> intakeSubsystem.percentOut(0), () -> intakeSubsystem.stop(), intakeSubsystem)); 
    }

    public void jointButtonBindings() {
        driverXbox.povUp().whileTrue(Commands.startEnd(() -> jointSubsystem.percentOut(0), () -> jointSubsystem.stop(), jointSubsystem));
        driverXbox.povDown().whileTrue(Commands.startEnd(() -> jointSubsystem.percentOut(0), () -> jointSubsystem.stop(), jointSubsystem));
    }

    public void puncherButtonBindings() {
        driverXbox.y().whileTrue(Commands.startEnd(() -> puncherSubsystem.percentOut(0), () -> puncherSubsystem.stop(), puncherSubsystem));
    }
}
