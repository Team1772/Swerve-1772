package frc.robot.buttonBindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.HIDConstants;
import frc.robot.Robot;

import frc.robot.subsystems.ElevatorSubsystem;
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
    private final ElevatorSubsystem elevatorSubsystem;

    private final SwerveInputStream driveDirectAngleSwerveInputStream;
    private final SwerveInputStream driveRobotOrientedSwerveInputStream;
    private final SwerveInputStream driveAngularVelocityKeyboardSwerveInputStream;
    private final SwerveInputStream driveDirectAngleKeyboardSwerveInputStream;
    private final SwerveInputStream driveAngularVelocitySwerveInputStream;

    private final Command driveFieldOrientedDirectAngleCommand;
    private final Command driveFieldOrientedAngularVelocityCommand;
    private final Command driveRobotOrientedAngularVelocityCommand;
    private final Command driveSetpointGenCommand;
    private final Command driveFieldOrientedDirectAngleKeyboardCommand;
    private final Command driveFieldOrientedAngularVelocityKeyboardCommand;
    private final Command driveSetpointGenKeyboardCommand;

    private final Command swerveZeroGyroCommand;
    private final Command swerveLockPoseCommand;
    private final Command swerveAddFakeVisionReadingCommand;
    private final Command swerveCenterModulesCommand;

    private final Command intakeCommand;
    private final Command outtakeCommand;
    private final Command jointAscendCommand;
    private final Command jointDescendCommand;
    private final Command puncherBuildUpCommand;
    private final Command puncherReleaseCommand;
    private final Command elevatorAscendCommand;
    private final Command elevatorDescendCommand;

    private final Command testPuncherBuildUpCommand;
    private final Command testPuncherReleaseCommand;
    private final Command testPuncherPrintCommand;
    private final Command testIntakeCommand;
    private final Command testOuttakeCommand;
    private final Command testElevatorAscendCommand;
    private final Command testElevatorDescendCommand;
    private final Command testJointCommand;
    

    public DriverButtonBindings(CommandXboxController driverXbox, SwerveSubsystem swerveSubsystem, 
                                IntakeSubsystem intakeSubsystem, JointSubsystem jointSubsystem, PuncherSubsystem puncherSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.driverXbox = driverXbox;

        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.puncherSubsystem = puncherSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        driveAngularVelocitySwerveInputStream = SwerveInputStream.of(this.swerveSubsystem.getSwerveDrive(),
            () -> this.driverXbox.getLeftY() * -1,
            () -> this.driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> {
                double leftTrigger = this.driverXbox.getLeftTriggerAxis();  
                double rightTrigger = this.driverXbox.getRightTriggerAxis(); 
                double rotation = rightTrigger - leftTrigger;
                return rotation;
            })
            .deadband(HIDConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

        driveDirectAngleSwerveInputStream = driveAngularVelocitySwerveInputStream.copy().withControllerHeadingAxis
            (this.driverXbox::getRightX, this.driverXbox::getRightY).headingWhile(true);

        driveRobotOrientedSwerveInputStream = driveAngularVelocitySwerveInputStream.copy().robotRelative(true)
            .allianceRelativeControl(false);

        driveAngularVelocityKeyboardSwerveInputStream = SwerveInputStream.of(this.swerveSubsystem.getSwerveDrive(),
            () -> -this.driverXbox.getLeftY(),
            () -> -this.driverXbox.getLeftX())
            .withControllerRotationAxis(() -> this.driverXbox.getRawAxis(
                2))
            .deadband(HIDConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

        driveDirectAngleKeyboardSwerveInputStream = driveAngularVelocityKeyboardSwerveInputStream.copy()
            .withControllerHeadingAxis(() -> Math.sin(
            this.driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
            () -> Math.cos(this.driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
            .headingWhile(true);

        driveFieldOrientedDirectAngleCommand = this.swerveSubsystem.driveFieldOriented(driveDirectAngleSwerveInputStream);
        driveFieldOrientedAngularVelocityCommand = this.swerveSubsystem.driveFieldOriented(driveAngularVelocitySwerveInputStream);
        driveRobotOrientedAngularVelocityCommand = this.swerveSubsystem.driveFieldOriented(driveRobotOrientedSwerveInputStream);
        driveSetpointGenCommand = this.swerveSubsystem.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSwerveInputStream);
        driveFieldOrientedDirectAngleKeyboardCommand = this.swerveSubsystem.driveFieldOriented(driveDirectAngleKeyboardSwerveInputStream);
        driveFieldOrientedAngularVelocityKeyboardCommand = this.swerveSubsystem.driveFieldOriented(driveAngularVelocityKeyboardSwerveInputStream);
        driveSetpointGenKeyboardCommand = this.swerveSubsystem.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboardSwerveInputStream);

        swerveZeroGyroCommand = Commands.runOnce(this.swerveSubsystem::zeroGyro);
        swerveLockPoseCommand = Commands.runOnce(this.swerveSubsystem::lock, this.swerveSubsystem).repeatedly();
        swerveAddFakeVisionReadingCommand = Commands.runOnce(this.swerveSubsystem::addFakeVisionReading);
        swerveCenterModulesCommand = this.swerveSubsystem.centerModulesCommand();

        intakeCommand = this.intakeSubsystem.percentOutCommand(() -> Constants.IntakeConstants.INTAKE);
        outtakeCommand = this.intakeSubsystem.percentOutCommand(() -> Constants.IntakeConstants.OUTTAKE);
        jointAscendCommand = this.jointSubsystem.percentOutCommand(() -> Constants.JointConstants.ASCEND);
        jointDescendCommand = this.jointSubsystem.percentOutCommand(() -> Constants.JointConstants.DESCEND);
        puncherBuildUpCommand = this.puncherSubsystem.buildUpCommand(() -> Constants.PuncherConstants.POSITION);
        puncherReleaseCommand = this.puncherSubsystem.releaseCommand(() -> Constants.PuncherConstants.RELEASE, () -> Constants.PuncherConstants.TIGHTEN, () -> Constants.PuncherConstants.TIMER);
        elevatorAscendCommand = this.elevatorSubsystem.percentOutCommand(() -> Constants.ElevatorConstants.ASCEND);
        elevatorDescendCommand = this.elevatorSubsystem.percentOutCommand(() -> Constants.ElevatorConstants.DESCEND);

        testPuncherBuildUpCommand = this.puncherSubsystem.testBuildUpCommand();
        testPuncherReleaseCommand = this.puncherSubsystem.testReleaseCommand();
        testPuncherPrintCommand = this.puncherSubsystem.testPrintCommand();
        testIntakeCommand = this.intakeSubsystem.testIntakeCommand();
        testOuttakeCommand = this.intakeSubsystem.testOuttakeCommand();
        testElevatorAscendCommand = this.elevatorSubsystem.testAscendCommand();
        testElevatorDescendCommand = this.elevatorSubsystem.testDescendCommand();
        testJointCommand = this.jointSubsystem.testOpenLoopCommand();
    }

    public void configureBindings() {
        drivebaseDefaultButtonBindings();
        drivebaseSimulationButtonBindings();
        drivebaseTestButtonBindings();

        if(Constants.DEV_MODE) {
            intakeTestButtonBindings();
            jointTestButtonBindings();
            puncherTestButtonBindings();
            elevatorTestButtonBindings();
        } else {
            intakeButtonBindings();
            jointButtonBindings();
            puncherButtonBindings();
            elevatorButtonBindings();
        }
    }

    public void drivebaseDefaultButtonBindings() {
        if (!RobotBase.isSimulation()) {
        swerveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocityCommand);
        driverXbox.leftBumper().whileTrue(driveRobotOrientedAngularVelocityCommand);
        }

        if (!DriverStation.isTest()) {
        driverXbox.start().onTrue(swerveZeroGyroCommand);
        driverXbox.leftStick().whileTrue(swerveLockPoseCommand);
        }
    }

    public void drivebaseSimulationButtonBindings() {
        if (RobotBase.isSimulation()) {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngleKeyboardCommand);
        } 

        if (Robot.isSimulation()) {
        driverXbox.start().onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
        driverXbox.button(1).whileTrue(swerveSubsystem.sysIdDriveMotorCommand());
        }
    }

    public void drivebaseTestButtonBindings() {
        if (DriverStation.isTest()) {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityCommand);
        driverXbox.x().whileTrue(swerveLockPoseCommand);
        driverXbox.start().onTrue(swerveZeroGyroCommand);
        driverXbox.back().whileTrue(swerveCenterModulesCommand);
        driverXbox.x().onTrue(swerveAddFakeVisionReadingCommand);
        }
    }

    public void intakeTestButtonBindings() {
        driverXbox.y().whileTrue(testIntakeCommand);
        driverXbox.x().whileTrue(testOuttakeCommand); 
    }

    public void jointTestButtonBindings() {
        driverXbox.a().whileTrue(testJointCommand);
    }

    public void puncherTestButtonBindings() {
        driverXbox.leftBumper().whileTrue(testPuncherBuildUpCommand);
        driverXbox.rightBumper().onTrue(testPuncherReleaseCommand);
    }

    public void elevatorTestButtonBindings() {
        driverXbox.x().whileTrue(testElevatorAscendCommand);
        driverXbox.b().whileTrue(testElevatorDescendCommand);
    }

    public void intakeButtonBindings() {
        driverXbox.y().whileTrue(intakeCommand);
        driverXbox.x().whileTrue(outtakeCommand); 
    }

    public void jointButtonBindings() {
        driverXbox.povUp().whileTrue(jointAscendCommand);
        driverXbox.povDown().whileTrue(jointDescendCommand);
    }

    public void puncherButtonBindings() {
        driverXbox.rightTrigger().whileTrue(puncherBuildUpCommand);
        driverXbox.rightBumper().onTrue(puncherReleaseCommand);
    }

    public void elevatorButtonBindings() {
        driverXbox.x().whileTrue(elevatorAscendCommand);
        driverXbox.b().whileTrue(elevatorDescendCommand);
    }
}
