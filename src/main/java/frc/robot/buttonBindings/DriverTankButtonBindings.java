package frc.robot.buttonBindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.HIDConstants;
import frc.robot.Robot;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.PuncherSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervetank.Drivetrain;
import swervelib.SwerveInputStream;

public class DriverTankButtonBindings {
    private final CommandXboxController driverXbox;

    private final Drivetrain drivetrain;
    private final IntakeSubsystem intakeSubsystem;
    private final JointSubsystem jointSubsystem;
    private final PuncherSubsystem puncherSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    private final Command driveCommand;

    private final Command intakeCommand;
    private final Command outtakeCommand;
    private final Command intakeStopCommand;
    private final Command jointAscendCommand;
    private final Command jointDescendCommand;
    private final Command puncherBuildUpCommand;
    private final Command puncherReleaseCommand;
    private final Command elevatorAscendCommand;
    private final Command elevatorDescendCommand;
    private final Command driverFeedbackCommand;

    //private final ConditionalCommand puncherConditionalCommand;

    private final Command testPuncherBuildUpCommand;
    private final Command testPuncherReleaseCommand;
    private final Command testPuncherPrintCommand;
    private final Command testIntakeCommand;
    private final Command testOuttakeCommand;
    private final Command testElevatorAscendCommand;
    private final Command testElevatorDescendCommand;
    private final Command testJointCommand;
    

    public DriverTankButtonBindings(CommandXboxController driverXbox, Drivetrain drivetrain, 
                                IntakeSubsystem intakeSubsystem, JointSubsystem jointSubsystem, PuncherSubsystem puncherSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.driverXbox = driverXbox;

        this.drivetrain = drivetrain;
        this.intakeSubsystem = intakeSubsystem;
        this.jointSubsystem = jointSubsystem;
        this.puncherSubsystem = puncherSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        driveCommand = this.drivetrain.drive(this.driverXbox::getLeftY, () -> this.driverXbox.getRightX());

        intakeCommand = this.intakeSubsystem.percentOutCommand(() -> Constants.IntakeConstants.INTAKE);
        outtakeCommand = this.intakeSubsystem.percentOutCommand(() -> Constants.IntakeConstants.OUTTAKE);
        intakeStopCommand = this.intakeSubsystem.percentOutCommand(() -> 0);
        jointAscendCommand = this.jointSubsystem.percentOutCommand(() -> Constants.JointConstants.ASCEND);
        jointDescendCommand = this.jointSubsystem.percentOutCommand(() -> Constants.JointConstants.DESCEND);
        puncherBuildUpCommand = this.puncherSubsystem.buildUpCommand(() -> Constants.PuncherConstants.POSITION);
        puncherReleaseCommand = this.puncherSubsystem.releaseCommand(() -> Constants.PuncherConstants.RELEASE, () -> Constants.PuncherConstants.TIGHTEN);
        elevatorAscendCommand = this.elevatorSubsystem.percentOutCommand(() -> Constants.ElevatorConstants.ASCEND);
        elevatorDescendCommand = this.elevatorSubsystem.descendCommand();

        

        driverFeedbackCommand = Commands.startEnd(() -> this.driverXbox.setRumble(RumbleType.kBothRumble, 1), () -> this.driverXbox.setRumble(RumbleType.kBothRumble, 0), puncherSubsystem).withTimeout(2);
        //puncherConditionalCommand = new ConditionalCommand(puncherBuildUpCommand, driverFeedbackCommand, () -> jointSubsystem.isSafeBuildUpAngle() && !puncherSubsystem.isPuncherReady());

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
        drivebaseButtonBindings();

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

    public void drivebaseButtonBindings() {
        drivetrain.setDefaultCommand(driveCommand);
    }

    public void intakeTestButtonBindings() {
        driverXbox.y().whileTrue(testIntakeCommand); //TESTAR
        driverXbox.x().whileTrue(testOuttakeCommand); //TESTAR
        driverXbox.b().whileTrue(intakeStopCommand); //TESTAR
    }

    public void jointTestButtonBindings() {
        //driverXbox.a().whileTrue(testJointCommand);
        driverXbox.povUp().whileTrue(jointAscendCommand);
        driverXbox.povDown().whileTrue(jointDescendCommand);
    }

    public void puncherTestButtonBindings() {
        //driverXbox.leftBumper().onTrue(puncherConditionalCommand); //TESTAR
        driverXbox.leftBumper().whileTrue(puncherBuildUpCommand);
        driverXbox.rightBumper().onTrue(testPuncherReleaseCommand);

    }

    public void elevatorTestButtonBindings() {
        driverXbox.povLeft().whileTrue(testElevatorAscendCommand);
        driverXbox.povRight().whileTrue(testElevatorDescendCommand);
    }

    public void intakeButtonBindings() {
        driverXbox.y().whileTrue(intakeCommand); //TESTAR
        driverXbox.x().whileTrue(outtakeCommand); //TESTAR
        driverXbox.b().whileTrue(intakeStopCommand); //TESTAR
    }

    public void jointButtonBindings() {
        driverXbox.povUp().whileTrue(jointAscendCommand);
        driverXbox.povDown().whileTrue(jointDescendCommand);
    }

    public void puncherButtonBindings() {
        driverXbox.leftBumper().whileTrue(puncherBuildUpCommand);
        driverXbox.rightBumper().onTrue(testPuncherReleaseCommand);
    }

    public void elevatorButtonBindings() {
        driverXbox.povLeft().whileTrue(elevatorAscendCommand);
        driverXbox.povRight().whileTrue(elevatorDescendCommand);
    }
}
