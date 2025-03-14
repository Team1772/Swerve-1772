// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.buttonBindings.DriverButtonBindings;
import frc.robot.buttonBindings.DriverTankButtonBindings;
import frc.robot.buttonBindings.OperatorButtonBindings;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.PuncherSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervetank.Drivetrain;

import java.io.File;

public class RobotContainer {
  private final CommandXboxController driverXbox;
  private final CommandXboxController operatorXbox;

  private final SwerveSubsystem swerveSubsystem;
  //private final Drivetrain swerveSubsystem;

  private final IntakeSubsystem intakeSubsystem;
  private final PuncherSubsystem puncherSubsystem;
  private final JointSubsystem jointSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private final DriverButtonBindings driverButtonBindings;
  //private final DriverTankButtonBindings driverButtonBindings;

  private final OperatorButtonBindings operatorButtonBindings;
  SendableChooser<Command> autonomousChooser;

  public RobotContainer() {
    driverXbox = new CommandXboxController(0);
    operatorXbox = new CommandXboxController(1);

    swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    //swerveSubsystem = new Drivetrain();
    intakeSubsystem = new IntakeSubsystem();
    puncherSubsystem = new PuncherSubsystem();
    jointSubsystem = new JointSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    
    driverButtonBindings = new DriverButtonBindings(driverXbox, swerveSubsystem, intakeSubsystem, jointSubsystem, puncherSubsystem, elevatorSubsystem);
    //driverButtonBindings = new DriverTankButtonBindings(driverXbox, swerveSubsystem, intakeSubsystem, jointSubsystem, puncherSubsystem, elevatorSubsystem);

    operatorButtonBindings = new OperatorButtonBindings(operatorXbox);
    configureBindings();

    autonomousChooser = new SendableChooser<Command> ();
    autonomousChooserSetup();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    driverButtonBindings.configureBindings();
    operatorButtonBindings.configureBindings();
  }

  public Command getAutonomousCommand() {
    //return swerveSubsystem.autonomousCommand();
    return null; //autonomousChooser.getSelected();
  }

  public void autonomousChooserSetup() {
    Shuffleboard.getTab("Autonomous").add("Choose Autonomous Routine", autonomousChooser);
    
    autonomousChooser.setDefaultOption("No auto", new PrintCommand("No Auto Selected"));
    autonomousChooser.addOption("Also no auto", new PrintCommand("Also No Auto Selected"));

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  public void setMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
  }

  public void resetPuncherSubsystemEncoders() {
    puncherSubsystem.resetEncoders();
    puncherSubsystem.resetState();
  }

  public void zeroGyro() {
    swerveSubsystem.zeroGyro();
  }
}
