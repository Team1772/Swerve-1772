// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.buttonBindings.DriverButtonBindings;
import frc.robot.buttonBindings.OperatorButtonBindings;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.PuncherSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

public class RobotContainer {
  private final CommandXboxController driverXbox;
  private final CommandXboxController operatorXbox;

  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PuncherSubsystem puncherSubsystem;
  private final JointSubsystem jointSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private final DriverButtonBindings driverButtonBindings;
  private final OperatorButtonBindings operatorButtonBindings;
  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    driverXbox = new CommandXboxController(0);
    operatorXbox = new CommandXboxController(1);

    swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    intakeSubsystem = new IntakeSubsystem();
    puncherSubsystem = new PuncherSubsystem();
    jointSubsystem = new JointSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    

    driverButtonBindings = new DriverButtonBindings(driverXbox, swerveSubsystem, intakeSubsystem, 
                                                    jointSubsystem, puncherSubsystem, elevatorSubsystem);
    operatorButtonBindings = new OperatorButtonBindings(operatorXbox);
    configureBindings();

    autoChooser = new SendableChooser<Command> ();
    autoSetup();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    driverButtonBindings.configureBindings();
    operatorButtonBindings.configureBindings();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void autoSetup() {
    SmartDashboard.putData(autoChooser);
    
    autoChooser.setDefaultOption("No auto", new PrintCommand("No Auto Selected"));
    autoChooser.addOption("Also no auto", new PrintCommand("Also No Auto Selected"));

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  public void setMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
  }
}
