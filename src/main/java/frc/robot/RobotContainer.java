// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.auto.AutoDistance;
import frc.robot.commands.auto.AutoRotate;
import frc.robot.commands.auto.PathFollow;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Joystick jsDrive;

  private ArcadeDrive m_arcadeDrive;

  private Drivetrain m_drive;
  private SendableChooser<Command> autoModeSwitcher;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    jsDrive = new Joystick(Constants.Controller.CONTROLLER_0);

    m_drive = new Drivetrain();

    m_arcadeDrive = new ArcadeDrive(m_drive,
      () -> { return -jsDrive.getRawAxis(Constants.Controller.AXIS_LIN)*Constants.Drive.DRIVE_MODIFIER;},
      () -> { return -jsDrive.getRawAxis(Constants.Controller.AXIS_ROT)*Constants.Drive.TURN_MODIFIER;});

    autoModeSwitcher = new SendableChooser<Command>();
    
    // Configure the button bindings
    configureButtonBindings();

    autoModeSwitcher.setDefaultOption("None", new InstantCommand());
    autoModeSwitcher.addOption("Test Turn", new AutoRotate(m_drive,90,3000));
    autoModeSwitcher.addOption("Test Drive", new AutoDistance(m_drive,0.25,3000));
    
    Shuffleboard.getTab("Autonomous").add(autoModeSwitcher);
  }


  public void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(m_drive,m_arcadeDrive); //sets the Drivetrain to default to ArcadeDrive in teleop
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return autoModeSwitcher.getSelected();
    return new PathFollow(m_drive);
  }
}
