// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoRotate extends CommandBase {
  /** Creates a new TimedAutoDistance. */
  private Drivetrain m_drive;
  private ProfiledPIDController m_controller;
  private double m_rotation;
  private long m_timeoutMillis;
  private long startTime;

  public AutoRotate(Drivetrain drive, double rotationDegrees, long timeoutMillis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;
    m_timeoutMillis = timeoutMillis;
    m_controller = new ProfiledPIDController(Constants.Drive.AUTO_ROTATE_KP,
        Constants.Drive.AUTO_ROTATE_KI,
        Constants.Drive.AUTO_ROTATE_KD,
        new Constraints(Constants.Drive.AUTO_ROTATE_MAX_V,
            Constants.Drive.AUTO_ROTATE_MAX_A));
    m_controller.enableContinuousInput(-180, 180);
    m_rotation = rotationDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.reset(m_drive.getGyroAngle());
    startTime = System.currentTimeMillis();
    m_controller.setGoal(m_drive.getGyroAngle() + m_rotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.basicMove(-m_controller.calculate(m_drive.getGyroAngle()),m_controller.calculate(m_drive.getGyroAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_controller.atGoal()||System.currentTimeMillis()-startTime>=m_timeoutMillis);
  }
}
