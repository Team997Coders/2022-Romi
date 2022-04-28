// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class PathFollow extends CommandBase {
  /** Creates a new PathFollow. */
  private Drivetrain m_drivetrain;
  private Trajectory m_trajectory;
  private Boolean trajectoryGenerated = false;
  private RamseteController m_ramsete;
  private DifferentialDriveKinematics m_Kinematics;
  private DifferentialDriveWheelSpeeds m_wheelSpeeds;
  private PIDController rightSideController;
  private PIDController leftSideController;
  private long timeMillis;
  public PathFollow(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drive;
    m_ramsete = new RamseteController(Constants.Drive.RAMSETE_B,Constants.Drive.RAMSETE_ZETA);
    m_Kinematics = new DifferentialDriveKinematics(Constants.Drive.DRIVE_TRACK_WIDTH_CM/100);
    m_wheelSpeeds = new DifferentialDriveWheelSpeeds();
    rightSideController = new PIDController(Constants.Drive.WHEEL_VELOCITY_KP, 
        Constants.Drive.WHEEL_VELOCITY_KI, Constants.Drive.WHEEL_VELOCITY_KD);
    leftSideController = new PIDController(Constants.Drive.WHEEL_VELOCITY_KP, 
      Constants.Drive.WHEEL_VELOCITY_KI, Constants.Drive.WHEEL_VELOCITY_KD);
    addRequirements(drive);
  }

  public void generateTrajectory() {
    m_trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(new Translation2d(1, 0)), 
      new Pose2d(0, 0, new Rotation2d(180)), 
      new TrajectoryConfig(Constants.Drive.PATH_FOLLOW_MAX_M_PER_S, 
        Constants.Drive.PATH_FOLLOW_MAX_M_PER_SEC_SQUARED));
    trajectoryGenerated = true;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetPose();
    m_drivetrain.resetEncoders();
    m_drivetrain.resetGyroAngle();
    if (!trajectoryGenerated) {
      generateTrajectory();
    }
    timeMillis = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wheelSpeeds = m_Kinematics.toWheelSpeeds(
        m_ramsete.calculate(m_drivetrain.getPose2d(), m_trajectory.sample((System.currentTimeMillis()-timeMillis)/1000)));
    rightSideController.setSetpoint(m_wheelSpeeds.rightMetersPerSecond);
    leftSideController.setSetpoint(m_wheelSpeeds.leftMetersPerSecond);

    m_drivetrain.basicMove(
        rightSideController.calculate(m_drivetrain.getRightAngularVMPS()), 
        leftSideController.calculate(m_drivetrain.getLeftAngularVMPS()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
