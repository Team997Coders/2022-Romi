package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private Spark m_rightSpark;
  private Spark m_leftSpark;

  private Encoder m_rightEncoder;
  private Encoder m_leftEncoder;

  private RomiGyro m_gyro;

  private Field2d m_field2d;
  private DifferentialDriveOdometry m_odometry;


  /** Creates a new Drivetrain. */
  public Drivetrain() {

    m_rightSpark = new Spark(Constants.Ports.FRONT_RIGHT_MOTOR_PORT); // constructs motor controllers and
    m_leftSpark = new Spark(Constants.Ports.FRONT_LEFT_MOTOR_PORT); // assigns them to the correct device ID
    m_rightSpark.setInverted(true);
    m_leftSpark.setInverted(false);

    m_rightEncoder = new Encoder(Constants.Ports.RIGHT_ENCODER_PORT_A, Constants.Ports.RIGHT_ENCODER_PORT_B);
    m_leftEncoder = new Encoder(Constants.Ports.LEFT_ENCODER_PORT_A, Constants.Ports.LEFT_ENCODER_PORT_B);

    m_rightEncoder.setDistancePerPulse(Constants.Drive.DRIVE_METERS_PER_COUNT);
    m_leftEncoder.setDistancePerPulse(Constants.Drive.DRIVE_METERS_PER_COUNT);

    m_gyro = new RomiGyro();

    m_field2d = new Field2d();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    resetEncoders(); // makes sure they're at zero
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void basicMove(double right,double left) { // handy little wrapper for diffDrive
    m_leftSpark.set(left); // pass in processed values
    m_rightSpark.set(right);
  }

  public double getGyroAngle() {
    return m_gyro.getAngle();
  }

  public void resetGyroAngle() {
    m_gyro.reset();
  }

  public double getRightSensorPosition() {
    return m_rightEncoder.getDistance();
  }

  public double getLeftSensorPosition() {
    return m_leftEncoder.getDistance();
  }

  public double getRightAngularVMPS() {
    return m_rightEncoder.getRate();
  }

  public double getLeftAngularVMPS() {
    return m_leftEncoder.getRate();
  }
  
  public Pose2d getPose2d() {
    m_odometry.update(new Rotation2d(Math.toRadians(getGyroAngle())),
        (m_leftEncoder.getDistance()), (m_rightEncoder.getDistance()));
    return m_odometry.getPoseMeters();
  }

  public void resetPose() {
    m_odometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0)), new Rotation2d(0));
  }
  
  @Override
  public void periodic() {
    m_odometry.update(new Rotation2d(Math.toRadians(getGyroAngle())),
        (m_leftEncoder.getDistance()), (m_rightEncoder.getDistance()));
    m_field2d.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("Field",m_field2d);
    SmartDashboard.putNumber("Rotation",m_odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("Position X",m_odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Position Y",m_odometry.getPoseMeters().getTranslation().getY());

    SmartDashboard.putNumber("L Motor",m_leftEncoder.getDistance());
    SmartDashboard.putNumber("R Motor",m_rightEncoder.getDistance());
  }
}