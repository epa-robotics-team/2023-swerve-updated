// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.helpers.SubsystemInspector;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules

  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPortsjr,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed,
          0.359270);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningEncoderPortsjr,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed,
          0.309556);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningEncoderPortsjr,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          0.413859);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningEncoderPortsjr,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed,
          0.235461);



  // create the limiterkMaxAccelerationMetersPerSecondSquared
  private SlewRateLimiter swerveXAccelerationFilterLimiter = new SlewRateLimiter(Constants.DriveConstants.kMaxDriveAccelerationMetersPerSecondSquared);
  private SlewRateLimiter swerveYAccelerationFilterLimiter = new SlewRateLimiter(Constants.DriveConstants.kMaxDriveAccelerationMetersPerSecondSquared);


  // creates the inspector
  private final SubsystemInspector inspector = new SubsystemInspector("Drivetrain");

  // The gyro sensor
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(5);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), null, null);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
getRotation2d(), new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition() });
    inspector.set("EncoderLeftFrontX",m_frontLeft.getTurningRadians());
    inspector.set("EncoderLeftBackX", m_rearLeft.getTurningRadians());
    inspector.set("EncoderRightFrontX",m_frontRight.getTurningRadians());
    inspector.set("EncoderRightBackX", m_rearRight.getTurningRadians());
    inspector.set("Gyro", getRotation2d().getRadians());

    inspector.set("EncoderLeftFrontXAbsValue",m_frontLeft.getAbsPostition());
    inspector.set("EncoderLeftBackXAbsValue", m_rearLeft.getAbsPostition());
    inspector.set("EncoderRightFrontXAbsValue",m_frontRight.getAbsPostition());
    inspector.set("EncoderRightBackXAbsValue", m_rearRight.getAbsPostition());
    }
    
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    // m_odometry.resetPosition(pose, getRotation2d());
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var smoothedXThrottlePercent = swerveXAccelerationFilterLimiter.calculate(xSpeed);
    var smoothedYThrottlePercent =  swerveYAccelerationFilterLimiter.calculate(ySpeed);
    smoothedXThrottlePercent = MathUtil.applyDeadband(smoothedXThrottlePercent, Constants.joystickDeadband);
    smoothedYThrottlePercent = MathUtil.applyDeadband(smoothedYThrottlePercent, Constants.joystickDeadband);
    inspector.set("smoothedX", smoothedXThrottlePercent);
    inspector.set("smoothedY", smoothedYThrottlePercent);
    inspector.set("xSpeed", xSpeed);
    inspector.set("ySpeed", ySpeed);
    var chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(smoothedXThrottlePercent, smoothedYThrottlePercent, rot, getRotation2d())
        : new ChassisSpeeds(smoothedXThrottlePercent, smoothedYThrottlePercent  , rot);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
  

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}