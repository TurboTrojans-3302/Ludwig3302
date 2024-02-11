// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      new CANSparkMax(DriveConstants.kFrontLeftDrivingCanId, MotorType.kBrushless),
      new CANSparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless),
      new EddieCoder(DriveConstants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER),
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      new CANSparkMax(DriveConstants.kFrontRightDrivingCanId, MotorType.kBrushless),
      new CANSparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless),
      new EddieCoder(DriveConstants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER),
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      new CANSparkMax(DriveConstants.kRearLeftDrivingCanId, MotorType.kBrushless),
      new CANSparkMax(DriveConstants.kRearLeftTurningCanId, MotorType.kBrushless),
      new EddieCoder(DriveConstants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER),
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      new CANSparkMax(DriveConstants.kRearRightDrivingCanId, MotorType.kBrushless),
      new CANSparkMax(DriveConstants.kRearRightTurningCanId, MotorType.kBrushless),
      new EddieCoder(DriveConstants.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER),
      DriveConstants.kFrontLeftChassisAngularOffset);

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private ShuffleboardTab m_shuffleboardTab;
  private GenericEntry m_FLangleEntry;
  private GenericEntry m_FRangleEntry;
  private GenericEntry m_BLangleEntry;
  private GenericEntry m_BRangleEntry;
  private GenericEntry m_FLspeedEntry;
  private GenericEntry m_FRspeedEntry;
  private GenericEntry m_BLspeedEntry;
  private GenericEntry m_BRspeedEntry;
  private GenericEntry m_gyroEntry;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()

      });

  public void updateShuffleboard() {
    m_FLangleEntry.setDouble(m_frontLeft.getPosition().angle.getDegrees());
    m_FRangleEntry.setDouble(m_frontRight.getPosition().angle.getDegrees());
    m_BRangleEntry.setDouble(m_rearRight.getPosition().angle.getDegrees());
    m_BLangleEntry.setDouble(m_rearLeft.getPosition().angle.getDegrees());
    m_FLspeedEntry.setDouble(m_frontLeft.getState().speedMetersPerSecond);
    m_FRspeedEntry.setDouble(m_frontRight.getState().speedMetersPerSecond);
    m_BRspeedEntry.setDouble(m_rearRight.getState().speedMetersPerSecond);
    m_BLspeedEntry.setDouble(m_rearLeft.getState().speedMetersPerSecond);
    m_gyroEntry.setDouble(m_gyro.getAngle());
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_shuffleboardTab = Shuffleboard.getTab("Drive");
    m_FLangleEntry = m_shuffleboardTab.add("FLa", 0.0).getEntry();
    m_FRangleEntry = m_shuffleboardTab.add("FRa", 0.0).getEntry();
    m_BLangleEntry = m_shuffleboardTab.add("BLa", 0.0).getEntry();
    m_BRangleEntry = m_shuffleboardTab.add("BRa", 0.0).getEntry();
    m_FLspeedEntry = m_shuffleboardTab.add("FLs", 0.0).getEntry();
    m_FRspeedEntry = m_shuffleboardTab.add("FRs", 0.0).getEntry();
    m_BLspeedEntry = m_shuffleboardTab.add("BLs", 0.0).getEntry();
    m_BRspeedEntry = m_shuffleboardTab.add("BRs", 0.0).getEntry();
    m_gyroEntry = m_shuffleboardTab.add("Gyro", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    updateShuffleboard();

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
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setAll(double speed, double angle){
     m_frontLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)));
    m_frontRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)));
      m_rearLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)));
     m_rearRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)));
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
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public MAXSwerveModule getM_frontLeft() {
    return m_frontLeft;
  }

  public MAXSwerveModule getM_frontRight() {
    return m_frontRight;
  }

  public MAXSwerveModule getM_rearLeft() {
    return m_rearLeft;
  }

  public MAXSwerveModule getM_rearRight() {
    return m_rearRight;
  }

  public double getM_currentRotation() {
    return m_currentRotation;
  }

  public void setM_currentRotation(double m_currentRotation) {
    this.m_currentRotation = m_currentRotation;
  }

  public double getM_currentTranslationDir() {
    return m_currentTranslationDir;
  }

  public void setM_currentTranslationDir(double m_currentTranslationDir) {
    this.m_currentTranslationDir = m_currentTranslationDir;
  }

  public double getM_currentTranslationMag() {
    return m_currentTranslationMag;
  }

  public void setM_currentTranslationMag(double m_currentTranslationMag) {
    this.m_currentTranslationMag = m_currentTranslationMag;
  }

  public SlewRateLimiter getM_magLimiter() {
    return m_magLimiter;
  }

  public void setM_magLimiter(SlewRateLimiter m_magLimiter) {
    this.m_magLimiter = m_magLimiter;
  }

  public SlewRateLimiter getM_rotLimiter() {
    return m_rotLimiter;
  }

  public void setM_rotLimiter(SlewRateLimiter m_rotLimiter) {
    this.m_rotLimiter = m_rotLimiter;
  }

  public double getM_prevTime() {
    return m_prevTime;
  }

  public void setM_prevTime(double m_prevTime) {
    this.m_prevTime = m_prevTime;
  }

  public SwerveDriveOdometry getM_odometry() {
    return m_odometry;
  }

  public void setM_odometry(SwerveDriveOdometry m_odometry) {
    this.m_odometry = m_odometry;
  }
}
