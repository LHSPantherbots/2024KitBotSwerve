// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {

  private Pigeon2 m_gyro = new Pigeon2(DriveConstants.kPigeonCAN_ID);

  private double lastTimestamp = Timer.getFPGATimestamp();
  private double lastAngle = 0.0;

  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftAngleZero);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kRearLeftAngleZero);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightAngleZero);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kRearRightAngleZero);

  private final LimeLight m_limeLight = new LimeLight();

  // PoseEstimator class for tracking robot pose with vision
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    new Thread(
        () -> {
          try {
            Thread.sleep(1000);
            zeroHeading();
          } catch (Exception e) {
          }
        })
        .start();

    new Thread(
        () -> {
          try {
            Thread.sleep(1000);
            zeroHeading();
          } catch (Exception e) {
          }
        })
        .start();
    if (m_limeLight.hasLockedVisionTarget()) {
      m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        m_limeLight.getBotPose3d().toPose2d());
    } else {
      m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d());
    }
    
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    if (m_limeLight.isTargetValid()) {
      m_poseEstimator.addVisionMeasurement(m_limeLight.getBotPose3d().toPose2d(), Timer.getFPGATimestamp() - (m_limeLight.getTargetLatency()/1000.0) - (m_limeLight.getCaptureLatency()/1000.0));
    }

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("Front Left Angle", m_frontLeft.getModuleAngle());
    SmartDashboard.putNumber("Front Right Angle", m_frontRight.getModuleAngle());
    SmartDashboard.putNumber("Rear Left Angle", m_rearLeft.getModuleAngle());
    SmartDashboard.putNumber("Rear Right Angle", m_rearRight.getModuleAngle());
    SmartDashboard.putNumber("Front Left Pos, m", m_frontLeft.getDriveEncoderPositionMeter());
    SmartDashboard.putNumber("Front Right Pos, m", m_frontRight.getDriveEncoderPositionMeter());
    SmartDashboard.putNumber("Back Left Pos, m", m_rearLeft.getDriveEncoderPositionMeter());
    SmartDashboard.putNumber("Back Right Pos, m", m_rearRight.getDriveEncoderPositionMeter());
    SmartDashboard.putNumber(
        "Front Left Velocity, m/s", m_frontLeft.getDriveEncoderVelocityMeterPerSec());
    SmartDashboard.putNumber(
        "Front Right Velocity, m/s", m_frontRight.getDriveEncoderVelocityMeterPerSec());
    SmartDashboard.putNumber(
        "Back Left Velocity, m/s", m_rearLeft.getDriveEncoderVelocityMeterPerSec());
    SmartDashboard.putNumber(
        "Back Right Velocity, m/s", m_rearRight.getDriveEncoderVelocityMeterPerSec());
    SmartDashboard.putString("FR Actual State", m_frontRight.getState().toString());

    SmartDashboard.putNumber(
        "Front Right Encoder Distance", m_frontRight.getDriveEncoderPositionMeter());
    SmartDashboard.putNumber(
        "Front Right Encoder Speed Meter/s", m_frontRight.getDriveEncoderVelocityMeterPerSec());

    SmartDashboard.putNumber("Front Left Wheel Rotations", m_frontLeft.getModuleAbsoluteAngle());

    SmartDashboard.putNumber("Gyro Angle", getHeading());
    SmartDashboard.putString("Pose", getPose().toString());
    // SmartDashboard.putData("Pose", getPose());
    SmartDashboard.putString("Vison Pose", m_limeLight.getBotPose3d().toString());
    SmartDashboard.putString("Rot 2d", getYaw().toString());

    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());

    SmartDashboard.putNumber("Roll Rate", getRollRate());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // return m_odometry.getPoseMeters();
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getYaw(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void resetOdometryReverse(Pose2d pose) {
    m_poseEstimator.resetPosition(
        new Rotation2d(Math.PI),
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
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    SmartDashboard.putString("FR2 Set State", swerveModuleStates[1].toString());
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
    SmartDashboard.putString("FR Set State", desiredStates[1].toString());
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0.0);
  }

  public Rotation2d getYaw() {
    double yawRadians = Math.toRadians(m_gyro.getYaw().getValueAsDouble());
    return new Rotation2d(yawRadians);
  }

  public void xWheels() {
    drive(0.0, 0.0, 0.8, true);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getYaw().getDegrees();
  }

  public double getPitch() {
    return m_gyro.getPitch().getValueAsDouble();
  }

  public double getRoll() {
    return m_gyro.getRoll().getValueAsDouble();
  }

  public double getRollRate() {
    double currentTimestamp = Timer.getFPGATimestamp();
    double currentAngle = m_gyro.getRoll().getValueAsDouble();

    double rollRate = (currentAngle - lastAngle) / (currentTimestamp - lastTimestamp);

    lastTimestamp = currentTimestamp;
    lastAngle = currentAngle;

    return rollRate;
  }

  public void resetAll() {
    resetEncoders();
    zeroHeading();
  }

  public void restAll180() {
    resetEncoders();
    m_gyro.setYaw(180.0);
  }

  public double getHeadingRadians() {
    return getYaw().getRadians();
  }

  public void restOdomWithCamData() {
    m_poseEstimator.resetPosition(
      new Rotation2d(getHeadingRadians()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      m_limeLight.getLastPose3d().toPose2d()
    );
  }
}
