// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm
  private static final double kWheelDiameterMeters = kWheelDiameterInch * 0.0254; // 70 mm in meters

  private static final double kTrackwidthMeters = 0.14; // 6 inches

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

  private final Field2d m_field = new Field2d();

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_gyro.getAngle()), getLeftDistanceMeter(), getRightDistanceMeter());

    //create the robot config from the GUI settings. This will be used to configure the auto builder.
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }


    AutoBuilder.configure(m_odometry::getPoseMeters, m_odometry::resetPose, this::getRobotRelativeSpeeds, this::driveFeedForwards, new PPLTVController(0.2), config, 
    () -> {

      var alliance = DriverStation.getAlliance().get();
      if(alliance == DriverStation.Alliance.Red) {
        return true;
      } else {
        return false;
      }
    }, this);

    SmartDashboard.putData("Field", m_field);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  public DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  //get the robot relative chassiss speeds using motor speeds. This assumes zero drift.
  public ChassisSpeeds getRobotRelativeSpeeds() {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());

    return kinematics.toChassisSpeeds(wheelSpeeds);
  }

  public void driveFeedForwards(ChassisSpeeds speeds, DriveFeedforwards driveFeedforwards) {
      DifferentialDriveWheelSpeeds differentialDriveWheelSpeeds = kinematics.toWheelSpeeds(speeds);

      m_diffDrive.tankDrive(differentialDriveWheelSpeeds.leftMetersPerSecond, differentialDriveWheelSpeeds.rightMetersPerSecond);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(m_gyro.getAngle()), getLeftDistanceMeter(), getRightDistanceMeter());

    SmartDashboard.putNumber("DistanceMetersLeft", getLeftDistanceMeter());
    SmartDashboard.putNumber("DistanceMetersRight", getRightDistanceMeter());

    m_field.setRobotPose(m_odometry.getPoseMeters());

    double angle = m_odometry.getPoseMeters().getRotation().getDegrees();
    SmartDashboard.putNumber("pose x", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("pose y", m_odometry.getPoseMeters().getY());

  }
}
