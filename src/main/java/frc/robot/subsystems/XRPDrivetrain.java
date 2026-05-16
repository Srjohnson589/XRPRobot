// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class XRPDrivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm
  private static final double kObstacleDistanceInch = 10.0;

  // XRP channels 0 and 1 are left and right drive motors
  private final XRPMotor m_leftMotor = new XRPMotor(DriveConstants.kLeftMotorChannel);
  private final XRPMotor m_rightMotor = new XRPMotor(DriveConstants.kRightMotorChannel);

  // XRP onboard encoders are hardcoded to DIO pins 4/5 and 6/7
  private final Encoder m_leftEncoder =
      new Encoder(DriveConstants.kLeftEncoderA, DriveConstants.kLeftEncoderB);
  private final Encoder m_rightEncoder =
      new Encoder(DriveConstants.kRightEncoderA, DriveConstants.kRightEncoderB);

  private final XRPRangefinder m_rangefinder = new XRPRangefinder();
  private final XRPGyro m_gyro = new XRPGyro();

  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Heading feedback: PID drives actual yaw to a desired heading that drifts
  // at kArcCurvature degrees/cycle × forward speed, producing a constant-radius arc.
  private final PIDController m_headingController = new PIDController(
      DriveConstants.kHeadingP, DriveConstants.kHeadingI, DriveConstants.kHeadingD);
  private double m_desiredHeading = 0.0;

  public XRPDrivetrain() {
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
    m_rightMotor.setInverted(true);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate, double curvature) {
    double rangeInches = m_rangefinder.getDistanceInches();
    if (xaxisSpeed > 0 && rangeInches > 0 && rangeInches <= kObstacleDistanceInch) {
      xaxisSpeed = 0;
    }

    if (Math.abs(zaxisRotate) < DriveConstants.kRotationDeadband) {
      // Arc mode: advance desired heading by curvature × speed each 20 ms cycle,
      // then let the PID close the error between actual and desired yaw.
      double actual = m_gyro.getAngle();
      m_desiredHeading += xaxisSpeed * curvature;
      // Prevent desired heading from running too far ahead — caps PID error and
      // stops rotation from overwhelming forward motion when the robot can't keep up.
      m_desiredHeading = MathUtil.clamp(
          m_desiredHeading,
          actual - DriveConstants.kMaxHeadingLead,
          actual + DriveConstants.kMaxHeadingLead);
      double rotation = MathUtil.clamp(
          m_headingController.calculate(actual, m_desiredHeading),
          -DriveConstants.kMaxRotationOutput,
          DriveConstants.kMaxRotationOutput);
      m_diffDrive.arcadeDrive(xaxisSpeed, rotation);
    } else {
      // Manual turn: re-sync desired heading so arc resumes smoothly when released.
      m_desiredHeading = m_gyro.getAngle();
      m_headingController.reset();
      m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    }
  }

  public void resetHeading() {
    m_gyro.reset();
    m_desiredHeading = 0.0;
    m_headingController.reset();
  }

  public double getHeading() {
    return m_gyro.getAngle();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
