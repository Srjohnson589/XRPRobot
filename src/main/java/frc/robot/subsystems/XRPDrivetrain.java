// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  public XRPDrivetrain() {
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
    m_rightMotor.setInverted(true);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    if (xaxisSpeed > 0 && m_rangefinder.getDistanceInches() <= kObstacleDistanceInch) {
      xaxisSpeed = 0;
    }
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
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
