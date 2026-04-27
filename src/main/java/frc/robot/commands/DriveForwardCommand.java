// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

/** Drives the XRP straight forward a specified distance (in inches) at half speed. */
public class DriveForwardCommand extends Command {
  private final XRPDrivetrain m_drivetrain;
  private final double m_targetDistanceInch;

  public DriveForwardCommand(XRPDrivetrain drivetrain, double distanceInch) {
    m_drivetrain = drivetrain;
    m_targetDistanceInch = distanceInch;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
  }

  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(0.5, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_drivetrain.getAverageDistanceInch() >= m_targetDistanceInch;
  }
}
