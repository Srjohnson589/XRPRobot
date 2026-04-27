// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.subsystems.XRPDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XRPDrivetrain m_drivetrain = new XRPDrivetrain();

  // Use Joystick so keyboard input from the DS "Keyboard 0" slot works
  // W/S → Axis 1 (forward/back), A/D → Axis 0 (turn)
  private final Joystick m_keyboard =
      new Joystick(Constants.OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // W/S drives forward/back (axis 1), A/D turns (axis 0)
    m_drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                m_drivetrain.arcadeDrive(
                    -m_keyboard.getRawAxis(1), m_keyboard.getRawAxis(0)),
            m_drivetrain));
  }

  public Command getAutonomousCommand() {
    // Drive forward 10 inches in autonomous
    return new DriveForwardCommand(m_drivetrain, 10.0);
  }
}
