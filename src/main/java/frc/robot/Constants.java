// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants {
    public static final int kLeftMotorChannel = 0;
    public static final int kRightMotorChannel = 1;

    public static final int kLeftEncoderA = 4;
    public static final int kLeftEncoderB = 5;
    public static final int kRightEncoderA = 6;
    public static final int kRightEncoderB = 7;

    // Maximum arc curvature: degrees of desired heading change per 20 ms cycle at full
    // joystick deflection. The joystick axis scales this from -kMaxArcCurvature to
    // +kMaxArcCurvature (up = arc right, down = arc left, center = straight).
    public static final double kMaxArcCurvature = 0.3;

    // PID gains for IMU-based heading feedback (output feeds arcadeDrive rotation, range ±1).
    public static final double kHeadingP = 0.1;
    public static final double kHeadingI = 0.0;
    public static final double kHeadingD = 0.00;

    // Clamps how far ahead the desired heading can lead actual heading (degrees).
    // Prevents accumulated error from overwhelming the PID and killing forward motion.
    public static final double kMaxHeadingLead = 5.0;

    // Max PID rotation contribution so forward motion is never fully overridden.
    public static final double kMaxRotationOutput = 0.3;

    // Rotation joystick input below this threshold engages arc mode.
    public static final double kRotationDeadband = 0.05;
  }

  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}
