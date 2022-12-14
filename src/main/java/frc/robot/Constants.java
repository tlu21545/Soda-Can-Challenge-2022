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
    public static final class DriveConstants {
        public static final double kP_drive = 0.4; // proportional gain constant for driveStraight control loop
        public static final double kPAngular = 0.001;
        public static final boolean kGyroReversed = false;
        public static final double tLagAngular = 0.020;
        public static final double kTurnP = 0.1;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;

        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 10;
    }
}
