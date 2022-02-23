/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class ModuleConstants {
        public static double wheelDiameter = Units.inchesToMeters(4);
        public static double driveMotorToWheel = 1 / 7.13;
        public static double turningMotorWheelRatio = 1;
        public static double physicalMaxSpeedMetersPerSecond = 5;
        public static double driveEncToMeters_s = driveMotorToWheel * Math.PI * wheelDiameter;
        public static double turnEncRotToRad = turningMotorWheelRatio * 2 * Math.PI;
        public static double driveEncRPMToMPS = driveEncToMeters_s / 60;
        public static double turningEncRPMToRadPerS = turnEncRotToRad / 60;
        public static double kP = 1.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double wheelSpinUnitsToMetersPerSecond = 10 / 2048;
        public static double ticksToRadians = Math.PI / 512;
        public static double ticksToMeters = Math.PI / 512;
    }
    /**
     * hey u found it :)
     */
    public static final class DrivetrainConstants {
        public static double wheelBase = Units.inchesToMeters(16 + (5/16)); // front and back wheels are slightly further apart than right and left
        public static double trackWidth = Units.inchesToMeters(16.125);
        
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2));
        
        
        public static int FL_DriveID = 13;
        public static int FL_TurnID = 1;
        public static int FL_CancoderID = 11;
        // ALIGN THE WHEELS WITH THE GEARS FACING OUT!!!
        public static boolean FL_driveReversed = true;
        public static boolean FL_turnReversed = false;
        
        public static int FR_DriveID = 4;
        public static int FR_TurnID = 3;
        public static int FR_CancoderID = 10;
        public static boolean FR_driveReversed = false;
        public static boolean FR_turnReversed = false;
        
        public static int BL_DriveID = 6;
        public static int BL_TurnID = 7;
        public static int BL_CancoderID = 8;
        public static boolean BL_driveReversed = true;
        public static boolean BL_turnReversed = false;
        
        public static int BR_DriveID = 0;
        public static int BR_TurnID = 12;
        public static int BR_CancoderID = 9;
        public static boolean BR_driveReversed = false;
        public static boolean BR_turnReversed = false;
    }
    
    public static final class JoyStickConstants {
        public static int joyStickPort = 0;
        public static double deadzone = 0.01;
    }
    
}
