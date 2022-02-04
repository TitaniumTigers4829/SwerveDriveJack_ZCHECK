/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {

    public static final class ModuleConstants{
        public static double wheelDiameter = Units.inchesToMeters(4);
        public static double driveMotorToWheel = 1 / 7.13;
        public static double turningMotorWheelRatio = 1;
        public static double physicalMaxSpeedMetersPerSecond = 5;
        public static double driveEncToMeters_s = driveMotorToWheel * Math.PI * wheelDiameter;
        public static double turnEncRotToRad = turningMotorWheelRatio * 2 * Math.PI;
        public static double driveEncRPMToMPS = driveEncToMeters_s / 60;
        public static double turningEncRPMToRadPerS = turnEncRotToRad / 60;
        public static double kP = 0.5;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double wheelSpinUnitsToMetersPerSecond = 10 / 2048;
        public static double ticksToRadians = Math.PI / 512;
        public static double ticksToMeters = Math.PI / 512;
    }

    public static final class DrivetrainConstants{
        public static int FL_DriveID = 0-9;
        public static int FL_TurnID = 0-9;
        public static boolean FL_driveReversed = false;
        public static boolean FL_turnReversed = false;
        
        public static int FR_DriveID = 0-9;
        public static int FR_TurnID = 0-9;
        public static boolean FR_driveReversed = false;
        public static boolean FR_turnReversed = false;

        
        public static int BL_DriveID = 0-9;
        public static int BL_TurnID = 0-9;
        public static boolean BL_driveReversed = false;
        public static boolean BL_turnReversed = false;
        
        public static int BR_DriveID = 0-9;
        public static int BR_TurnID = 0-9;
        public static boolean BR_driveReversed = false;
        public static boolean BR_turnReversed = false;
    }

}
