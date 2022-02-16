  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {
  // Sets up all of the swerve modules with our constants
  private final SwerveModule frontLeftModule = new SwerveModule(
    DrivetrainConstants.FL_DriveID,
    DrivetrainConstants.FL_TurnID,
    DrivetrainConstants.FL_CancoderID,
    DrivetrainConstants.FL_driveReversed,
    DrivetrainConstants.FL_turnReversed,
    0
  );

  private final SwerveModule frontRightModule = new SwerveModule(
    DrivetrainConstants.FR_DriveID,
    DrivetrainConstants.FR_TurnID,
    DrivetrainConstants.FR_CancoderID,
    DrivetrainConstants.FR_driveReversed,
    DrivetrainConstants.FR_turnReversed,
    1
  );

  private final SwerveModule backLeftModule = new SwerveModule(
    DrivetrainConstants.BL_DriveID,
    DrivetrainConstants.BL_TurnID,
    DrivetrainConstants.BL_CancoderID,
    DrivetrainConstants.BL_driveReversed,
    DrivetrainConstants.BL_turnReversed,
    2
  );

  private final SwerveModule backRightModule = new SwerveModule(
    DrivetrainConstants.BR_DriveID,
    DrivetrainConstants.BR_TurnID,
    DrivetrainConstants.BR_CancoderID,
    DrivetrainConstants.BR_driveReversed,
    DrivetrainConstants.BR_turnReversed,
    3
  );

    /** Creates a new SwerveSubsystem. */
    public SwerveSubsystem() {
      // Makes a separate thread to wait for gyro to boot up before zeroing it
      new Thread(() -> {
        try {
          Thread.sleep(2000);
          zeroGyro();
        } catch (Exception e) {
        }
      }).start();
    }

  // Sets up gyro to tell which direction the robot is facing
  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  public void zeroGyro() {
        gyro.reset();
  }

  public double getGyroPositionInRadians() {
    return Math.IEEEremainder(Math.toRadians(gyro.getAngle()), 2 * Math.PI);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getAngle(), 360));
  }

  // Stops the movement of the swerve drive
  public void stopModules() {
    frontLeftModule.stopModule();
    frontRightModule.stopModule();
    backLeftModule.stopModule();
    backRightModule.stopModule();
  }

  // ITS A LIST U NERD
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Makes it so the swerve drive can still turn even when all the wheels are being told to go to max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.physicalMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Position in Rad", getGyroPositionInRadians());
  }

}
