// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.JoyStickConstants;

public class SwerveJoyStickDrive extends CommandBase {

  public final SwerveSubsystem swerveSubsystem;
  public final Supplier<Double> xSpdFunc, ySpdFunc, turnSpdFunc;
  public final Supplier<Boolean> fieldOrientedFunc;

  // private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  /** Creates a new SwerveJoyStickDrive. */
  public SwerveJoyStickDrive(SwerveSubsystem swerveSubsystem, 
  Supplier<Double> xSpdFunc, Supplier<Double> ySpdFunc, Supplier<Double> turnSpdFunc, Supplier<Boolean> fieldOrientedFunc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunc = xSpdFunc;
    this.ySpdFunc = ySpdFunc;
    this.turnSpdFunc = turnSpdFunc;
    this.fieldOrientedFunc = fieldOrientedFunc;
    // this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gets inputs real time (every tick)
    double xSpd = xSpdFunc.get();
    double ySpd = ySpdFunc.get();
    double turnSpd = turnSpdFunc.get();

    // Adds a deadzone to the joystics
    xSpd = Math.abs(xSpd) > JoyStickConstants.deadzone ? xSpd : 0;
    ySpd = Math.abs(ySpd) > JoyStickConstants.deadzone ? ySpd : 0;

    // xSpd = xLimiter.calculate(xSpd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // ySpd = yLimiter.calculate(ySpd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // turnSpd = turningLimiter.calculate(turnSpd)
    //         * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;
    // Allows the robot to drive relative to the field rather than relative to the direction its facing
    if (fieldOrientedFunc.get()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpd, ySpd, turnSpd, swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpd, ySpd, turnSpd);
    }

    SwerveModuleState[] moduleStates = DrivetrainConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
