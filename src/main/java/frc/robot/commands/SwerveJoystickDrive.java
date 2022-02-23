// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.JoyStickConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveJoystickDrive extends CommandBase {
  
  public final SwerveSubsystem swerveSubsystem;
  public final Supplier<Double> xSpdFunc, ySpdFunc, turnSpdFunc;
  
  // private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  
  /**
  * Creates the drive command
  * @param swerveSubsystem instance of the SwerveSubsytem
  * @param xSpdFunc X speed
  * @param ySpdFunc Y speed
  * @param turnSpdFunc Turn speed
  */
  public SwerveJoystickDrive(SwerveSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunc, Supplier<Double> ySpdFunc, Supplier<Double> turnSpdFunc) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunc = xSpdFunc;
    this.ySpdFunc = ySpdFunc;
    this.turnSpdFunc = turnSpdFunc;
    
    // Implements a max rate of change of 'input' data coming form joysticks. Good for preventing
    // robot tipping, etc. However, if the drivers are good it can be left off.
    
    // this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gets inputs real time (every tick)
    double xSpd = xSpdFunc.get();
    double ySpd = ySpdFunc.get();
    double turnSpd = turnSpdFunc.get();
    
    // Adds a deadzone to the joysticks
    xSpd = Math.abs(xSpd) > JoyStickConstants.deadzone ? xSpd : 0;
    ySpd = Math.abs(ySpd) > JoyStickConstants.deadzone ? ySpd : 0;
    
    // xSpd = xLimiter.calculate(xSpd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // ySpd = yLimiter.calculate(ySpd) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // turnSpd = turningLimiter.calculate(turnSpd)
    //         * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
    ChassisSpeeds chassisSpeeds;
    // E meme
    // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    // xSpd, ySpd, turnSpd, swerveSubsystem.getRotation2d());
    chassisSpeeds = new ChassisSpeeds(xSpd, ySpd, turnSpd);
    
    SwerveModuleState[] moduleStates = DrivetrainConstants.driveKinematics.toSwerveModuleStates(
    chassisSpeeds);
    
    swerveSubsystem.setModuleStates(moduleStates);
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopAllModules();
  }
  
  // Returns true when the command should end. (In this case never, because it's the drive train).
  // wow a correct interpretation
  @Override
  public boolean isFinished() {
    return false;
  }
}
