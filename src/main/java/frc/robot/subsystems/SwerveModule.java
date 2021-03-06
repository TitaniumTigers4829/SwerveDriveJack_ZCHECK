// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final MotorPair pair;
  private final WPI_TalonFX driveMotor;
  private final int moduleNumber;

  /**
   * Creates a new Swerve Module
   * @param driveMotorID ID of the drive motor
   * @param turnMotorID ID of the turn
   * @param canCoderID ID of the cancoder
   * @param driveMotorReversed is the drive motor reversed
   * @param turnMotorReversed is the turn motor reversed
   * @param moduleNumber number of the module
   */
  public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, boolean driveMotorReversed, boolean turnMotorReversed, int moduleNumber) {

    this.moduleNumber = moduleNumber;

    pair = new MotorPair(turnMotorID, canCoderID, turnMotorReversed, moduleNumber);

    driveMotor = new WPI_TalonFX(driveMotorID);
    driveMotor.setInverted(driveMotorReversed);

    resetEncoders();
  }

  /**
   * Gets the angle in Radians of the current turn motor
   */
  public double getTurnAngle() {
    return pair.getPosition();
  }

  // /**
  //  * Gets the velocity in Radians per second for the current turn motor
  //  */
  // public double getTurnVelocity() {
  //   return (turnMotor.getSelectedSensorVelocity()) * ModuleConstants.ticksToRadians;
  // }

  /**
   * Gets the velocity in Meters per second of the current drive motor
   */
  public double getDriveVelocity() {
    // First gets the motor shaft units per 100ms, then multiplies by 10 to get it in units per second,
    // next it divides it by ticks and motor shaft units to get the rps,
    // next it multiplies that by the circumference of the wheel to get inches per second
    // finally it converts it to meters per second
    return ((((driveMotor.getSelectedSensorVelocity()) * 10) / (2048 * 7.13) ) * 4 * Math.PI * 0.0254);
  }
/**
 * getState() returns the current state of the module
 * @return the state of the module
 */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnAngle()));
  }
/**
 * Completely stops the module
 */
  public void stopModule() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    pair.set(ControlMode.PercentOutput, 0);
  }
/**
 * Resets the encoders
 */
  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    pair.setPos(0);
}
/**
 * Set the desired state of the module
 * @param state desired state
 */
  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.005) {
      stopModule();
      return;
    }
    // Optimizes angle so the wheel won't ever have to move more than 90 degrees
    state = SwerveModuleState.optimize(state, new Rotation2d((pair.getPosition() * (Math.PI / 180))));
    // Sets the drive motor's speed from 0.0 to 1.0
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / ModuleConstants.physicalMaxSpeedMetersPerSecond);
    pair.turnTo(state.angle.getDegrees());
    SmartDashboard.putNumber("Module " + moduleNumber + " angle", state.angle.getDegrees());
  }
/**
 * hehe nErDs go BRRRBRRRRR
 */
  @Override
  public void periodic() {
    pair.debugToDashboard();
  }
}
