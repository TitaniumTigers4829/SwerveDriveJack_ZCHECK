// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;;

public class SwerveModule extends SubsystemBase {

  private final WPI_TalonFX driveMotor;
  private final WPI_TalonSRX turnMotor;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, boolean turnMotorReversed) {

    // Making the motor objects
    driveMotor = new WPI_TalonFX(driveMotorID);
    turnMotor = new WPI_TalonSRX(turnMotorID);

    // Reversing them if necessary
    driveMotor.setInverted(driveMotorReversed);
    turnMotor.setInverted(turnMotorReversed);

    // Setting up the encoders
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    turnMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

    // Setting up PID for turnMoter
    turnMotor.config_kP(0, ModuleConstants.kP);
    turnMotor.config_kI(0, ModuleConstants.kI);
    turnMotor.config_kD(0, ModuleConstants.kD);

    // TODO: Setup Absolute Encoder stuff to straighten wheels when robot boots up

  }

  /**
   * Gets the angle in Radians of the current turn motor
   */
  public double getTurnAngle() {
    return (turnMotor.getSelectedSensorPosition() % 1024) * ModuleConstants.ticksToRadians;
  }

  /**
   * Gets the velocity in Radians per second for the current turn motor
   */
  public double getTurnVelocity() {
    return (turnMotor.getSelectedSensorVelocity()) * ModuleConstants.ticksToRadians;
  }

  // TODO: Check if this works
  /**
   * Gets the velocity in Meters per second of the current drive motor
   */
  public double getDriveVelocity() {
    // First gets the motor shaft units per 100ms, then multiplies by 10 to get it in units per second,
    // next it divides it by ticks and motor shaft units to get the rps,
    // next it multiplies that by the circumference of the wheel to get inches per second
    // finally it converts it to meters per second
    return ((((turnMotor.getSelectedSensorVelocity()) * 10) / (2048 * 7.13) ) * 4 * Math.PI * 0.0254);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnAngle()));
  }

  public void stopModule() {
    driveMotor.set(0);
    turnMotor.set(0);
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.005) {
      stopModule();
      return
    }
    // Optimizes angle so the wheel won't ever have to move more than 90 degrees
    state = SwerveModuleState.optimize(state, getState().angle);
    // Sets the drive motor's speed from 0.0 to 1.0
    driveMotor.set(state.speedMetersPerSecond / ModuleConstants.physicalMaxSpeedMetersPerSecond);
    // TODO: ask how to get the built in PID working, also for trying to fix maybe using PID Controller class is a good idea
    turnMotor.set();
    SmartDashboard.putString("Module " + driveMotorID.toString() + ", State: " + state.toString());
  }

  @Override
  public void periodic() {

    // TODO: Ask for examples of what goes in here

    // This method will be called once per scheduler run
  }
}
