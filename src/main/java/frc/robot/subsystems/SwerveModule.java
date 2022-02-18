// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final WPI_TalonFX driveMotor;
  private final WPI_TalonSRX turnMotor;
  private final int moduleNumber;

  private final CANCoder turnEncoder;


  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, boolean driveMotorReversed, boolean turnMotorReversed, int moduleNumber) {

    this.moduleNumber = moduleNumber;

    // Making the motor objects
    driveMotor = new WPI_TalonFX(driveMotorID);
    turnMotor = new WPI_TalonSRX(turnMotorID);

    // Reversing them if necessary
    driveMotor.setInverted(driveMotorReversed);
    turnMotor.setInverted(turnMotorReversed);

    turnMotor.config_kP(0, 1.0);

    // Setting up the encoders
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.turnEncoder = new CANCoder(canCoderID);

    resetEncoders();
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
    driveMotor.set(ControlMode.PercentOutput, 0);
    turnMotor.set(ControlMode.PercentOutput, 0);
  }

  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    turnEncoder.setPosition(0);
}

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.005) {
      stopModule();
      return;
    }
    // Optimizes angle so the wheel won't ever have to move more than 90 degrees
    state = SwerveModuleState.optimize(state, getState().angle);
    // Sets the drive motor's speed from 0.0 to 1.0
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / ModuleConstants.physicalMaxSpeedMetersPerSecond);
    // turnEncoder = cancoder in degrees
    // state.angle.getDegrees is also in degrees
    // talon fx has 2048 ticks/rotation
    // if mult by cancoder deg by (ticksPerRot/degPerRot) -> 
    // cancoder DEG     ticksPerRot     ticks
    // ------------  x  ----------- --> ------
    //       1          degPerRot          1
    // ticksPerRot = 2048
    // degPerRot = 360
    int ticksPerDeg = 2048;
    int ratio = (ticksPerDeg / 360);
    turnMotor.set(ControlMode.MotionMagic, state.angle.getDegrees() * ratio);
    SmartDashboard.putNumber("Module " + moduleNumber + " angle", state.angle.getDegrees());
  }

  @Override
  public void periodic() {

    // TODO: Ask for examples of what goes in here
    // Z--- This is where we put debug commands like encoder values
    // Prateek says he doesn't think so.

    SmartDashboard.putNumber("Module " + moduleNumber + " cancoder", turnEncoder.getPosition());
    SmartDashboard.putNumber("Module " + moduleNumber + " cancoder", turnMotor.getSelectedSensorPosition());
  }
}
