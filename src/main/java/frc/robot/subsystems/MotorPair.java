// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorPair {
    private WPI_TalonFX motor;
    private CANCoder coder;
    private int number;
    boolean isInverted;
    /**
     * Creates a new MotorPair object
     * @param motorID ID of the Talon FX
     * @param coderID ID of the cancoder
     * @param motorReversed is the motor reversed
     * @param num the number of the module
     */
    public MotorPair(int motorID, int coderID, boolean motorReversed, int num){
        motor = new WPI_TalonFX(motorID);
        motor.setInverted(motorReversed);
        motor.config_kP(0, 0.2);
        motor.config_kI(0, 0);
        motor.config_kD(0, 0);
        motor.config_kF(0, 0);

        coder = new CANCoder(coderID);

        isInverted = motorReversed;

        number = num;
    }
/**
 * Turns the module to the specified degree
 * @param degree the degree to turn to
 */
    public void turnTo(double degree){
        // double currentDeg = (isInverted ? -coder.getPosition() : coder.getPosition());
        // double acceptable = 2.5;
        // if (Math.abs(currentDeg - degree) <= acceptable){
        //   motor.set(ControlMode.PercentOutput, 0);
        // }
        // else{
        //   if (currentDeg > degree){
        //     motor.set(ControlMode.PercentOutput, isInverted ? -0.1 : 0.1);
        //   }
        //   else{
        //     motor.set(ControlMode.PercentOutput, isInverted ? 0.1 : -0.1);
        //   }
        // }

        // deg      ticks
        // ---  X   -----    =   ticks to turn
        //  1       deg
        motor.set(ControlMode.MotionMagic, degree * 2048 / 360);
    }
/**
 * Gets the angle of the cancoder
 * @return the degree of the cancoder
 */
    public double getPosition(){
      return coder.getPosition() % 360;
    }
/**
 * Outputs debug info to SmartDashboard
 */
    public void debugToDashboard(){
      SmartDashboard.putNumber("CANCoder " + number + " angle", coder.getPosition());
    }

    /**
     * Sets the motor of the motorpair
     * @param mode ControlMode of the motor
     * @param x the value to set it to
     */
    public void set(ControlMode mode, double x){
      motor.set(mode, x);
    }
/**
 * Used to reset the cancoder
 * @param pos position (degrees) to set the cancoder to
 */
    public void setPos(double pos){
      coder.setPosition(pos);
    }
}
