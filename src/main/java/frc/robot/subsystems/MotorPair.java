// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class MotorPair {
    private WPI_TalonFX motor;
    private CANCoder coder;
    private int number;
    public MotorPair(int motorID, int coderID, boolean motorReversed, int num){
        this.motor = new WPI_TalonFX(motorID);
        this.motor.setInverted(motorReversed);
        this.coder = new CANCoder(coderID);
        this.number = num;
    }

    public void turnTo(double degree){
        double currentDeg = coder.getPosition();
        double acceptable = 2.5;
        if (Math.abs(currentDeg - degree) <= acceptable){
          motor.set(ControlMode.PercentOutput, 0);
        }
        else{
          if (currentDeg > degree){
            motor.set(ControlMode.PercentOutput, 0.1);
          }
          else{
            motor.set(ControlMode.PercentOutput, -0.1);
          }
        }
    }

    public double getPosition(){
      return coder.getPosition();
    }

    public void debugToDashboard(){
      SmartDashboard.putNumber("Module " + number + " angle", coder.getPosition());
    }

    public void set(ControlMode mode, double x){
      motor.set(mode, x);
    }

    public void setPos(double pos){
      coder.setPosition(pos);
    }
}
