/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoyStickConstants;
import frc.robot.commands.SwerveJoystickDrive;
import frc.robot.subsystems.SwerveSubsystem;
/**
 * meme
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  //SendableChooser<Command> chooser = new SendableChooser<>();
  
  private final SwerveSubsystem swerveSubsystem;
  
  private final Joystick driverJoystick = new Joystick(JoyStickConstants.joyStickPort);
  
  /**
  * Container for the robot's everything
  */
  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    swerveSubsystem.setDefaultCommand(new SwerveJoystickDrive(
    swerveSubsystem, 
    () -> -driverJoystick.getRawAxis(1),
    () -> driverJoystick.getRawAxis(0),
    () -> driverJoystick.getRawAxis(2)));
    configureButtonBindings();
  }

  /**
   * put in setdefaultcommand
    // Add ! to make field relative the default
    () -> driverJoystick.getRawButton(2)
   */
  /**
  * Use this method to define your button->command mappings. Buttons can be
  * created by instantiating a {@link GenericHID} or one of its subclasses
  * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
  * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
  */
  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, 1).whenPressed(swerveSubsystem::zeroGyro);
  }
  
  /**
  * Use this to pass the autonomous command to the main {@link Robot} class.
  *
  * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    return null;
  }
}
