// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.SetTurretPos;
import frc.robot.commands.trackTarget;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drive drive = new Drive();
  public static Shooter shooter = new Shooter();
  public static Turret turret = new Turret();
  
  public static PS4Gamepad driverPad = new PS4Gamepad(RobotMap.DRIVER_JOYSTICK_1_USB_ID);
 
  Button driverTriangle = driverPad.getButtonTriangle();
  Button driverSquare = driverPad.getButtonSquare();
  Button driverCircle = driverPad.getButtonCircle();
  Button driverX = driverPad.getButtonX();
  Button driverShare = driverPad.getShareButton();
  Button driverOptions = driverPad.getOptionsButton();
  Button driverPadButton = driverPad.getButtonPad();
  Button driverL1 = driverPad.getL1();
  Button driverL2 = driverPad.getL2();
  Button driverL3 = driverPad.getL3();
  Button driverR1 = driverPad.getR1();
  Button driverR3 = driverPad.getR3();
  Button startButton = driverPad.getStartButton();
  Button driverDPadUp = driverPad.getDPadUp();
  Button driverDPadDown = driverPad.getDPadDown();
  Button driverDPadLeft = driverPad.getDPadLeft();
  Button driverDPadRight = driverPad.getDPadRight();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
   
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverDPadDown.whileHeld(new trackTarget());
    //turns turret forward
    driverDPadUp.whileHeld(new SetTurretPos(0));
    //turns turret to the left
    driverDPadLeft.whileHeld(new SetTurretPos(90));
    //turns turret to the right
    driverDPadRight.whileHeld(new SetTurretPos(-90));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  //Get Controller Objects
  public static PS4Gamepad getDriver() {
    return driverPad;
  }
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
}
}
