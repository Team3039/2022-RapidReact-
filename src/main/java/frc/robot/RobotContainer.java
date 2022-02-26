// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.auto.routines.RightFarFourBallAuto;
import frc.robot.commands.SetGear;
import frc.robot.commands.SetSnap;
import frc.robot.commands.TeleopSwerve;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;



/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Subsystems */
  public static final Drive drive = new Drive();
  public static final Intake intake = new Intake();
  public static final Indexer indexer = new Indexer();
  public static final Shooter shooter = new Shooter();
  
  /* Controllers */
  private final InterpolatedPS4Gamepad driver = new InterpolatedPS4Gamepad(0);

  /* Driver Buttons */
  private final JoystickButton driverX = new JoystickButton(driver, PS4Gamepad.BUTTON_X);

  private final JoystickButton driverDPadUp = new JoystickButton(driver, PS4Gamepad.DPAD_UP);
  private final JoystickButton driverDPadDown = new JoystickButton(driver, PS4Gamepad.DPAD_DOWN);
  private final JoystickButton driverDPadLeft = new JoystickButton(driver, PS4Gamepad.DPAD_LEFT);
  private final JoystickButton driverDPadRight = new JoystickButton(driver, PS4Gamepad.DPAD_RIGHT);
   
  private final JoystickButton driverR1 = new JoystickButton(driver, PS4Gamepad.BUTTON_R1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;

    Drive.getInstance().setDefaultCommand(
        new TeleopSwerve(
          Drive.getInstance(),
          driver, 
          fieldRelative, 
          openLoop)
    );

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    driverX.whenPressed(new InstantCommand(() -> Drive.getInstance().zeroGyro()));
    
    driverDPadUp.whileHeld(new SetSnap(0));
    driverDPadDown.whileHeld(new SetSnap(180));
    driverDPadLeft.whileHeld(new SetSnap(90));
    driverDPadRight.whileHeld(new SetSnap(270));

    driverR1.whileHeld(new SetGear(true));
    driverR1.whenReleased(new SetGear(false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new RightFarFourBallAuto(Drive.getInstance());
  }
}