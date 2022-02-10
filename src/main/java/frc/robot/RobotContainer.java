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
import frc.robot.commands.SetSnap;
import frc.robot.commands.SetHopperModeIndex;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final InterpolatedPS4Gamepad driver = new InterpolatedPS4Gamepad(0);

  /* Drive Controls */
  private final int translationAxis = (int) driver.interpolatedLeftYAxis();
  private final int strafeAxis = (int) driver.interpolatedLeftXAxis();
  private final int rotationAxis = (int) driver.interpolatedRightXAxis();

  /* Driver Buttons */
  private final JoystickButton driverX = new JoystickButton(driver, PS4Gamepad.BUTTON_X);
  private final JoystickButton driverCircle = new JoystickButton(driver, PS4Gamepad.BUTTON_Circle);
  private final JoystickButton driverSquare = new JoystickButton(driver, PS4Gamepad.BUTTON_Square);
  private final JoystickButton driverTriangle = new JoystickButton(driver, PS4Gamepad.BUTTON_Triangle);
  

  

  /* Subsystems */
  public static final Swerve s_Swerve = new Swerve();
  public static final Intake intake = new Intake();
  public static final Hopper hopper = new Hopper();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
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
    /* Driver Buttons */
    driverX.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    driverSquare.whileHeld(new SetSnap(0));
    driverCircle.toggleWhenPressed(new SetHopperModeIndex());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new RightFarFourBallAuto(s_Swerve);
  }
}
