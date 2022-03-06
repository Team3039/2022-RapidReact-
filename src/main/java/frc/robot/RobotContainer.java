// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SetIndexing;
import frc.robot.commands.SetGear;
import frc.robot.commands.FeedCargo;
import frc.robot.commands.SetSnap;
import frc.robot.commands.SetUnjamming;
import frc.robot.commands.TeleopSwerve;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.subsystems.*;


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
  public static final Drive mDrive = new Drive();
  public static final Intake mIntake = new Intake();
  public static final Indexer mIndexer = new Indexer();
  public static final Shooter mShooter = new Shooter();
  public static final Turret mTurret = new Turret();
  public static final LEDs mLEDs = new LEDs();
  public static final Limelight mLimelight = new Limelight(mDrive);
  
  /* Controllers */
  private final InterpolatedPS4Gamepad mDriver = new InterpolatedPS4Gamepad(0);
  private final InterpolatedPS4Gamepad mOperator = new InterpolatedPS4Gamepad(1);

  /* Driver Buttons */
  private final JoystickButton driverX = new JoystickButton(mDriver, PS4Gamepad.BUTTON_X);
  private final JoystickButton driverSquare = new JoystickButton(mDriver, PS4Gamepad.BUTTON_Square);
  private final JoystickButton driverTriangle = new JoystickButton(mDriver, PS4Gamepad.BUTTON_Triangle);

  private final JoystickButton driverDPadUp = new JoystickButton(mDriver, PS4Gamepad.DPAD_UP);
  private final JoystickButton driverDPadDown = new JoystickButton(mDriver, PS4Gamepad.DPAD_DOWN);
  private final JoystickButton driverDPadLeft = new JoystickButton(mDriver, PS4Gamepad.DPAD_LEFT);
  private final JoystickButton driverDPadRight = new JoystickButton(mDriver, PS4Gamepad.DPAD_RIGHT);
   
  private final JoystickButton driverR1 = new JoystickButton(mDriver, PS4Gamepad.BUTTON_R1);
  private final JoystickButton driverShare = new JoystickButton(mDriver, PS4Gamepad.BUTTON_SHARE);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Drive.getInstance().setDefaultCommand(
        new TeleopSwerve(
          Drive.getInstance(),
                mDriver,
          true,
          true)
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
    driverTriangle.whenPressed(new InstantCommand(() -> Drive.getInstance().zeroGyro()));
    
    driverDPadUp.whenPressed(new SetSnap(0));

    driverR1.whileHeld(new SetGear(true));
    driverR1.whenReleased(new SetGear(false));

    driverSquare.whileHeld(new SetIndexing());
    driverX.whileHeld(new FeedCargo());
    driverShare.whileHeld(new SetUnjamming());
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