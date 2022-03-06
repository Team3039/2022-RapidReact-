// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.spec.RSAKeyGenParameterSpec;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FeedCargo;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetGear;
import frc.robot.commands.SetIndexing;
import frc.robot.commands.SetShooterPercent;
import frc.robot.commands.SetSnap;
import frc.robot.commands.TeleopSwerve;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;



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
  public static final Indexer indexer = new Indexer();
  public static final Drive drive = new Drive();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static final Turret turret = new Turret();
  
  /* Controllers */
  private final InterpolatedPS4Gamepad driver = new InterpolatedPS4Gamepad(1);

  /* Driver Buttons */
  private final JoystickButton driverX = new JoystickButton(driver, PS4Gamepad.BUTTON_X);
  private final JoystickButton driverSquare = new JoystickButton(driver, PS4Gamepad.BUTTON_Square);
  private final JoystickButton driverTriangle = new JoystickButton(driver, PS4Gamepad.BUTTON_Triangle);
  
  @SuppressWarnings("unused") 
  private final JoystickButton driverCircle = new JoystickButton(driver, PS4Gamepad.BUTTON_Circle);

  private final JoystickButton driverDPadUp = new JoystickButton(driver, PS4Gamepad.DPAD_UP);
  private final JoystickButton driverDPadDown = new JoystickButton(driver, PS4Gamepad.DPAD_DOWN);
  private final JoystickButton driverDPadLeft = new JoystickButton(driver, PS4Gamepad.DPAD_LEFT);
  private final JoystickButton driverDPadRight = new JoystickButton(driver, PS4Gamepad.DPAD_RIGHT);
   
  private final JoystickButton driverR1 = new JoystickButton(driver, PS4Gamepad.BUTTON_R1);

  @SuppressWarnings("unused") 
  private final JoystickButton driverOptions = new JoystickButton(driver, PS4Gamepad.BUTTON_OPTIONS);
  private final JoystickButton driverShare = new JoystickButton(driver, PS4Gamepad.BUTTON_SHARE);
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
    driverTriangle.whenPressed(new InstantCommand(() -> Drive.getInstance().zeroGyro()));
    
    driverDPadUp.whileHeld(new SetSnap(0));
    driverDPadDown.whileHeld(new SetSnap(180));
    driverDPadLeft.whileHeld(new SetSnap(90));
    driverDPadRight.whileHeld(new SetSnap(270));

   // driverR1.whileHeld(new SetGear(true));
  //  driverR1.whenReleased(new SetGear(false));

    // driverSquare.whileHeld(new SetIndexing());
   driverX.toggleWhenPressed(new RunIntake());
    // driverShare.whileHeld(new SetUnjamming());

    driverCircle.toggleWhenPressed(new SetShooterPercent(.8));

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