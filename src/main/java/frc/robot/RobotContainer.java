// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FeedCargo;
import frc.robot.commands.SetIndexing;
import frc.robot.commands.SetTurretManualMode;
import frc.robot.commands.SetUnjamming;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TrackTarget;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
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
  public static final Intake intake = new Intake();
  public static final Indexer indexer = new Indexer();
  public static final Drive drive = new Drive();
  public static final Shooter shooter = new Shooter();
  public static final Turret turret = new Turret();
  // public static final Climber mClimber = new Climber();
  public static final LEDs mLEDs = new LEDs();
  public static final Limelight limelight = new Limelight(drive);

  /* Controllers */
  private static final InterpolatedPS4Gamepad driver = new InterpolatedPS4Gamepad(1);
  private static final InterpolatedPS4Gamepad operator = new InterpolatedPS4Gamepad(2);

  /* Driver Buttons */
  private final JoystickButton driverX = new JoystickButton(driver, PS4Gamepad.BUTTON_X);
  private final JoystickButton driverSquare = new JoystickButton(driver, PS4Gamepad.BUTTON_Square);
  private final JoystickButton driverTriangle = new JoystickButton(driver, PS4Gamepad.BUTTON_Triangle);
  private final JoystickButton driverCircle = new JoystickButton(driver, PS4Gamepad.BUTTON_Circle);

  private final JoystickButton driverDPadUp = new JoystickButton(driver, PS4Gamepad.DPAD_UP);
  private final JoystickButton driverDPadDown = new JoystickButton(driver, PS4Gamepad.DPAD_DOWN);
  private final JoystickButton driverDPadLeft = new JoystickButton(driver, PS4Gamepad.DPAD_LEFT);
  private final JoystickButton driverDPadRight = new JoystickButton(driver, PS4Gamepad.DPAD_RIGHT);

  private final JoystickButton driverL1 = new JoystickButton(driver, PS4Gamepad.BUTTON_L1);
  private final JoystickButton driverR1 = new JoystickButton(driver, PS4Gamepad.BUTTON_R1);

  private final JoystickButton driverL2 = new JoystickButton(driver, PS4Gamepad.BUTTON_L2);
  private final JoystickButton driverR2 = new JoystickButton(driver, PS4Gamepad.BUTTON_R2);

  private final JoystickButton driverPadButton = new JoystickButton(driver, PS4Gamepad.BUTTON_PAD);
  private final JoystickButton driverStart = new JoystickButton(driver, PS4Gamepad.BUTTON_START);

  /* Operator Buttons */
  private final JoystickButton operatorX = new JoystickButton(operator, PS4Gamepad.BUTTON_X);
  private final JoystickButton operatorSquare = new JoystickButton(operator, PS4Gamepad.BUTTON_Square);
  private final JoystickButton operatorTriangle = new JoystickButton(operator, PS4Gamepad.BUTTON_Triangle);
  private final JoystickButton operatorCircle = new JoystickButton(operator, PS4Gamepad.BUTTON_Circle);

  private final JoystickButton operatorDPadUp = new JoystickButton(operator, PS4Gamepad.DPAD_UP);
  private final JoystickButton operatorDPadDown = new JoystickButton(operator, PS4Gamepad.DPAD_DOWN);
  private final JoystickButton operatorDPadLeft = new JoystickButton(operator, PS4Gamepad.DPAD_LEFT);
  private final JoystickButton operatorDPadRight = new JoystickButton(operator, PS4Gamepad.DPAD_RIGHT);

  private final JoystickButton operatorL1 = new JoystickButton(operator, PS4Gamepad.BUTTON_L1);
  private final JoystickButton operatorR1 = new JoystickButton(operator, PS4Gamepad.BUTTON_R1);

  private final JoystickButton operatorL2 = new JoystickButton(operator, PS4Gamepad.BUTTON_L2);
  private final JoystickButton operatorR2 = new JoystickButton(operator, PS4Gamepad.BUTTON_R2);
  private final JoystickButton operatorR3 = new JoystickButton(operator, PS4Gamepad.BUTTON_R3);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Drive.getInstance().setDefaultCommand(
        new TeleopSwerve(
            Drive.getInstance(),
            driver,
            true,
            true));

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

    // driverDPadUp.whileHeld(new SetSnap(0));
    // driverDPadRight.whileHeld(new SetSnap(90));
    // driverDPadLeft.whileHeld(new SetSnap(-90));

    // driverR1.whileHeld(new SetGear(true));
    // driverR1.whenReleased(new SetGear(false));
    driverR1.toggleWhenPressed(new TrackTarget());

    operatorL1.whileHeld(new SetIndexing());
    operatorR2.whileHeld(new FeedCargo());
    operatorL2.whileHeld(new SetUnjamming());
    operatorR1.whileHeld(new SpinShooter());
    operatorR3.toggleWhenPressed(new SetTurretManualMode());
    // operatorSquare.toggleWhenPressed(new SetClimberActuateMode());
    // operatorStart.whenPressed(new SetSubsystemsClimbMode());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public static InterpolatedPS4Gamepad getDriver() {
    return driver;
  }

  public static InterpolatedPS4Gamepad getOperator() {
    return operator;
  }

  public Command getAutonomousCommand() {
    return null;
  }
}