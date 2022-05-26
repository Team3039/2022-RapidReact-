// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ActuateClimb;
import frc.robot.commands.DisableClimbSoftLimits;
// import frc.robot.auto.commands.SetClimbSpeed;
import frc.robot.commands.FeedCargo;
// import frc.robot.commands.SetClimberActuateMode;
// import frc.robot.commands.SetHoodServoAngle;
import frc.robot.commands.SetIndexing;
import frc.robot.commands.SetLeftClimber;
import frc.robot.commands.SetManualTurretMode;
import frc.robot.commands.SetRightClimber;
import frc.robot.commands.SetSubsystemsClimbMode;
import frc.robot.commands.SetUnjamming;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.SpinShooterNoTrack;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TrackTarget;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretState;

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
  public static final Climber climber = new Climber();
  public static final LEDs LEDs = new LEDs();
  public static final Limelight limelight = new Limelight(drive);

  public static Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  /* Controllers */
  private static final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(1);
  private static final InterpolatedPS4Gamepad operatorPad = new InterpolatedPS4Gamepad(2);

  /* Driver Buttons */
  private final JoystickButton driverX = new JoystickButton(driverPad, PS4Gamepad.BUTTON_X);
  private final JoystickButton driverSquare = new JoystickButton(driverPad, PS4Gamepad.BUTTON_Square);
  private final JoystickButton driverTriangle = new JoystickButton(driverPad, PS4Gamepad.BUTTON_Triangle);
  private final JoystickButton driverCircle = new JoystickButton(driverPad, PS4Gamepad.BUTTON_Circle);

  private final JoystickButton driverDPadUp = new JoystickButton(driverPad, PS4Gamepad.DPAD_UP);
  private final JoystickButton driverDPadDown = new JoystickButton(driverPad, PS4Gamepad.DPAD_DOWN);
  private final JoystickButton driverDPadLeft = new JoystickButton(driverPad, PS4Gamepad.DPAD_LEFT);
  private final JoystickButton driverDPadRight = new JoystickButton(driverPad, PS4Gamepad.DPAD_RIGHT);

  private final JoystickButton driverL1 = new JoystickButton(driverPad, PS4Gamepad.BUTTON_L1);
  private final JoystickButton driverR1 = new JoystickButton(driverPad, PS4Gamepad.BUTTON_R1);

  private final JoystickButton driverL2 = new JoystickButton(driverPad, PS4Gamepad.BUTTON_L2);
  private final JoystickButton driverR2 = new JoystickButton(driverPad, PS4Gamepad.BUTTON_R2);

  private final JoystickButton driverPadButton = new JoystickButton(driverPad, PS4Gamepad.BUTTON_PAD);
  private final JoystickButton driverStart = new JoystickButton(driverPad, PS4Gamepad.BUTTON_START);

  private final JoystickButton driverShare = new JoystickButton(driverPad, PS4Gamepad.BUTTON_SHARE);
  private final JoystickButton driverOptions = new JoystickButton(driverPad, PS4Gamepad.BUTTON_OPTIONS);

  /* Operator Buttons */
  private final JoystickButton operatorX = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_X);
  private final JoystickButton operatorSquare = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_Square);
  private final JoystickButton operatorTriangle = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_Triangle);
  private final JoystickButton operatorCircle = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_Circle);

  private final JoystickButton operatorDPadUp = new JoystickButton(operatorPad, PS4Gamepad.DPAD_UP);
  private final JoystickButton operatorDPadDown = new JoystickButton(operatorPad, PS4Gamepad.DPAD_DOWN);
  private final JoystickButton operatorDPadLeft = new JoystickButton(operatorPad, PS4Gamepad.DPAD_LEFT);
  private final JoystickButton operatorDPadRight = new JoystickButton(operatorPad, PS4Gamepad.DPAD_RIGHT);

  private final JoystickButton operatorL1 = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_L1);
  private final JoystickButton operatorR1 = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_R1);

  private final JoystickButton operatorL2 = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_L2);
  private final JoystickButton operatorR2 = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_R2);
  private final JoystickButton operatorR3 = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_R3);

  private final JoystickButton operatorStart = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_START);
  private final JoystickButton operatorShare = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_SHARE);
  private final JoystickButton operatorOptions = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_OPTIONS);
  private final JoystickButton operatorPadButton = new JoystickButton(operatorPad, PS4Gamepad.BUTTON_PAD);

  public static double mHoodAngle = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    compressor.enableDigital();

    Drive.getInstance().setDefaultCommand(
        new TeleopSwerve(
            Drive.getInstance(),
            driverPad,
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
    driverOptions.whenPressed(new InstantCommand(() -> Drive.getInstance().setGyro(0)));

    // driverTriangle.whileHeld(new SetSnap(0));
    // driverSquare.whileHeld(new SetSnap(90));
    // driverCircle.whileHeld(new SetSnap(-90));
    // driverX.whileHeld(new SetSnap(180));

    driverShare.toggleWhenPressed(new TrackTarget());

    operatorL1.whileHeld(new SetIndexing());
    operatorR2.whileHeld(new FeedCargo());
    operatorL2.whileHeld(new SetUnjamming());

    operatorR1.whileHeld(new SpinShooter(2200, false));

    operatorTriangle.whenPressed(new SpinShooterNoTrack(2250));

    // operatorTriangle.whileHeld(new SetHoodServoAngle(90));
    // operatorCircle.whileHeld(new SetHoodServoAngle(135));

    operatorR3.toggleWhenPressed(new SetManualTurretMode());

    driverStart.whenPressed(new InstantCommand(
        () -> climber.leftClimber.set(ControlMode.Position, Constants.Climber.TELESCOPING_TO_MID_BAR_VALUE_LEFT)));
    driverStart.whenPressed(new InstantCommand(
        () -> climber.rightClimber.set(ControlMode.Position, Constants.Climber.TELESCOPING_TO_MID_BAR_VALUE_RIGHT)));
    driverStart.whenReleased(new InstantCommand(() -> climber.leftClimber.set(ControlMode.PercentOutput, 0)));
    driverStart.whenReleased(new InstantCommand(() -> climber.rightClimber.set(ControlMode.PercentOutput, 0)));

    // driverTriangle.whenPressed(new InstantCommand(() -> turret.setTurretPosition(0)));
    // driverX.whenPressed(new InstantCommand(() -> turret.setTurretPosition(-180)));

    driverL1.whileHeld(new SetLeftClimber(.90));
    driverL2.whileHeld(new SetLeftClimber(-.90));

    driverR1.whileHeld(new SetRightClimber(.90));
    driverR2.whileHeld(new SetRightClimber(-.90));

    driverPadButton.whenPressed(new InstantCommand(() -> RobotContainer.drive.isHighGear = false));
    driverPadButton.whenReleased(new InstantCommand(() -> RobotContainer.drive.isHighGear = true));

    // driverTriangle.toggleWhenPressed(new ActuateClimb());

    operatorX.whenPressed(new InstantCommand(() -> {
      shooter.setHoodAngle(0);
    }));

    operatorCircle.whenPressed(new InstantCommand(() -> shooter.setHoodAngle(0)));

    driverPadButton.toggleWhenPressed(new DisableClimbSoftLimits());
  }
  
  public static InterpolatedPS4Gamepad getDriver() {
    return driverPad;
  }

  public static InterpolatedPS4Gamepad getOperator() {
    return operatorPad;
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