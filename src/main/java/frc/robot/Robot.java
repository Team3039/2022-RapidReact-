// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.math.FieldOrientedTurretHelper;
import frc.lib.math.FieldOrientedTurretHelper.Start_Pose;
import frc.robot.auto.routines.DriveStraight;
import frc.robot.auto.routines.RightFarFourBallAuto;
import frc.robot.auto.routines.RightNearFiveBallAuto;
import frc.robot.auto.routines.RightNearFourBallAuto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Turret.TurretState;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Field2d field;

  public static Timer mTimer = new Timer();

  public static boolean isRedAlliance;

  public static CTREConfigs ctreConfigs;
  public static Trajectory mTrajectory = new Trajectory();

  public static FieldOrientedTurretHelper mFieldOrientedTurretHelper;

  private Command autonomousCommand;

  @SuppressWarnings("unused")
  private RobotContainer robotContainer;

  SendableChooser<Command> autonTaskChooser = new SendableChooser<>();
  SendableChooser<FieldOrientedTurretHelper.Start_Pose> mStartPoseChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.s
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    robotContainer = new RobotContainer();

    autonTaskChooser = new SendableChooser<>();

    autonTaskChooser.setDefaultOption("Do Nothing", new PrintCommand("Do Nothing"));

    autonTaskChooser.addOption("Right Far Four Ball", new RightFarFourBallAuto(Drive.getInstance()));
    autonTaskChooser.addOption("Right Near Four Ball", new RightNearFourBallAuto(Drive.getInstance()));
    autonTaskChooser.addOption("Right Near Five Ball", new RightNearFiveBallAuto(Drive.getInstance()));
    
    autonTaskChooser.addOption("Drive Straight", new DriveStraight(Drive.getInstance()));

    SmartDashboard.putData("Autonomous", autonTaskChooser);
    // field.setRobotPose(Swerve.getInstance().getPose());
    // SmartDashboard.putData("Field", field);

    mStartPoseChooser.addOption("Left Far Start Pose", Start_Pose.LEFT_FAR);
    mStartPoseChooser.addOption("Left Near Start Pose", Start_Pose.LEFT_NEAR);
    mStartPoseChooser.addOption("Right Far Start Pose", Start_Pose.RIGHT_FAR);
    mStartPoseChooser.addOption("Right Near Start Pose", Start_Pose.RIGHT_NEAR);

    mStartPoseChooser.setDefaultOption("Left Far Start Pose", Start_Pose.LEFT_FAR);

    SmartDashboard.putData("Start Pose Chooser", mStartPoseChooser);

    UsbCamera usbCamera = CameraServer.startAutomaticCapture();
    usbCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 180, 60);

    RobotContainer.turret.setState(TurretState.DRIVE);
    RobotContainer.shooter.setState(ShooterState.IDLE);
    RobotContainer.indexer.setState(IndexerState.IDLE);
    RobotContainer.intake.setState(IntakeState.IDLE);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    mFieldOrientedTurretHelper = new FieldOrientedTurretHelper(mStartPoseChooser.getSelected());

    CommandScheduler.getInstance().run();

    SmartDashboard.putBoolean("Second Stage Beam Break", RobotContainer.indexer.mSecondStageGate.get());
    SmartDashboard.putBoolean("First Stage Beam Break", RobotContainer.indexer.mFirstStageGate.get());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // RobotContainer.turret.setState(TurretState.DRIVE);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = autonTaskChooser.getSelected();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    RobotContainer.turret.setState(TurretState.DRIVE);
  }

  @Override
  public void autonomousPeriodic() {
    addPeriodic(
        () -> Drive.getInstance().swerveOdometry.update(
            Drive.getInstance().getYaw(),
            Drive.getInstance().getStates()),
        0.01);
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.turret.setState(TurretState.DRIVE);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}