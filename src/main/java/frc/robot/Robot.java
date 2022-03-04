// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
// import frc.robot.auto.routines.RightFarFourBallAuto;
// import frc.robot.auto.routines.RightNearFourBallAuto;
import frc.robot.subsystems.Drive;
<<<<<<< HEAD
=======
import frc.robot.subsystems.Turret.TurretMode;
>>>>>>> parent of 4fcc90b (-Minor Edits)

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

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  SendableChooser<Command> autonTaskChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.s
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();

    autonTaskChooser = new SendableChooser<>();

    autonTaskChooser.setDefaultOption("Do Nothing", new PrintCommand("Do Nothing"));

    // autonTaskChooser.addOption("Right Far Four Ball", new RightFarFourBallAuto(Drive.getInstance()));
    // autonTaskChooser.addOption("Right Near Four Ball", new RightNearFourBallAuto(Drive.getInstance()));

    SmartDashboard.putData("Autonomous", autonTaskChooser);
<<<<<<< HEAD
=======
    // field.setRobotPose(Swerve.getInstance().getPose());
    // SmartDashboard.putData("Field", field);
    
    RobotContainer.turret.setCamMode(false);
    RobotContainer.turret.setTurretMode(TurretMode.DRIVE);
>>>>>>> parent of 4fcc90b (-Minor Edits)
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
    CommandScheduler.getInstance().run();

    if(DriverStation.getAlliance().equals(Alliance.Red)) {
      isRedAlliance = true;
    }
    else if(DriverStation.getAlliance().equals(Alliance.Blue)) {
      isRedAlliance = false;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
<<<<<<< HEAD
=======
    RobotContainer.turret.turretAngle = 0;
    RobotContainer.turret.setTurretMode(TurretMode.DRIVE);

>>>>>>> parent of 4fcc90b (-Minor Edits)
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autonTaskChooser.getSelected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // addPeriodic(
      // () -> Drive.getInstance().swerveOdometry.update(
          // Drive.getInstance().getYaw(),
          // Drive.getInstance().getStates()),
      // 0.01);
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
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