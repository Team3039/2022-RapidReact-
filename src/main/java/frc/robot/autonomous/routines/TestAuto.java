// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.routines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.autonomous.TrajectoryGenerator;
import frc.robot.autonomous.commands.ResetOdometry;
import frc.robot.autonomous.commands.TrajectoryStop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public TestAuto() {
    TrajectoryGenerator trajectories = TrajectoryGenerator.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometry(),
      new WaitCommand(1),
      new RamseteCommand(
          trajectories.testTrajectory, 
          RobotContainer.drive::getPose,
          new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
          new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          RobotContainer.drive::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
          RobotContainer.drive::tankDriveVolts,
          RobotContainer.drive),
      new TrajectoryStop()
      // new RamseteCommand(
      //   trajectories.testTrajectoryReversed, 
      //   RobotContainer.drive::getPose,
      //   new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      //   new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
      //   Constants.kDriveKinematics,
      //   RobotContainer.drive::getWheelSpeeds,
      //   new PIDController(Constants.kPDriveVel, 0, 0),
      //   new PIDController(Constants.kPDriveVel, 0, 0),
      //   RobotContainer.drive::tankDriveVolts,
      //   RobotContainer.drive),
      // new TrajectoryStop()
    );
  }
}
