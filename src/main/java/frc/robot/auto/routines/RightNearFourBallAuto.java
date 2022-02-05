// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.commands.StopTrajectory;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightNearFourBallAuto extends SequentialCommandGroup {
  private TrajectoryConfig configLow;

  /** Creates a new ThreeBallMidWallAuto. */
  public RightNearFourBallAuto(Swerve s_Swerve) {

    var thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand fourBallCommandOne = new SwerveControllerCommand(
        frc.robot.auto.TrajectoryGenerator.getRightNearStartToFirstBall(),
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        Swerve.getSwerveHeadingSupplier(0),
        s_Swerve::setModuleStates,
        s_Swerve);

    SwerveControllerCommand fourBallCommandTwo = new SwerveControllerCommand(
        frc.robot.auto.TrajectoryGenerator.getRightNearFirstBallToSecondBall(),
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        Swerve.getSwerveHeadingSupplier(60),
        s_Swerve::setModuleStates,
        s_Swerve);

    SwerveControllerCommand fourBallCommandThree = new SwerveControllerCommand(
        frc.robot.auto.TrajectoryGenerator.getRightNearSecondBallToThirdBall(),
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        Swerve.getSwerveHeadingSupplier(-20),
        s_Swerve::setModuleStates,
        s_Swerve);

    SwerveControllerCommand fourBallCommandFour = new SwerveControllerCommand(
        frc.robot.auto.TrajectoryGenerator.getRightNearThirdBallToFinish(),
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        Swerve.getSwerveHeadingSupplier(180),
        s_Swerve::setModuleStates,
        s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())),
        fourBallCommandOne,
        new StopTrajectory(),
        fourBallCommandTwo,
        new StopTrajectory(),
        new WaitCommand(1),
        fourBallCommandThree,
        new StopTrajectory(),
        fourBallCommandFour,
        new StopTrajectory()
        );
  }
}