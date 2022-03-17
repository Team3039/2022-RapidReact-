// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.commands.SetIndexingFeedMode;
import frc.robot.auto.commands.SetIndexingIdleMode;
import frc.robot.auto.commands.SetIndexingIntakeMode;
import frc.robot.auto.commands.SetShooterIdleMode;
import frc.robot.auto.commands.SetShooterSpinUpMode;
import frc.robot.auto.commands.SetTurretDriveMode;
import frc.robot.auto.commands.SetTurretTrackMode;
import frc.robot.auto.commands.StopTrajectory;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GenericTwoBallAuto extends SequentialCommandGroup {
    
    /** Creates a new GenericTwoBallAuto. */
    public GenericTwoBallAuto(Drive s_Swerve) {

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.KP_THETA_CONTROLLER, 0, 0,
                Constants.AutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand grabBallCommand = new SwerveControllerCommand(
                frc.robot.auto.TrajectoryGenerator.getGenericStartToFirstBall(),
                s_Swerve::getPose,
                Constants.Swerve.SWERVE_KINEMATICS,
                new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
                thetaController,
                Drive.getSwerveHeadingSupplier(0),
                s_Swerve::setModuleStates,
                s_Swerve);



        addCommands(
                new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())),
                new SetTurretDriveMode(),
                new SetShooterSpinUpMode(4800),
                new SetIndexingIntakeMode(),
                grabBallCommand,
                new StopTrajectory(),
                new SetTurretTrackMode(),
                new SetIndexingFeedMode(),
                new WaitCommand(1),
                new SetTurretDriveMode(),
                new SetShooterIdleMode(),
                new SetIndexingIdleMode()
        );
    }
}