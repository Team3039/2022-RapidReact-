// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

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
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Figure8Auto extends SequentialCommandGroup {
  private TrajectoryConfig configLow;


  /** Creates a new Figure8Auto. */
  public Figure8Auto(Swerve s_Swerve) {
  
   TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics);
    
                    TrajectoryConfig configLow =
                    new TrajectoryConfig(
                            Constants.AutoConstants.kMaxSpeedMetersPerSecond - 2,
                            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared - 2)
                        .setKinematics(Constants.Swerve.swerveKinematics);
    
   

           Trajectory figureEightTrajectory =
             TrajectoryGenerator.generateTrajectory(
               List.of( 
                  new Pose2d(),
                //   new Pose2d(new Translation2d(Units.inchesToMeters(28.2), Units.inchesToMeters(28.2)), Rotation2d.fromDegrees(0)),
                  new Pose2d(new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(40)), Rotation2d.fromDegrees(0)),
                //   new Pose2d(new Translation2d(Units.inchesToMeters(68.2), Units.inchesToMeters(28.2)), Rotation2d.fromDegrees(0)),
                  new Pose2d(new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(0)), Rotation2d.fromDegrees(0))
               ),
               configLow
             );

           Trajectory figureEightTrajectory2 =
             TrajectoryGenerator.generateTrajectory(
                 List.of(
                    new Pose2d(new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(0)), Rotation2d.fromDegrees(0)),
                    // new Pose2d(new Translation2d(Units.inchesToMeters(68.2), Units.inchesToMeters(-28.2)), Rotation2d.fromDegrees(0)),
                    new Pose2d(new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(-40)), Rotation2d.fromDegrees(0)),
                    // new Pose2d(new Translation2d(Units.inchesToMeters(28.2), Units.inchesToMeters(-28.2)), Rotation2d.fromDegrees(0)),
                    new Pose2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)), Rotation2d.fromDegrees(0))
               ),
               configLow
             );
               
             var thetaController =
             new ProfiledPIDController(
                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
         thetaController.enableContinuousInput(-Math.PI, Math.PI);
 
         SwerveControllerCommand figureEightOneCommand =
             new SwerveControllerCommand(
                 figureEightTrajectory,
                 s_Swerve::getPose,
                 Constants.Swerve.swerveKinematics,
                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                 thetaController,
                 s_Swerve::getPI,
                 s_Swerve::setModuleStates,
                 s_Swerve
             );

             SwerveControllerCommand figureEightTwoCommand =
             new SwerveControllerCommand(
                 figureEightTrajectory2,
                 s_Swerve::getPose,
                 Constants.Swerve.swerveKinematics,
                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                 thetaController,
                 s_Swerve::getPI,
                 s_Swerve::setModuleStates,
                 s_Swerve
             );
 
 
         addCommands(
             new InstantCommand(() -> s_Swerve.resetOdometry(figureEightTrajectory.getInitialPose())),
             figureEightOneCommand,
             figureEightTwoCommand
         );
     }
}