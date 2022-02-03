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
public class ThreeBallMidWallAuto extends SequentialCommandGroup {
  private TrajectoryConfig configLow;


  /** Creates a new ThreeBallMidWallAuto. */
  public ThreeBallMidWallAuto(Swerve s_Swerve) {
  
   TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics);
                //    .setReversed(true);
    
                    TrajectoryConfig configLow =
                    new TrajectoryConfig(
                            Constants.AutoConstants.kMaxSpeedMetersPerSecond - 2,
                            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared - 2)
                        .setKinematics(Constants.Swerve.swerveKinematics);
                     //   .setReversed(true);
                    
   
            //START AT 210 DEGREES AT THE STARTING POINT
           Trajectory testThreeBallTrajectoryBallOne =
             TrajectoryGenerator.generateTrajectory(
               List.of( 
                  new Pose2d(new Translation2d(Units.inchesToMeters(0), 0), new Rotation2d(0)),
                  new Pose2d(new Translation2d(Units.inchesToMeters(-54), 0), new Rotation2d(180))
               ),
               config
             );             
            
           Trajectory testThreeBallTrajectoryBallTwo =
            TrajectoryGenerator.generateTrajectory(
            List.of(
             new Pose2d(new Translation2d(Units.inchesToMeters(-54), 0), new Rotation2d(0)),
             new Pose2d(new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(-75)), new Rotation2d(0))
            ),
            config
          );   
           
          Trajectory testThreeBallTrajectoryBallThree =
          TrajectoryGenerator.generateTrajectory(
          List.of( 
            new Pose2d(new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(-75)), new Rotation2d(0)),
            new Pose2d(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(-15)), new Rotation2d(0))
          ),
          config
        );   
            
             var thetaController =
             new ProfiledPIDController(
                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
         thetaController.enableContinuousInput(-Math.PI, Math.PI);


         SwerveControllerCommand threeBallCommandOne =
             new SwerveControllerCommand(
                 testThreeBallTrajectoryBallOne,
                 s_Swerve::getPose,
                 Constants.Swerve.swerveKinematics,
                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                 thetaController,
                 Swerve.getSwerveHeadinSupplier(-130),
                 s_Swerve::setModuleStates,
                 s_Swerve);
            //-145 is good for the next angle being 40
            //-160 is good for the next angle being 20
           // idk which is better :/

         SwerveControllerCommand threeBallCommandTwo =
             new SwerveControllerCommand(
                 testThreeBallTrajectoryBallTwo,
                 s_Swerve::getPose,
                 Constants.Swerve.swerveKinematics,
                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                 thetaController,
                 Swerve.getSwerveHeadinSupplier(40),
                 s_Swerve::setModuleStates,
                 s_Swerve);
            // 40 is good if the previous angle is -145
            // 20 is good if the previous angle is -160
         
             SwerveControllerCommand threeBallCommandThree =
             new SwerveControllerCommand(
                 testThreeBallTrajectoryBallThree,
                 s_Swerve::getPose,
                 Constants.Swerve.swerveKinematics,
                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                 thetaController,
                 Swerve.getSwerveHeadinSupplier(0),
                 s_Swerve::setModuleStates,
                 s_Swerve);

         addCommands(
             new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())),
             threeBallCommandOne,
             threeBallCommandTwo,
             threeBallCommandThree
         );
     }
}