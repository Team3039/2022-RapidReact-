// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Add your docs here. */
public class TrajectoryGenerator {

    private static final TrajectoryGenerator trajectoryInstance = new TrajectoryGenerator();

    public static TrajectoryGenerator getInstance() {
        return trajectoryInstance;
    }
    
   DifferentialDriveVoltageConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        10);
    
    TrajectoryConfig testConfig =
      new TrajectoryConfig(Constants.kMinSpeedMetersPerSecond,
                Constants.kMinAcclerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

    TrajectoryConfig testConfigReversed =
      new TrajectoryConfig(Constants.kMinSpeedMetersPerSecond,
                Constants.kMinAcclerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
               .setKinematics(Constants.kDriveKinematics)
                // Apply the voltage constraint
               .setReversed(true)
               .addConstraint(autoVoltageConstraint);
               

    public Trajectory testTrajectory = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory( 
        new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
                 new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
                 new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(-40)),
                 new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(-80))
      ),
         new Pose2d(Units.inchesToMeters(140), Units.inchesToMeters(-80), Rotation2d.fromDegrees(140)), 
        testConfig
        );
     

    public Trajectory testTrajectoryReversed = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        new Pose2d(140, 0, new Rotation2d(0)),
          List.of(
                 new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(-80)),
                 new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(-40)),
                 new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
        
        ),
        new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
        testConfigReversed
    );
}
