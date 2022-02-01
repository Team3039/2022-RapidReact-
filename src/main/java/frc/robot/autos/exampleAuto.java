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

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){
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

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(
                    new Pose2d(),
                    new Pose2d(new Translation2d(Units.inchesToMeters(78), 0), Rotation2d.fromDegrees(0)),
                    // new Pose2d(new Translation2d(Units.inchesToMeters(78), 0), Rotation2d.fromDegrees(0)),
                    new Pose2d(new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(-50)), Rotation2d.fromDegrees(0))
                ),
                configLow
                );

            Trajectory exA =
                TrajectoryGenerator.generateTrajectory(
                    List.of(
                        new Pose2d(new Translation2d(Units.inchesToMeters(78), 0), Rotation2d.fromDegrees(0)),
                        new Pose2d(new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(-50)), Rotation2d.fromDegrees(0))
                    ),
                    configLow
                    );

                Trajectory exampleB =
                    TrajectoryGenerator.generateTrajectory(
                        List.of(
                            new Pose2d(new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(-50)), Rotation2d.fromDegrees(0)),
                            new Pose2d(new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(0)), Rotation2d.fromDegrees(0))
                        ), 
                        configLow
                        );

            // TrajectoryGenerator.generateTrajectory(
            //     // Start at the origin facing the +X direction
            //     new Pose2d(0, 0, new Rotation2d(0)),
            //     List.of(),
            //     new Pose2d(Units.inchesToMeters(140), 0, Rotation2d.fromDegrees(179)),
            //     config);

        Trajectory exTrajectory =
            TrajectoryGenerator.generateTrajectory(new Pose2d(Units.inchesToMeters(140), 0, Rotation2d.fromDegrees(179)), List.of(), new Pose2d(Units.inchesToMeters(190), 0, Rotation2d.fromDegrees(179)), config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::getHalfPi,
                s_Swerve::setModuleStates,
                s_Swerve
            );

        SwerveControllerCommand mCommand =
            new SwerveControllerCommand(
            exA,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::getHalfPi,
            s_Swerve::setModuleStates,
            s_Swerve);

        SwerveControllerCommand mCommandB =
            new SwerveControllerCommand(
            exampleB,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::getQuarterPi,
            s_Swerve::setModuleStates,
            s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand,
           // mCommand,
            mCommandB
            // mCommand
        );
    }
}