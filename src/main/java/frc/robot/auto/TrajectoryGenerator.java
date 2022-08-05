package frc.robot.auto;

import java.util.List;

// import javax.management.MalformedObjectNameException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.SplineParameterizer.MalformedSplineException;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.MathUtils;
import frc.robot.Constants;

public class TrajectoryGenerator {
        public static TrajectoryGenerator INSTANCE = new TrajectoryGenerator();

        public static TrajectoryConfig configFast = new TrajectoryConfig(
                        Constants.AutoConstants.K_MAX_SPEED_METERS_PER_SECOND - 0.5,
                        Constants.AutoConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 1.5)
                                        .setKinematics(Constants.Swerve.SWERVE_KINEMATICS);
        // .setReversed(true);

        public static TrajectoryConfig configSlow = new TrajectoryConfig(
                        Constants.AutoConstants.K_MAX_SPEED_METERS_PER_SECOND - 2.5,
                        Constants.AutoConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 2.5)
                                        .setKinematics(Constants.Swerve.SWERVE_KINEMATICS);
        // .setReversed(true);

        public static class Waypoints {
                // terminal ball is human player station

                // Right Near Five Ball Waypoints
                public static Pose2d rightNearMidBall = new Pose2d(
                                new Translation2d(Units.inchesToMeters(36), Units.inchesToMeters(-20)),
                                new Rotation2d(-0.5));

                 public static Pose2d rightNearMidBallRot = new Pose2d(
                                new Translation2d(Units.inchesToMeters(36), Units.inchesToMeters(-20)),
                                new Rotation2d(90));

                public static Pose2d rightNearRightBallOne = new Pose2d(
                                new Translation2d(Units.inchesToMeters(8), Units.inchesToMeters(45)),
                                new Rotation2d(90));

                public static Pose2d rightNearRightBallTwo = new Pose2d(
                                new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(62)),
                                new Rotation2d(90));

                public static Pose2d rightNearRightBallRot = new Pose2d(
                                new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(58)),
                                new Rotation2d(36.8));

                public static Pose2d rightNearTerminalBall = new Pose2d(
                                // original x was 150, original y was -77
                        new Translation2d(Units.inchesToMeters(132), Units.inchesToMeters(-83)),
                                new Rotation2d(36.8));

                public static Pose2d rightNearTerminalBallRot = new Pose2d(
                                 // original x was 150, original y was -77
                        new Translation2d(Units.inchesToMeters(129), Units.inchesToMeters(-67)),
                        new Rotation2d(90.35));

                public static Pose2d rightNearShootPositionFinal = new Pose2d(
                        new Translation2d(Units.inchesToMeters(45), Units.inchesToMeters(-10)),
                        new Rotation2d(90.35));

                // Generic Two Ball Waypoints
                public static Pose2d genericBall = new Pose2d(
                        new Translation2d(Units.inchesToMeters(35), Units.inchesToMeters(0)),
                        new Rotation2d(0));

                // Steal Balls
                public static Pose2d leftNearBall = new Pose2d(
                        new Translation2d(Units.inchesToMeters(27), Units.inchesToMeters(0)),
                        new Rotation2d(0));
                
                public static Pose2d leftNearBallRot = new Pose2d(
                        new Translation2d(Units.inchesToMeters(27), Units.inchesToMeters(0)),
                        Rotation2d.fromDegrees(-110));

                public static Pose2d stealFarLeftBallOne = new Pose2d(
                        new Translation2d(Units.inchesToMeters(20), Units.inchesToMeters(-25)),
                        Rotation2d.fromDegrees(-110));

                public static Pose2d stealFarLeftBallTwo = new Pose2d(
                        new Translation2d(Units.inchesToMeters(20), Units.inchesToMeters(-25)),
                        Rotation2d.fromDegrees(-45));

                public static Pose2d stealFarLeftBallThree = new Pose2d(
                        new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(-35)),
                        Rotation2d.fromDegrees(-45));

                public static Pose2d stealFarLeftBallRot = new Pose2d(
                        new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(-35)),
                        Rotation2d.fromDegrees(38.5));
                
                public static Pose2d stealMidBall = new Pose2d(
                        new Translation2d(Units.inchesToMeters(5), Units.inchesToMeters(99)),
                        Rotation2d.fromDegrees(-45));

                public static Pose2d stealMidBallRot = new Pose2d(
                        new Translation2d(Units.inchesToMeters(5), Units.inchesToMeters(99)),
                        Rotation2d.fromDegrees(-45));
        
                public static Pose2d ejectBalls = new Pose2d(
                        new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(0)),
                        Rotation2d.fromDegrees(38.5));


        }

        // Right Far Trajectories

        // public static Trajectory getRightFarStartToFirstBall() throws MalformedSplineException {
        //         try {
        //                 return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        //                                 List.of(
        //                                                 new Pose2d(),
        //                                                 Waypoints.rightFarRightBall),
        //                                 configLow);
        //         } catch (MalformedSplineException e) {
        //                 e.printStackTrace();
        //                 System.out.println("Spline Malformed");
        //                 CommandScheduler.getInstance().cancelAll();
        //                 System.out.println("Autonomous Canceled");
        //         }
        //         return new Trajectory();
        // }

        // public static Trajectory getRightFarFirstBallToShootingPoint() throws MalformedSplineException {
        //         try {
        //                 return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        //                                 List.of(
        //                                                 Waypoints.rightFarRightBall,
        //                                                 new Pose2d()),
        //                                 configFast);
        //         } catch (MalformedSplineException e) {
        //                 e.printStackTrace();
        //                 System.out.println("Spline Malformed");
        //                 CommandScheduler.getInstance().cancelAll();
        //                 System.out.println("Autonomous Canceled");
        //         }
        //         return new Trajectory();
        // }

        // public static Trajectory getRightFarFirstBallToSecondBall() throws MalformedSplineException {
        //         try {
        //                 return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        //                                 List.of(
        //                                                 Waypoints.rightFarRightBall,
        //                                                 Waypoints.rightFarMidBall),
        //                                 configFast);
        //         } catch (MalformedSplineException e) {
        //                 e.printStackTrace();
        //                 System.out.println("Spline Malformed");
        //                 CommandScheduler.getInstance().cancelAll();
        //                 System.out.println("Autonomous Canceled");
        //         }
        //         return new Trajectory();
        // }

        // public static Trajectory getRightFarShootingPointToSecondBall() throws MalformedSplineException {
        //         try {
        //                 return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        //                                 List.of(
        //                                                 new Pose2d(),
        //                                                 Waypoints.rightFarMidBall),
        //                                 configFast);
        //         } catch (MalformedSplineException e) {
        //                 e.printStackTrace();
        //                 System.out.println("Spline Malformed");
        //                 CommandScheduler.getInstance().cancelAll();
        //                 System.out.println("Autonomous Canceled");
        //         }
        //         return new Trajectory();
        // }

        // public static Trajectory getRightFarSecondBallToThirdBall() throws MalformedSplineException {
        //         try {
        //                 return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        //                                 List.of(
        //                                                 Waypoints.rightFarMidBall,
        //                                                 Waypoints.rightFarTerminalBall),
        //                                 configFast);
        //         } catch (MalformedSplineException e) {
        //                 e.printStackTrace();
        //                 System.out.println("Spline Malformed");
        //                 CommandScheduler.getInstance().cancelAll();
        //                 System.out.println("Autonomous Canceled");
        //         }
        //         return new Trajectory();
        // }

        // public static Trajectory getRightFarThirdBallToFinish() throws MalformedSplineException {
        //         try {
        //                 return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        //                                 List.of(
        //                                                 Waypoints.rightFarTerminalBall,
        //                                                 Waypoints.rightFarMidBall),
        //                                 configFast);
        //         } catch (MalformedSplineException e) {
        //                 e.printStackTrace();
        //                 System.out.println("Spline Malformed");
        //                 CommandScheduler.getInstance().cancelAll();
        //                 System.out.println("Autonomous Canceled");
        //         }
        //         return new Trajectory();
        // }
        // Right Near Trajectories

        public static Trajectory getRightNearStartToFirstBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        new Pose2d(new Translation2d(0, 0), new Rotation2d(-0.5)),
                                                        Waypoints.rightNearMidBall),
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        public static Trajectory getRightNearFirstBallToSecondBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(        
                                                        Waypoints.rightNearMidBallRot,
                                                        Waypoints.rightNearRightBallOne,
                                                        Waypoints.rightNearRightBallTwo),
                                        
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        // public static Trajectory getRightNearSecondBallToStart() throws MalformedSplineException {
        //         try {
        //                 return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        //                                 List.of(
        //                                                 Waypoints.rightNearRightBall,
        //                                                 new Pose2d()),
        //                                 configSlow);
        //         } catch (MalformedSplineException e) {
        //                 e.printStackTrace();
        //                 System.out.println("Spline Malformed");
        //                 CommandScheduler.getInstance().cancelAll();
        //                 System.out.println("Autonomous Canceled");
        //         }
        //         return new Trajectory();
        // }

        // public static Trajectory getRightNearStartToThirdBall() throws MalformedSplineException {
        //         try {
        //                 return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
        //                                 List.of(
        //                                                 new Pose2d(),
        //                                                 Waypoints.rightNearTerminalBall),
        //                                 configSlow);
        //         } catch (MalformedSplineException e) {
        //                 e.printStackTrace();
        //                 System.out.println("Spline Malformed");
        //                 CommandScheduler.getInstance().cancelAll();
        //                 System.out.println("Autonomous Canceled");
        //         }
        //         return new Trajectory();
        // }

        public static Trajectory getRightNearSecondBallToThirdBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        Waypoints.rightNearRightBallRot,
                                                        Waypoints.rightNearTerminalBall),
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        public static Trajectory getRightNearThirdBallToFinish() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        Waypoints.rightNearTerminalBallRot,
                                                        Waypoints.rightNearShootPositionFinal),
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        // Generic Two Ball Trajectories
        public static Trajectory getGenericStartToFirstBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        new Pose2d(),
                                                        Waypoints.genericBall),
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        // Steal One BalL Trajectories
        public static Trajectory getStartToLeftNearBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        new Pose2d(),
                                                        Waypoints.leftNearBall),
                                        configSlow);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        public static Trajectory getLeftNearBallToLeftFarBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        Waypoints.leftNearBallRot,
                                                        Waypoints.stealFarLeftBallOne,
                                                        Waypoints.stealFarLeftBallTwo,
                                                        Waypoints.stealFarLeftBallThree),
                                        configSlow);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        public static Trajectory getLeftFarBallToEjectBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        Waypoints.stealFarLeftBallRot,
                                                        Waypoints.ejectBalls),
                                        configSlow);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }
}