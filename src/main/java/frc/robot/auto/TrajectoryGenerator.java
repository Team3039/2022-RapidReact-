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
import frc.robot.Constants;

public class TrajectoryGenerator {
        public static TrajectoryGenerator INSTANCE = new TrajectoryGenerator();

        public static TrajectoryConfig configFast = new TrajectoryConfig(
                        Constants.AutoConstants.K_MAX_SPEED_METERS_PER_SECOND,
                        Constants.AutoConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                                        .setKinematics(Constants.Swerve.SWERVE_KINEMATICS);
        // .setReversed(true);

        public static TrajectoryConfig configLow = new TrajectoryConfig(
                        Constants.AutoConstants.K_MAX_SPEED_METERS_PER_SECOND - 2,
                        Constants.AutoConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED - 2)
                                        .setKinematics(Constants.Swerve.SWERVE_KINEMATICS);
        // .setReversed(true);

        public static class Waypoints {
                // terminal ball is human player station
                // Right Far Four Ball Waypoints
                public static Pose2d rightFarRightBall = new Pose2d(
                                new Translation2d(Units.inchesToMeters(22), Units.inchesToMeters(0)),
                                new Rotation2d());

                public static Pose2d rightFarMidBall = new Pose2d(
                                new Translation2d(Units.inchesToMeters(-5), Units.inchesToMeters(-60)),
                                new Rotation2d(0));

                public static Pose2d rightFarTerminalBall = new Pose2d(
                                new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-200)),
                                new Rotation2d(0));

                // Right Near Four Ball Waypoints
                public static Pose2d rightNearMidBall = new Pose2d(
                                new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(-15)),
                                new Rotation2d(0));

                public static Pose2d rightNearRightBall = new Pose2d(
                                new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(60)),
                                new Rotation2d(0));

                public static Pose2d rightNearTerminalBall = new Pose2d(
                                new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(-75)),
                                new Rotation2d(0));

                // Left Far Four Ball Waypoints
                public static Pose2d LeftFarLeftBall = new Pose2d(
                                new Translation2d(Units.inchesToMeters(35), Units.inchesToMeters(3.5)),
                                new Rotation2d(0));
        }

        // Right Far Four Ball Trajectories

        public static Trajectory getRightFarStartToFirstBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        new Pose2d(),
                                                        Waypoints.rightFarRightBall),
                                        configLow);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        public static Trajectory getRightFarFirstBallToShootingPoint() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        Waypoints.rightFarRightBall,
                                                        new Pose2d()),
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        public static Trajectory getRightFarFirstBallToSecondBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        Waypoints.rightFarRightBall,
                                                        new Pose2d()),
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        public static Trajectory getRightFarShootingPointToSecondBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        new Pose2d(),
                                                        Waypoints.rightFarMidBall),
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        public static Trajectory getRightFarSecondBallToThirdBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        Waypoints.rightFarMidBall,
                                                        Waypoints.rightFarTerminalBall),
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        public static Trajectory getRightFarThirdBallToFinish() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        Waypoints.rightFarTerminalBall,
                                                        Waypoints.rightFarMidBall),
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }
        // Right Near Four Ball Trajectories

        public static Trajectory getRightNearStartToFirstBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        new Pose2d(),
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
                                                        Waypoints.rightNearMidBall,
                                                        Waypoints.rightNearRightBall),
                                        configFast);
                } catch (MalformedSplineException e) {
                        e.printStackTrace();
                        System.out.println("Spline Malformed");
                        CommandScheduler.getInstance().cancelAll();
                        System.out.println("Autonomous Canceled");
                }
                return new Trajectory();
        }

        public static Trajectory getRightNearSecondBallToThirdBall() throws MalformedSplineException {
                try {
                        return edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                                        Waypoints.rightNearRightBall,
                                                        Waypoints.rightNearMidBall,
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
                                                        Waypoints.rightNearTerminalBall,
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
}