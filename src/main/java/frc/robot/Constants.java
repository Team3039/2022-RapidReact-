package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double STICK_DEADBAND = 0.1;

    public static final class RobotMap {

        public static final int pigeonID = 12;

        /* change these later when robot is fully wired */
        public static final int intake = 1;
        public static final int intakeDeploy = 1;

        public static final int hopperFeeder = 1;
        public static final int hopperGripper = 1;

        public static final int hopperFeederGate = 0;
        public static final int hopperGripperGate = 1;

        public static final int shooter = 1;

        public static final int climberRaiseA = 1;
        public static final int climberRaiseB = 1;

        public static final int climberTiltA = 1;
        public static final int climberTiltB = 1;

        public static final int turret = 1;

    }

    public static final class Shooter {
        public static final double SHOOTER_TO_ENCODER_RATIO = 1;
        public static final double TICKS_PER_ROTATION = 2048;
    }

    public static final class Turret {
        public static final double kP_TURRET_TRACK = 0.03;
        public static final double kP_TURRET_RESET_POS = 0.015;
    }
    public static final class Swerve {
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.80);
        public static final double WHEEL_BASE = Units.inchesToMeters(22.80);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.99);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0); // 6.86:1
        public static final double ANGLE_GEAR_RATIO = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        public static final SwerveModuleState[] ZERO_STATES = {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 0.2;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.1;
        public static final double ANGLE_KF = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.10;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double DRIVE_KS = (0.667 / 12); // divide by 12 to convert from volts to percent output for
                                                            // CTRE
        public static final double DRIVE_KV = (2.44 / 12);
        public static final double DRIVE_KA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 4.5; // meters per second
        public static final double MAX_ANGULAR_VELOCITY = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERT = false;
        public static final boolean ANGLE_MOTOR_INVERT = false;

        /* Angle Encoder Invert */
        public static final boolean CANCONDER_INVERT = false;

        /* Snap Constants */

        public static final double KP_SNAP = 3.0;
        public static final double KI_SNAP = 0;
        public static final double KD_SNAP = 0.0;
        public static final double ESPILON_SNAP = 0.01;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 0;
            public static final double angleOffset = 79.36;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 1;
            public static final double angleOffset = 164.08;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final double angleOffset = 119.97;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 2;
            public static final double angleOffset = 97.99;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double K_MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.5;
        public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double KPX_CONTROLLER = 1;
        public static final double KPY_CONTROLLER = 1;
        public static final double KP_THETA_CONTROLLER = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }
}