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

    public static final class Ports {
        
        // CAN
        public static final int INTAKE = 14;

        public static final int FIRST_STAGE = 9;
        public static final int SECOND_STAGE = 10;

        public static final int SHOOTER_MASTER = 11;
        public static final int SHOOTER_SLAVE = 12;

        public static final int CLIMB_MASTER = 15;
        public static final int CLIMB_SLAVE = 13;

        public static final int TURRET = 8;
        
        // DIO
        public static final int FIRST_STAGE_GATE = 0;
        public static final int SECOND_STAGE_GATE = 1;

        public static final int LED_OUTPUT_A = 2;
        public static final int LED_OUTPUT_B = 3;
        
        // PCM
        public static final int INTAKE_SOLENOID = 3;

        public static final int CLIMB_ACTUATOR = 2;

        // PWM
        public static final int LEFT_HOOD = 0;
        public static final int RIGHT_HOOD = 1;
    }

    public static final class Shooter {
        public static final double SHOOTER_TO_ENCODER_RATIO = 1;
        public static final double TICKS_PER_ROTATION = 2048;
    }

    public static final class Turret {
        public static final double kP_TURRET_TRACK = 0.02;
    }

    public static final class Climber {
        public static final int TRAVERSING_BARS_ENCODER_LIMIT = 99999999;
        public static final int TELESCOPING_TO_MID_BAR_LIMIT = 219000;
    }

    public static final class Swerve {
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-
        
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
            public static final double angleOffset = 81.474;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 1;
            public static final double angleOffset = 300.849;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 2;
            public static final double angleOffset = 344.707;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final double angleOffset = 102.041;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double K_MAX_SPEED_METERS_PER_SECOND = 3.5;
        public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4.25;
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
