package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.KD_SNAP;
import static frc.robot.Constants.Swerve.KI_SNAP;
import static frc.robot.Constants.Swerve.KP_SNAP;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Drive extends SubsystemBase {

    public static Drive INSTANCE = new Drive();

    public static enum DriveState {
        AUTO,
        PRECISION,
        DEFENSE,
        SPEED
    }

    public static DriveState mState = DriveState.AUTO;

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;

    public ProfiledPIDController snapPIDController;

    public static TrapezoidProfile.Constraints thetaController;

    public boolean isSnapping;
    public boolean isHighGear;
    public boolean isAltCenter;

    public Drive() {
        gyro = new PigeonIMU(Constants.RobotMap.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_KINEMATICS, getYaw());

        thetaController = new TrapezoidProfile.Constraints(
                Constants.AutoConstants.K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                Constants.AutoConstants.K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        snapPIDController = new ProfiledPIDController(KP_SNAP, KI_SNAP, KD_SNAP, thetaController);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        isSnapping = false;
        isHighGear = false;
    }

    public static Drive getInstance() {
        return INSTANCE;
    }

    public static DriveState getState() {
        return mState;
    }

    public static void setState(DriveState traverseState) {
        mState = traverseState;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if (isSnapping) {
            if (Math.abs(rotation) == 0.0) {
                checkStopSnap(false);
                rotation = calculateSnapValue();
            } else {
                checkStopSnap(true);
            }
        }
        SwerveModuleState[] swerveModuleStates = 
        Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        isHighGear ? MathUtil.clamp(translation.getX(), -1.0, 1.0) : translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void checkStopSnap(boolean force) {
        if (!isSnapping) {
            return;
        }
        if (force || snapComplete()) {
            isSnapping = false;
            snapPIDController.reset(getYaw().getRadians());
        }
    }

    public double calculateSnapValue() {
        return snapPIDController.calculate(getYaw().getRadians());
    }

    public void startSnap(double snapAngle) {
        snapPIDController.reset(getYaw().getRadians());
        snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
        isSnapping = true;
    }

    private boolean snapComplete() {
        double error = snapPIDController.getGoal().position - getYaw().getRadians();
        return Math.abs(error) < Math.toRadians(Constants.Swerve.ESPILON_SNAP);
    }

    public void setGear(boolean isHighGear) {
        this.isHighGear = isHighGear;
    }

    public boolean getGear() {
        return isHighGear;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public static Supplier<Rotation2d> getSwerveHeadingSupplier(double theta) {
        return () -> Rotation2d.fromDegrees(theta);
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic() {
        switch (Drive.getInstance().getState()) {
            case AUTO:
                swerveOdometry.update(getYaw(), getStates());
                isAltCenter = false;
                break;
            case DEFENSE:
                isHighGear = true;
                isAltCenter = true;
                break;
            case PRECISION:
                isHighGear = false;
                isAltCenter = false;
                break;
            case SPEED:
                isHighGear = true;
                isAltCenter = false;
                break;
            default:
                isHighGear = true;
                isAltCenter = false;
                break;
        }

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}