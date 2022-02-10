package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.KD_SNAP;
import static frc.robot.Constants.Swerve.KI_SNAP;
import static frc.robot.Constants.Swerve.KP_SNAP;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {

    public static Swerve INSTANCE = new Swerve();

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;

    public boolean isSnapping;
    public ProfiledPIDController snapPIDController;

    public static Trajectory trajectory = new Trajectory();
    
    public static TrapezoidProfile.Constraints thetaController; 

    public Swerve() {
        gyro = new PigeonIMU(Constants.RobotMap.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

        thetaController = new TrapezoidProfile.Constraints(
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,  
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared
        );
      
    

        isSnapping = false;
        snapPIDController = new ProfiledPIDController(KP_SNAP, KI_SNAP, KD_SNAP, thetaController);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
            String trajectoryJSON = "\n  C:/Users/Robotics/Documents/GitHub/BaseFalconSwerve/src/main/deploy/output/Unnamed.wpilib.json";
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            try {
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException e) {
                e.printStackTrace();
            }
    }

    public static Swerve getInstance() {
        return INSTANCE;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if(isSnapping) {
            if(Math.abs(rotation) == 0.0) {
                checkStopSnap(false);
                rotation = calculateSnapValue();
            } else {
                checkStopSnap(true);
            }
        } 
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void checkStopSnap(boolean force){
        if(!isSnapping){
            return;
        } 
        if(force || snapComplete()) {
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



    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
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

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}