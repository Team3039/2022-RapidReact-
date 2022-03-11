package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FieldOrientedTurretHelper {

    public enum Start_Pose {
        LEFT_FAR,
        LEFT_NEAR,
        RIGHT_FAR,
        RIGHT_NEAR
    }
    
    private Start_Pose mStartPose;
    private Pose2d mTarget;

    public FieldOrientedTurretHelper(Start_Pose startPose) {
        switch(startPose) {
            case LEFT_FAR:
                mTarget = new Pose2d();
                break;
            case LEFT_NEAR:
                mTarget = new Pose2d();
                break;
            case RIGHT_FAR:
                mTarget = new Pose2d(Units.inchesToMeters(-93), Units.inchesToMeters(25), Rotation2d.fromDegrees(0));
                break;
            case RIGHT_NEAR:
                mTarget = new Pose2d();
                break;
            default:
                break;
        }
    }
    public Rotation2d getAngleToTarget(Pose2d currentPose) {
        double dX = currentPose.getX() - mTarget.getX();
        double dY = currentPose.getY() - mTarget.getY();
        
        return Rotation2d.fromDegrees(Math.tan(dX / dY));
    }
}
