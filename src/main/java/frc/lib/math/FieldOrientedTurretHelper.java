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
    
    public static double startAngle;
    public static Pose2d hubPose;

    public FieldOrientedTurretHelper(Start_Pose startPose) {
        switch(startPose) {
            case LEFT_FAR:
                startAngle = 133.5;
                hubPose = new Pose2d();
                break;
            case LEFT_NEAR:
                startAngle = -178.5;
                hubPose = new Pose2d();
                break;
            case RIGHT_FAR:
                startAngle = -88.5;
                hubPose = new Pose2d(Units.inchesToMeters(-93), Units.inchesToMeters(25), Rotation2d.fromDegrees(0));
                break;
            case RIGHT_NEAR:
                startAngle = -136.5;
                hubPose = new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(3), Rotation2d.fromDegrees(0));
                break;
            default:
                break;
        }
    }
    public static Rotation2d getAngleToTarget(Pose2d currentPose) {
        double dX = currentPose.getX() - hubPose.getX();
        double dY = currentPose.getY() - hubPose.getY();
        
        return Rotation2d.fromDegrees(Math.tan(dX / dY));
    }
}
