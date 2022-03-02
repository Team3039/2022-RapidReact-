package frc.robot.subsystems;

import frc.lib.util.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.OptionalDouble;

public class LLDriver extends SubsystemBase {
    private static final double TARGET_HEIGHT = 98.25;
    private static final double LIMELIGHT_HEIGHT = 22.0;

    private static final double INNER_TARGET_RANGE_ANGLE = Math.toRadians(10.0); // "Theoretical": 18.0
    private static final double INNER_TARGET_DEPTH = 29.25;
    // The distance from the inner target to the apex of the triangle we use to find the distance
    private static final double DISTANCE_FROM_INNER_TO_APEX = 16.92;

    private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(29.0);

    private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);

    private static final Limelight LIMELIGHT = new Limelight();

    private boolean hasTarget;
    private boolean isInnerTargetVisible;
    private OptionalDouble distanceToTarget = OptionalDouble.empty();
    private OptionalDouble angleToTarget = OptionalDouble.empty();

    public static LLDriver INSTANCE = new LLDriver();

    public static LLDriver getInstance() {
        return INSTANCE;
    }

    public void setCamMode(Limelight.CamMode mode) {
        LIMELIGHT.setCamMode(mode);
    }

    public OptionalDouble getDistanceToTarget() {
        return distanceToTarget;
    }

    public OptionalDouble getAngleToTarget() {
        return angleToTarget;
    }

    public OptionalDouble getHorizontalError() {
        OptionalDouble distanceToTargetOpt = getDistanceToTarget();
        OptionalDouble angleToTargetOpt = getAngleToTarget();

        if (distanceToTargetOpt.isEmpty() || angleToTargetOpt.isEmpty()) {
            return OptionalDouble.empty();
        }

        double gyroAngle = Drive.getInstance().getPose().getRotation().getRadians();
        return OptionalDouble.of(distanceToTargetOpt.getAsDouble() *
                (Math.sin(gyroAngle - angleToTargetOpt.getAsDouble()) / Math.sin(Math.PI / 2.0 - gyroAngle)));
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public boolean isInnerTargetVisible() {
        return isInnerTargetVisible;
    }

    @Override
    public void periodic() {
        // Shooter limelight
        // Determine whether the Limelight has a target or not
        hasTarget = LIMELIGHT.hasTarget();
        if (hasTarget) {
            // Calculate the distance to the outer target
            Vector2 targetPosition = LIMELIGHT.getTargetPosition();
            double theta = LIMELIGHT_MOUNTING_ANGLE + targetPosition.y;
            double distanceToOuterTarget = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(theta);

            // Get the field oriented angle for the outer target, with latency compensation
            double angleToOuter = Drive.getInstance().getPoseAtTime(Timer.getFPGATimestamp() - LIMELIGHT.getPipelineLatency() / 1000.0).rotation.toRadians() - targetPosition.x;
            double dYOuter = distanceToOuterTarget * Math.sin(angleToOuter);
            double dXOuter = distanceToOuterTarget * Math.cos(angleToOuter);

            // Calculate the distance to the inner target
            double dXInner = dXOuter + INNER_TARGET_DEPTH;
            double distanceToInnerTarget = Math.hypot(dXInner, dYOuter);
            // Add DISTANCE_FROM_INNER_TO_APEX to dXInner here because we want if we did it when we defined dXInner
            // distanceToInnerTarget would be incorrect, and we only need this extra bit to determine if we can see
            // the inner target
            double angleToApex = Math.atan(dYOuter / (dXInner + DISTANCE_FROM_INNER_TO_APEX));
            if (angleToApex < 0.0) {
                angleToApex += 2 * Math.PI;
            }
            double angleToInner = Math.atan(dYOuter / dXInner);
            if (angleToInner < 0.0) {
                angleToInner += 2 * Math.PI;
            }

            // Check whether we can see the inner target
            isInnerTargetVisible = angleToApex <= INNER_TARGET_RANGE_ANGLE || angleToApex >= 2 * Math.PI - INNER_TARGET_RANGE_ANGLE;
            if (isInnerTargetVisible) {
                distanceToTarget = OptionalDouble.of(distanceToInnerTarget);
            } else {
                distanceToTarget = OptionalDouble.of(distanceToOuterTarget);
            }
            angleToTarget = OptionalDouble.of(angleToOuter);
        } else {
            distanceToTarget = OptionalDouble.empty();
            angleToTarget = OptionalDouble.empty();
            isInnerTargetVisible = false;
        }
        }

    public boolean isOnTarget() {
        OptionalDouble targetAngle = getAngleToTarget();
        if (targetAngle.isEmpty()) {
            return false;
        }

        double delta = targetAngle.getAsDouble() - Drive.getInstance().getPose().getRotation().getRadians();
        if (delta > Math.PI) {
            delta = 2.0 * Math.PI - delta;
        }

        return MathUtils.epsilonEquals(
                delta,
                0,
                TARGET_ALLOWABLE_ERROR
        );
    }

    public void setLedMode(Limelight.LedMode mode) {
        LIMELIGHT.setLedMode(mode);
    }

    public void setSnapshotEnabled(boolean isEnabled) {
        LIMELIGHT.setSnapshotsEnabled(isEnabled);
    }
}