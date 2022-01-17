package frc.robot;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class InterpolatedPS4Gamepad extends PS4Gamepad {

    static double deadZoneThreshold;
    static double fullThrottleThreshold;

    static RampComponent rComponent;

    public InterpolatedPS4Gamepad(int port) {
        super(port);

        deadZoneThreshold = 0.05;
        fullThrottleThreshold = 0.9;

        rComponent = new RampComponent(1, 0.5);
    }

    public static boolean inDeadZone(double axis) {
        return (axis > -deadZoneThreshold) && (axis < deadZoneThreshold);
    }

    public static boolean isCeiling(double axis) {
        return axis <= -fullThrottleThreshold || axis >= fullThrottleThreshold;
    }

    public double interpolatedLeftYAxis() {
            if (this.getLeftYAxis() <= 0.01)
                return 0.0;
            return rComponent.applyAsDouble(-(Math.pow(4, this.getLeftYAxis())));
        }

    public double interpolatedLeftXAxis() {
        if (this.getLeftXAxis() <= 0.01)
            return 0.0;
        return rComponent.applyAsDouble(-(Math.pow(4, this.getLeftXAxis())));
    }

    public double interpolatedRightXAxis() {
        if (this.getRightXAxis() <= 0.01)
            return 0.0;
        return rComponent.applyAsDouble(-(Math.pow(4, this.getRightXAxis())));
    }
}