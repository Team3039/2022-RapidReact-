package frc.robot;

public class InterpolatedPS4Gamepad extends PS4Gamepad {

    static double deadZoneThreshold;
    static double fullThrottleThreshold;

    public InterpolatedPS4Gamepad(int port) {
        super(port);

        deadZoneThreshold = 0.05;
        fullThrottleThreshold = 0.9;
    }

    public static boolean inDeadZone(double axis) {
        return (axis > -deadZoneThreshold) && (axis < deadZoneThreshold);
    }

    public static boolean isCeiling(double axis) {
        return axis <= -fullThrottleThreshold || axis >= fullThrottleThreshold;
    }

    public double interpolatedLeftYAxis() {
            if (Math.abs(this.getLeftYAxis()) <= 0.05)
                return 0.0;
            return ((Math.sin(this.getLeftYAxis())))/2;
        }

    public double interpolatedLeftXAxis() {
        if (Math.abs(this.getLeftXAxis()) <= 0.05)
            return 0.0;
        return ((Math.sin(this.getLeftXAxis())))/2;
    }

    public double interpolatedRightXAxis() {
        if (Math.abs(this.getRightXAxis()) <= 0.05)
            return 0.0;
        return -(Math.sin(this.getRightXAxis()))/2;
    }
}