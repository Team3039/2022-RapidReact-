package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.subsystems.Drive;

public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;

    private Drive s_Swerve;
    private InterpolatedPS4Gamepad controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(Drive s_Swerve, InterpolatedPS4Gamepad controller, int translationAxis, int strafeAxis,
            int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        if (RobotState.isTeleop()) {
            double yAxis = -controller.interpolatedLeftYAxis();
            double xAxis = -controller.interpolatedLeftXAxis();
            double rAxis = controller.interpolatedRightXAxis();

            translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.MAX_SPEED);
            rotation = rAxis * Constants.Swerve.MAX_ANGULAR_VELOCITY;
            s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
        }
    }
}