// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.auto.routines;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
// import frc.robot.auto.commands.StopTrajectory;
// import frc.robot.subsystems.Drive;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class RightFarFourBallAuto extends SequentialCommandGroup {
    
//     /** Creates a new FourBallBottomAuto. */
//     public RightFarFourBallAuto(Drive s_Swerve) {

//         var thetaController = new ProfiledPIDController(
//                 Constants.AutoConstants.KP_THETA_CONTROLLER, 0, 0,
//                 Constants.AutoConstants.K_THETA_CONTROLLER_CONSTRAINTS);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         SwerveControllerCommand fourBallCommandOne = new SwerveControllerCommand(
//                 frc.robot.auto.TrajectoryGenerator.getRightFarStartToFirstBall(),
//                 s_Swerve::getPose,
//                 Constants.Swerve.SWERVE_KINEMATICS,
//                 new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
//                 new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
//                 thetaController,
//                 Drive.getSwerveHeadingSupplier(0),
//                 s_Swerve::setModuleStates,
//                 s_Swerve);

//         SwerveControllerCommand fourBallCommandTwo = new SwerveControllerCommand(
//                 frc.robot.auto.TrajectoryGenerator.getRightFarFirstBallToShootingPoint(),
//                 s_Swerve::getPose,
//                 Constants.Swerve.SWERVE_KINEMATICS,
//                 new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
//                 new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
//                 thetaController,
//                 Drive.getSwerveHeadingSupplier(170),
//                 s_Swerve::setModuleStates,
//                 s_Swerve);

//         SwerveControllerCommand fourBallCommandThree = new SwerveControllerCommand(
//                 frc.robot.auto.TrajectoryGenerator.getRightFarShootingPointToSecondBall(),
//                 s_Swerve::getPose,
//                 Constants.Swerve.SWERVE_KINEMATICS,
//                 new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
//                 new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
//                 thetaController,
//                 Drive.getSwerveHeadingSupplier(-120),
//                 s_Swerve::setModuleStates,
//                 s_Swerve);

//         SwerveControllerCommand fourBallCommandFour = new SwerveControllerCommand(
//                 frc.robot.auto.TrajectoryGenerator.getRightFarSecondBallToThirdBall(),
//                 s_Swerve::getPose,
//                 Constants.Swerve.SWERVE_KINEMATICS,
//                 new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
//                 new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
//                 thetaController,
//                 // Swerve.getSwerveHeadingSupplier(-25),
//                 s_Swerve::setModuleStates,
//                 s_Swerve);

//         SwerveControllerCommand fourBallCommandFive = new SwerveControllerCommand(
//                 frc.robot.auto.TrajectoryGenerator.getRightFarThirdBallToFinish(),
//                 s_Swerve::getPose,
//                 Constants.Swerve.SWERVE_KINEMATICS,
//                 new PIDController(Constants.AutoConstants.KPX_CONTROLLER, 0, 0),
//                 new PIDController(Constants.AutoConstants.KPY_CONTROLLER, 0, 0),
//                 thetaController,
//                 Drive.getSwerveHeadingSupplier(135),
//                 s_Swerve::setModuleStates,
//                 s_Swerve);

//         // Add your commands in the addCommands() call, e.g.
//         // addCommands(new FooCommand(), new BarCommand());
//         addCommands(
//                 new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())),
//                 fourBallCommandOne,
//                 new StopTrajectory(),
//                 new WaitCommand(1),
//                 fourBallCommandTwo,
//                 new StopTrajectory(),
//                 fourBallCommandThree,
//                 fourBallCommandFour,
//                 new StopTrajectory(),
//                 fourBallCommandFive,
//                 new StopTrajectory());
//     }
// }