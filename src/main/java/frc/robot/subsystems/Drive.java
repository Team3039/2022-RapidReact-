package frc.robot.subsystems;

<<<<<<< Updated upstream
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
=======
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
>>>>>>> Stashed changes
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
<<<<<<< Updated upstream

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
=======
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Drive extends SubsystemBase {
<<<<<<< Updated upstream

  public enum DriveControlMode {
    JOYSTICK, PATH_FOLLOWING
  }

  private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;

  // Left Drive
  private WPI_TalonFX leftMaster;
  private TalonFX leftSlave;

  // Right Drive
  private WPI_TalonFX rightMaster;
  private TalonFX rightSlave;

  private TalonSRX pigeonTalon;

  // Gyro
  private PigeonIMU gyroPigeon;
  private double[] yprPigeon = new double[3];
  private boolean isCalibrating = false;
  private double gyroYawOffsetAngleDeg = 0;

  // Differential Drive
  private DifferentialDrive drive;

  //Path Following
  private final DifferentialDriveOdometry odometry;


  // Subsystem Instance
  private final static Drive INSTANCE = new Drive();

  public Drive() {
    leftMaster = new WPI_TalonFX(RobotMap.leftFrontDrive);
    leftSlave = new TalonFX(RobotMap.leftRearDrive);

    rightMaster = new WPI_TalonFX(RobotMap.rightFrontDrive);
    rightSlave = new TalonFX(RobotMap.rightRearDrive);

    pigeonTalon = new TalonSRX(9);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    leftMaster.configAllSettings(configs);
    leftSlave.configAllSettings(configs);

    rightMaster.configAllSettings(configs);
    rightSlave.configAllSettings(configs);

    leftMaster.setInverted(TalonFXInvertType.Clockwise);
    leftSlave.setInverted(TalonFXInvertType.FollowMaster);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);

    leftSlave.follow(leftMaster);

    rightMaster.setInverted(TalonFXInvertType.CounterClockwise);
    rightSlave.setInverted(TalonFXInvertType.FollowMaster);

    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

    rightSlave.follow(rightMaster);

    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setSafetyEnabled(false);

    gyroPigeon = new PigeonIMU(pigeonTalon);
    gyroPigeon.configFactoryDefault();
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightSlave.configFactoryDefault();
    
    drive.setDeadband(0.05);

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroFusedHeadingAngleDeg()));
  }

  public static Drive getInstance() {
    return INSTANCE;
  }

  //Encoder Setup
  public synchronized DriveControlMode getControlMode() {
    return driveControlMode;
  }

  public synchronized void setControlMode(DriveControlMode controlMode) {
    this.driveControlMode = controlMode;
  }

  // Encoder Setup

  //Returns left sensors velocity in ticks per 100ms
  public double getLeftVelocityNativeUnits() {
    return leftMaster.getSelectedSensorVelocity(0);
  }

  //Returns right sensors velocity in ticks per 100ms
  public double getRightVelocityNativeUnits() {
    return leftMaster.getSelectedSensorVelocity(0);
  }

  //Returns left sensors position in ticks
  public double getLeftSensorPosition(){
    return leftMaster.getSelectedSensorPosition(0);
  }

  //Returns right sensors position in ticks
  public double getRightSensorPosition(){
    return leftMaster.getSelectedSensorPosition(0);
  }

  //Takes that times the wheel has rotated * by the circumference of the wheel to get its distance traveled in inches
  public static double rotationsToInches(double rotations) {
    return rotations * (Constants.kWheelDiameterInches * Math.PI);
  }

  //Takes inches and converts it to meters using units class
  public static double inchesToMeters(double inches){
    return Units.inchesToMeters(inches);
  }

  //Takes the sensor velocity of an encoder * by 10 to get ticks per second / the encoder PPR to get encoder rotations
  //per second and then uses the rotations to inches functions to get inches per second
  private static double ticksPer100msToInchesPerSec(double ticks_100ms) {
    return rotationsToInches(ticks_100ms * 10.0 / 2048);
  }

  //Returns left inches per second using the sensor velocity and the ticksToInches conversion method
  public double getLeftInchesPerSecond(){
    return ticksPer100msToInchesPerSec(getLeftVelocityNativeUnits()) / Constants.kEncoderRotationToWheelRotationRatio;
  }

  //Returns right inches per second using the sensor velocity and the ticksToInches conversion method
  public double getRightInchesPerSecond(){
    return ticksPer100msToInchesPerSec(getRightVelocityNativeUnits()) / Constants.kEncoderRotationToWheelRotationRatio;
  }

  //Returns left meters per second using inchesPerSecond calculation and inchesToMeters method
  public double getLeftMetersPerSecond(){
    return inchesToMeters(getLeftInchesPerSecond());
  }

  //Returns right meters per second using inchesPerSecond calculation and inchesToMeters method
  public double getRightMetersPerSecond(){
    return inchesToMeters(getRightInchesPerSecond());
  }

  //Sensors positions in ticks / Pulses per Revolution of the Encoder = Encoder Rotations (If ratio is 1:1)
  public double getLeftEncoderRotations() {
    return getLeftSensorPosition() / Constants.kDriveEncoderPPR;
  }

  public double getRightEncoderRotations() {
    return getRightSensorPosition() / Constants.kDriveEncoderPPR;
  }

  //Wheel Rotations = Encoder Rotations (If ratio is 1:1)
  public double getLeftWheelRotations() {
    return getLeftEncoderRotations() / Constants.kEncoderRotationToWheelRotationRatio;
  }

  public double getRightWheelRotations() {
    return getRightEncoderRotations() / Constants.kEncoderRotationToWheelRotationRatio;
  }

  //Returns left distance traveled in inches by taking wheel rotations and converting it to inches
  public double getLeftWheelDistanceInches() {
    return rotationsToInches(getLeftWheelRotations());
  }

  //Returns right distance traveled in inches by taking wheel rotations and converting it to inches
  public double getRightWheelDistanceInches() {
    return rotationsToInches(getRightWheelRotations());
  }

  //Returns left distance traveled in meters using calculated inches distances and inchesToMeters conversion
  public double getLeftWheelDistanceMeters() {
    return inchesToMeters(getLeftWheelDistanceInches());
  }

  //Returns right distance traveled in meters using calculated inches distances and inchesToMeters conversion
  public double getRightWheelDistanceMeters(){
    return inchesToMeters(getRightWheelDistanceInches());
  }

  public synchronized void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  // Gyro Set Up
  public void calibrateGyro() {
    gyroPigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.Temperature);
  }

  public void endGyroCalibration() {
    if (isCalibrating == true) {
      isCalibrating = false;
    }
  }

  public void setGyroYawOffset(double offsetDeg) {
    gyroYawOffsetAngleDeg = offsetDeg;
  }

  public synchronized double getGyroYawAngleDeg() {
    gyroPigeon.getYawPitchRoll(yprPigeon);
    return yprPigeon[0] + gyroYawOffsetAngleDeg;
  }

  public synchronized double getGyroFusedHeadingAngleDeg() {
    return (gyroPigeon.getFusedHeading() + gyroYawOffsetAngleDeg) % 360;
  }

  public synchronized double getGyroPitchAngle() {
    gyroPigeon.getYawPitchRoll(yprPigeon);
    return yprPigeon[2];
  }

  public synchronized void resetGyroYawAngle() {
    gyroPigeon.setYaw(0);
    gyroPigeon.setFusedHeading(0);
  }

  public synchronized void resetGyroYawAngle(double homeAngle) {
    resetGyroYawAngle();
    setGyroYawOffset(homeAngle);
  }

  public synchronized void driveWithJoystick() {
    double y = -1 * RobotContainer.getDriver().getLeftYAxis() * Constants.DRIVER_Y;
    double rot = RobotContainer.getDriver().getRightXAxis() * Constants.DRIVER_ROT;

    // Calculated Outputs (Limits Output to 12V)
    double leftOutput = rot + y ;
    double rightOutput = y - rot;

    // Assigns Each Motor's Power
    leftMaster.set(ControlMode.PercentOutput, leftOutput);
    rightMaster.set(ControlMode.PercentOutput, rightOutput);
  }

  //Path Following
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftMetersPerSecond(), getRightMetersPerSecond());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyroYawAngle(0);
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroFusedHeadingAngleDeg()));
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
    drive.feed();
=======
	//Drive Setup 
	private static Drive instance;

	public static enum DriveControlMode {
		JOYSTICK, OPEN_LOOP,
	};

	// One revolution of the wheel = Pi * D inches = 4096 ticks
	private static final double DRIVE_ENCODER_PPR = 4096.0;
	public static final double ENCODER_TICKS_TO_INCHES = DRIVE_ENCODER_PPR / (Constants.kDriveWheelDiameterInches * Math.PI);
	public static final double TRACK_WIDTH_INCHES = 23.92; // 24.56; // 26.937;

	// Motor Controllers
	private ArrayList<TalonSRX> motorControllers = new ArrayList<TalonSRX>();

	private TalonSRX leftDrive1;
	private TalonSRX leftDrive2;
	private TalonSRX rightDrive1;
	private TalonSRX rightDrive2;

	private DifferentialDrive m_drive;

	private PigeonIMU gyroPigeon;
	private double[] yprPigeon = new double[3];
	private short[] xyzPigeon = new short[3];
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;
	private Rotation2d mGyroOffset = new Rotation2d();
	protected Rotation2d mAngleAdjustment = new Rotation2d();
  private Rotation2d gyroHeading = new Rotation2d();
	
	private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;
  private DifferentialDriveOdometry odometry;

  public static final double STEER_NON_LINEARITY = 0.5;
	public static final double MOVE_NON_LINEARITY = 1.0;

	public static final double STICK_DEADBAND = 0.02;

	public static final double PITCH_THRESHOLD = 20;

	private int pitchWindowSize = 5;
	private int windowIndex = 0;
	private double pitchSum = 0;
	private double[] pitchAverageWindow = new double[pitchWindowSize];

	private int m_moveNonLinear = 0;
	private int m_steerNonLinear = -3;

	private double m_moveScale = 1.0;
	private double m_steerScale = 0.85;

	private double m_moveInput = 0.0;
	private double m_steerInput = 0.0;

	private double m_moveOutput = 0.0;
	private double m_steerOutput = 0.0;

	private double m_moveTrim = 0.0;
	private double m_steerTrim = 0.0;

	private boolean mIsBrakeMode = false;
	/**
	 * Configures talons for velocity control
	 */

	private void configureMaster(TalonSRX talon) {
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
		talon.enableVoltageCompensation(true);
		talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
		talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
		talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
		talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
		talon.configNeutralDeadband(0.04, 0);
	}

	public Drive() {
		try {

			leftDrive1 = TalonSRXFactory.createDefaultTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			leftDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID);
      
			rightDrive1 = TalonSRXFactory.createDefaultTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID,
					ENCODER_TICKS_TO_INCHES, true, FeedbackDevice.QuadEncoder);
			rightDrive2 = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID,
					RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);

			leftDrive1.setSensorPhase(false);

			leftDrive1.setInverted(true);
			leftDrive2.setInverted(true);

			rightDrive1.setSensorPhase(false);

			rightDrive1.setInverted(false);
			rightDrive2.setInverted(false);

			configureMaster(leftDrive1);
			configureMaster(rightDrive1);

			motorControllers.add(leftDrive1);
			motorControllers.add(rightDrive1);

			m_drive = new DifferentialDrive((MotorController) leftDrive1, (MotorController) rightDrive1);
			m_drive.setSafetyEnabled(false);

			gyroPigeon = new PigeonIMU(rightDrive2);
			gyroPigeon.configFactoryDefault();
			rightDrive2.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

      odometry = new DifferentialDriveOdometry(gyroHeading);

			setBrakeMode(true);
		} catch (Exception e) {
			System.err.println("An error occurred in the DriveTrain constructor");
		}
	}

	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	  public void calibrateGyro() {
		  gyroPigeon.enterCalibrationMode(CalibrationMode.Temperature, 500);
	}

	public void endGyroCalibration() {
		if (isCalibrating == true) {
			isCalibrating = false;
		}
	}

	public void setGyroOffset(double offsetDeg) {
		gyroOffsetDeg = offsetDeg;
	}

	public synchronized Rotation2d getGyroAngle() {
		return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(-getGyroAngleDeg()));
	}

	public synchronized void setGyroAngle(Rotation2d adjustment) {
		resetGyro();
		mAngleAdjustment = adjustment;
	}

	public synchronized double getGyroAngleDeg() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return -yprPigeon[0] + gyroOffsetDeg;
	}

	public synchronized double getGyroPitchAngle() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return yprPigeon[2];
	}

	public short getGyroXAccel() {
		gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		return xyzPigeon[0];
	}

	public short getGyroYAccel() {
		gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		return xyzPigeon[1];
	}

	public short getGyroZAccel() {
		gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		return xyzPigeon[2];
	}

	public boolean checkPitchAngle() {
		double pitchAngle = Math.abs(getGyroPitchAngle());
		if (pitchAngle > 10) {
			return true;
		}
		return false;
	}

	public synchronized void resetGyro() {
		gyroPigeon.setYaw(0);
		gyroPigeon.setFusedHeading(0);
	}

	public synchronized Rotation2d getHeading() {
		return Rotation2d.fromDegrees(gyroPigeon.getFusedHeading());
	}

	public synchronized void driveWithJoystick() {
		if (m_drive == null)
			return;

		m_moveInput = RobotContainer.getDriverController().getLeftYAxis();
		m_steerInput = RobotContainer.getDriverController().getRightXAxis();

		m_moveOutput = adjustForSensitivity(m_moveScale, m_moveTrim, m_moveInput, m_moveNonLinear, MOVE_NON_LINEARITY);
		m_steerOutput = adjustForSensitivity(m_steerScale, m_steerTrim, m_steerInput, m_steerNonLinear,
				STEER_NON_LINEARITY);

		double pitchAngle = updatePitchWindow();
		if (Math.abs(pitchAngle) > PITCH_THRESHOLD) {
			m_moveOutput = Math.signum(pitchAngle) * -1.0;
			m_steerOutput = 0;
		}
		m_drive.arcadeDrive(-m_moveOutput, -m_steerOutput);
	}

	private double updatePitchWindow() {
		double lastPitchAngle = pitchAverageWindow[windowIndex];
		double currentPitchAngle = getGyroPitchAngle();
		pitchAverageWindow[windowIndex] = currentPitchAngle;
		pitchSum = pitchSum - lastPitchAngle + currentPitchAngle;

		windowIndex++;
		if (windowIndex == pitchWindowSize) {
			windowIndex = 0;
		}

		return pitchSum / pitchWindowSize;
	}

	private boolean inDeadZone(double input) {
		boolean inDeadZone;
		if (Math.abs(input) < STICK_DEADBAND) {
			inDeadZone = true;
		} else {
			inDeadZone = false;
		}
		return inDeadZone;
	}

	public double adjustForSensitivity(double scale, double trim, double steer, int nonLinearFactor,
			double wheelNonLinearity) {
		if (inDeadZone(steer))
			return 0;

		steer += trim;
		steer *= scale;
		steer = limitValue(steer);

		int iterations = Math.abs(nonLinearFactor);
		for (int i = 0; i < iterations; i++) {
			if (nonLinearFactor > 0) {
				steer = nonlinearStickCalcPositive(steer, wheelNonLinearity);
			} else {
				steer = nonlinearStickCalcNegative(steer, wheelNonLinearity);
			}
		}
		return steer;
	}

	private double limitValue(double value) {
		if (value > 1.0) {
			value = 1.0;
		} else if (value < -1.0) {
			value = -1.0;
		}
		return value;
	}

	private double nonlinearStickCalcPositive(double steer, double steerNonLinearity) {
		return Math.sin(Math.PI / 2.0 * steerNonLinearity * steer) / Math.sin(Math.PI / 2.0 * steerNonLinearity);
	}

	private double nonlinearStickCalcNegative(double steer, double steerNonLinearity) {
		return Math.asin(steerNonLinearity * steer) / Math.asin(steerNonLinearity);
	}

	public synchronized DriveControlMode getControlMode() {
		return driveControlMode;
	}

	public synchronized void setControlMode(DriveControlMode controlMode) {
		this.driveControlMode = controlMode;
	}

  //Returns left sensors velocity in ticks per 100ms
  public double getLeftVelocityNativeUnits() {
    return leftDrive1.getSelectedSensorVelocity(0);
  }

  //Returns right sensors velocity in ticks per 100ms
  public double getRightVelocityNativeUnits() {
    return leftDrive1.getSelectedSensorVelocity(0);
  }

  //Returns left sensors position in ticks
  public double getLeftSensorPosition(){
    return leftDrive1.getSelectedSensorPosition(0);
  }

  //Returns right sensors position in ticks
  public double getRightSensorPosition(){
    return leftDrive1.getSelectedSensorPosition(0);
  }

  //Takes that times the wheel has rotated * by the circumference of the wheel to get its distance traveled in inches
  public static double rotationsToInches(double rotations) {
    return rotations * (Constants.kWheelDiameterInches * Math.PI);
  }

  //Takes inches and converts it to meters using units class
  public static double inchesToMeters(double inches){
    return Units.inchesToMeters(inches);
  }

  //Takes the sensor velocity of an encoder * by 10 to get ticks per second / the encoder PPR to get encoder rotations
  //per second and then uses the rotations to inches functions to get inches per second
  private static double ticksPer100msToInchesPerSec(double ticks_100ms) {
    return rotationsToInches(ticks_100ms * 10.0 / 2048);
  }

  //Returns left inches per second using the sensor velocity and the ticksToInches conversion method
  public double getLeftInchesPerSecond(){
    return ticksPer100msToInchesPerSec(getLeftVelocityNativeUnits()) / Constants.kEncoderRotationToWheelRotationRatio;
  }

  //Returns right inches per second using the sensor velocity and the ticksToInches conversion method
  public double getRightInchesPerSecond(){
    return ticksPer100msToInchesPerSec(getRightVelocityNativeUnits()) / Constants.kEncoderRotationToWheelRotationRatio;
  }

  //Returns left meters per second using inchesPerSecond calculation and inchesToMeters method
  public double getLeftMetersPerSecond(){
    return inchesToMeters(getLeftInchesPerSecond());
  }

  //Returns right meters per second using inchesPerSecond calculation and inchesToMeters method
  public double getRightMetersPerSecond(){
    return inchesToMeters(getRightInchesPerSecond());
  }

  //Sensors positions in ticks / Pulses per Revolution of the Encoder = Encoder Rotations (If ratio is 1:1)
  public double getLeftEncoderRotations() {
    return getLeftSensorPosition() / Constants.kDriveEncoderPPR;
  }

  public double getRightEncoderRotations() {
    return getRightSensorPosition() / Constants.kDriveEncoderPPR;
  }

  //Wheel Rotations = Encoder Rotations (If ratio is 1:1)
  public double getLeftWheelRotations() {
    return getLeftEncoderRotations() / Constants.kEncoderRotationToWheelRotationRatio;
  }

  public double getRightWheelRotations() {
    return getRightEncoderRotations() / Constants.kEncoderRotationToWheelRotationRatio;
  }

  //Returns left distance traveled in inches by taking wheel rotations and converting it to inches
  public double getLeftWheelDistanceInches() {
    return rotationsToInches(getLeftWheelRotations());
  }

  //Returns right distance traveled in inches by taking wheel rotations and converting it to inches
  public double getRightWheelDistanceInches() {
    return rotationsToInches(getRightWheelRotations());
  }

  //Returns left distance traveled in meters using calculated inches distances and inchesToMeters conversion
  public double getLeftWheelDistanceMeters() {
    return inchesToMeters(getLeftWheelDistanceInches());
  }

  //Returns right distance traveled in meters using calculated inches distances and inchesToMeters conversion
  public double getRightWheelDistanceMeters(){
    return inchesToMeters(getRightWheelDistanceInches());
  }

	public boolean isBrakeMode() {
		return mIsBrakeMode;
	}

	public synchronized void setBrakeMode(boolean on) {
		if (mIsBrakeMode != on) {
			mIsBrakeMode = on;
			rightDrive1.setNeutralMode(NeutralMode.Brake);
			rightDrive2.setNeutralMode(NeutralMode.Brake);
			leftDrive1.setNeutralMode(NeutralMode.Brake);
			leftDrive2.setNeutralMode(NeutralMode.Brake);
		}
	}

	public void updateStatus(RobotState state) {
		if (RobotState.isTest()) {
			try {
			} catch (Exception e) {
			}
		} else if (RobotState.isTeleop()) {
			if (getHeading() != null) {
				SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
			}
		}
	}

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    ((MotorController) leftDrive1).setVoltage(leftVolts);
    ((MotorController) leftDrive2).setVoltage(rightVolts);
>>>>>>> Stashed changes
  }

  public void periodic() {
<<<<<<< Updated upstream
    synchronized (Drive.this){
      DriveControlMode currentControlMode = getControlMode();
      switch (currentControlMode){
        case JOYSTICK:
          driveWithJoystick();
          break;
        case PATH_FOLLOWING:
          odometry.update(Rotation2d.fromDegrees(getGyroFusedHeadingAngleDeg()),
                  getLeftWheelDistanceMeters(),getRightWheelDistanceMeters());
          break;
        default:
          System.out.println("Unknown drive control mode: " + currentControlMode);
      }
    }

    // SmartDashboard.putNumber("Left Distance Inches: ", getLeftWheelDistanceInches());
    // SmartDashboard.putNumber("Right Distance Inches: ", getRightWheelDistanceInches());

    // SmartDashboard.putNumber("Left Distance Meters: ", getLeftWheelDistanceMeters());
    // SmartDashboard.putNumber("Right Distance Meters: ", getRightWheelDistanceMeters());

    // SmartDashboard.putNumber("Heading: ", getGyroFusedHeadingAngleDeg());

  }
}
=======
    synchronized (Drive.this) {
			DriveControlMode currentControlMode = getControlMode();
				switch (currentControlMode) {
				case OPEN_LOOP:
          driveWithJoystick();
					break;
				default:
          odometry.update(Rotation2d.fromDegrees(getGyroAngleDeg()),
            getLeftWheelDistanceMeters(), getRightWheelDistanceMeters());
					System.out.println("Unknown drive control mode: " + currentControlMode);
					break;
				}
			}
    }
  }
>>>>>>> Stashed changes
