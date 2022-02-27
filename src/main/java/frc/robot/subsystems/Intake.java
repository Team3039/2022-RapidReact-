package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    private static double kIntakingVoltage = 12.0;
    private static double kIdleVoltage = 0;

    private static Intake mInstance;

    private Solenoid mDeploySolenoid;

    public enum IntakeState {
        IDLE, INTAKING, REJECTION, INDEXING,
    }

    private IntakeState mState = IntakeState.IDLE;

    private final TalonFX mMaster;

    ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    Color detectedColor;
    boolean isWrongBall;

    public Intake() {
        mMaster = new TalonFX(0);
        mMaster.changeMotionControlFramePeriod(255);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        mMaster.setInverted(true);
        mDeploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    }

    public synchronized static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    public synchronized IntakeState getState() {
        return mState;
    }

    public synchronized void setOpenLoop(double percentage) {
         mMaster.set(ControlMode.PercentOutput, percentage);
    }

    public double getVoltage() {
        return mMaster.getMotorOutputVoltage();
    }

    public void setState(IntakeState wanted_state) {
        mState = wanted_state;
        }

    //Checks if the color sensor detects the opponent's ball
    public boolean wrongBallCheck() {
        if (!Robot.isRedAlliance && detectedColor.red >= 0.4) 
            return true;
        else if (Robot.isRedAlliance && detectedColor.blue >= 0.4) 
            return true;
        
         return false;
       }
        
    

    @Override
    public void periodic() {
        detectedColor = colorSensor.getColor();

      synchronized (Intake.this) {
          switch(getState()) {
            case IDLE:
                mDeploySolenoid.set(false);
                mMaster.set(ControlMode.PercentOutput, 0.0);
                break;
            case INTAKING:
                if (!Indexer.getInstance().isFeeding)
                    mDeploySolenoid.set(true);
                mMaster.set(ControlMode.PercentOutput, 0.75);
                break;
            case REJECTION:
                if (!Indexer.getInstance().isFeeding)
                    mDeploySolenoid.set(true);
                mMaster.set(ControlMode.PercentOutput, -0.25);
                break;
            default:
                break;
          }
      }
    }
}