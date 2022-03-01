package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.IntakeState;

public class Indexer extends SubsystemBase {
    private static Indexer mInstance = null;
    
    private static final double kZoomingVelocity = 80.;
    private static final double kPassiveIndexingVelocity = 80.0;

    private static final double kJamCurrent = 150.0;
    private double mLastCurrentSpikeTime = 0.0;
    private static final double kCurrentIgnoreTime = 1.0; 

    public enum IndexerState {
        IDLE, PASSIVE_INDEXING,  ACTIVE_INDEXING, CLIMBING, HELLA_ZOOMING, UNJAMMING
    }

    private final TalonFX mGripper;
    private final TalonFX mFeeder;

    private final DigitalInput mFeederGate;
    private final DigitalInput mGripperGate;

    private IndexerState mState = IndexerState.IDLE;
    private boolean mBackwards = false;

    private boolean hasOneBall;
    private boolean hasTwoBalls;
    private boolean isJamming;
    public boolean isFeeding;

    public Indexer() {
        mFeeder = new TalonFX(Constants.RobotMap.hopperFeeder);
        mGripper = new TalonFX(Constants.RobotMap.hopperGripper);

        mFeederGate = new DigitalInput(Constants.RobotMap.hopperFeederGate);
        mGripperGate = new DigitalInput(Constants.RobotMap.hopperGripperGate);

        mFeeder.changeMotionControlFramePeriod(255);
        mFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 125);
        mFeeder.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 125);

        mFeeder.configVoltageCompSaturation(12.0);
        mFeeder.enableVoltageCompensation(true);
    }

    public synchronized static Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    public synchronized IndexerState getState() {
        return mState;
    }

    public void setState(IndexerState wanted_state) {
        final IndexerState prev_state = mState;
        mState = wanted_state;
    }

    public void gateCheck() {
        hasOneBall = !mFeederGate.get();
        hasTwoBalls = hasOneBall && !mGripperGate.get();
    }

    public void jammingCheck() {
        isJamming = (mFeeder.getStatorCurrent() > Constants.Indexer.JAMMING_CURRENT_THRESHOLD && hasOneBall);
    }

    public synchronized void setOpenLoop(
      double gripperOutput,
      double feederOutput) {
       mGripper.set(ControlMode.PercentOutput, gripperOutput);
       mFeeder.set(ControlMode.PercentOutput, feederOutput);
    }

    public void stop() {
        setOpenLoop(0, 0);
        mFeeder.setNeutralMode(NeutralMode.Brake);
    }

    public void indexIntake() {
            switch (Indexer.getInstance().getState()) {
                case ACTIVE_INDEXING:
                    if (Intake.getInstance().wrongBallCheck())
                        Intake.getInstance().setState(IntakeState.REJECTION);
                    else {
                        if (!this.hasTwoBalls)
                            Intake.getInstance().setState(IntakeState.INTAKING);
                        else
                            Intake.getInstance().setState(IntakeState.IDLE);
                    }
                    case PASSIVE_INDEXING:
                    Intake.getInstance().setState(IntakeState.INTAKING);
                default:
                    break;
            }
        
    }

    public synchronized void setBackwardsMode(boolean backwards) {
        mBackwards = backwards;
    }

    @Override
    public void periodic() {
        gateCheck();
        jammingCheck();
        if (isJamming) 
            setState(IndexerState.UNJAMMING);

        switch (mState) {
            case IDLE:
                setOpenLoop(0, 0);
                break;
            case PASSIVE_INDEXING:
                isFeeding = true;
                setOpenLoop(0.5, 0.5);
                indexIntake();
                break;
            case ACTIVE_INDEXING:
                isFeeding = false;
                if (!hasOneBall && !hasTwoBalls)
                    setOpenLoop(0.75, 0.75);
                else if (hasOneBall && !hasTwoBalls)
                    setOpenLoop(0.25, 0);
                else if (hasTwoBalls)
                    setOpenLoop(0, 0);
                indexIntake();
                break;
            case CLIMBING:
                mGripper.set(ControlMode.Disabled, 0);
                mFeeder.set(ControlMode.Disabled, 0);
                break;
            case UNJAMMING:
                if(isJamming) {
                    setOpenLoop(-0.25, -0.25);
                    Intake.getInstance().setState(IntakeState.REJECTION);
                }
                else {
                    setOpenLoop(0, 0);
                    Intake.getInstance().setState(IntakeState.IDLE);
                }
            default:
                System.out.println("Fell through on Indexer states!");
            }
    }
}