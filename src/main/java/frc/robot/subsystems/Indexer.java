package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeState;

public class Indexer extends SubsystemBase {

    public enum IndexerState {
        IDLE, SHOOTING, INDEXING, CLIMBING, HELLA_ZOOMING, UNJAMMING,
    }

    private final TalonFX mFirstStageMaster;
    private final TalonFX mSecondStageMaster;

    private final DigitalInput mFirstStageGate;
    private final DigitalInput mSecondStageGate;

    private IndexerState mState = IndexerState.IDLE;

    private boolean hasOneBall;
    private boolean hasTwoBalls;
    public boolean isFeeding;
  

    public Indexer() {
        mFirstStageMaster = new TalonFX(Constants.Ports.FIRST_STAGE);
        mSecondStageMaster = new TalonFX(Constants.Ports.SECOND_STAGE);

        mFirstStageGate = new DigitalInput(Constants.Ports.FIRST_STAGE_GATE);
        mSecondStageGate = new DigitalInput(Constants.Ports.SECOND_STAGE_GATE);

        mFirstStageMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 125);
        mSecondStageMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 125);

        mFirstStageMaster.configVoltageCompSaturation(12.0);
        mFirstStageMaster.enableVoltageCompensation(true);
        mSecondStageMaster.configVoltageCompSaturation(12.0);
        mSecondStageMaster.enableVoltageCompensation(true);

        mFirstStageMaster.setNeutralMode(NeutralMode.Brake);
        mSecondStageMaster.setNeutralMode(NeutralMode.Brake);
    }

    public synchronized IndexerState getState() {
        return mState;
    }

    public void setState(IndexerState wanted_state) {
        mState = wanted_state;
    }

    public void setOpenLoop(double firstStageOuput, double secondStageOutput) {
       mFirstStageMaster.set(ControlMode.PercentOutput, firstStageOuput);
       mSecondStageMaster.set(ControlMode.PercentOutput, secondStageOutput);
    }

    public void stop() {
        setOpenLoop(0, 0);
    }

    public void indexIntake() {
        switch (getState()) {
            case INDEXING:
                if (!hasTwoBalls) {
                    Intake.getInstance().setState(IntakeState.INTAKING);
                }
                else {
                    Intake.getInstance().setState(IntakeState.IDLE);
                }
                break;
            case SHOOTING:
                Intake.getInstance().setState(IntakeState.INTAKING);
            default:
                break;
        } 
    }

    @Override
    public void periodic() {
        hasOneBall = !mSecondStageGate.get();
        hasTwoBalls = hasOneBall && !mFirstStageGate.get();

        switch (mState) {
            case HELLA_ZOOMING:
                System.out.println("Zoomies");
            case IDLE:
                setOpenLoop(0, 0);
                break;
            case SHOOTING:
                isFeeding = true;
                setOpenLoop(0.5, 0.5);
                indexIntake();
                break;
            case INDEXING:
                isFeeding = false;
                if (!hasOneBall && !hasTwoBalls) {
                    setOpenLoop(0.75, 0.75); 
                }
                else if (hasOneBall && !hasTwoBalls) {
                    setOpenLoop(0.25, 0);
                }
                else if (hasTwoBalls) {
                    setOpenLoop(0, 0);
                }
                indexIntake();
                break;
            case CLIMBING:
                mFirstStageMaster.set(ControlMode.Disabled, 0);
                mSecondStageMaster.set(ControlMode.Disabled, 0);
                break;
            case UNJAMMING:
                    setOpenLoop(-0.25, -0.25);
                break;
            default:
                System.out.println("Fell through on Indexer states!");
            }
    }
}