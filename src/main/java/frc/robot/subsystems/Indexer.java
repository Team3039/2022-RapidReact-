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

    private final TalonFX firstStage;
    private final TalonFX secondStage;

    private final DigitalInput secondStageGate;
    private final DigitalInput firstStageGate;

    private IndexerState state = IndexerState.IDLE;

    private boolean hasOneBall;
    private boolean hasTwoBalls;
    public boolean isFeeding;
  

    public Indexer() {
        firstStage = new TalonFX(Constants.RobotMap.firstStage);
        secondStage = new TalonFX(Constants.RobotMap.secondStage);

        firstStageGate = new DigitalInput(Constants.RobotMap.firstStageGate);
        secondStageGate = new DigitalInput(Constants.RobotMap.secondStageGate);

        firstStage.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 125);
        secondStage.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 125);

        firstStage.configVoltageCompSaturation(12.0);
        firstStage.enableVoltageCompensation(true);
        secondStage.configVoltageCompSaturation(12.0);
        secondStage.enableVoltageCompensation(true);

        firstStage.setNeutralMode(NeutralMode.Brake);
        secondStage.setNeutralMode(NeutralMode.Brake);
    }

    public synchronized IndexerState getState() {
        return state;
    }

    public void setState(IndexerState wanted_state) {
        state = wanted_state;
    }

    public void setOpenLoop(double firstStageOuput, double secondStageOutput) {
       firstStage.set(ControlMode.PercentOutput, firstStageOuput);
       secondStage.set(ControlMode.PercentOutput, secondStageOutput);
    }

    public void stop() {
        setOpenLoop(0, 0);
    }

    //Index Help to Index Balls
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
        hasOneBall = !secondStageGate.get();
        hasTwoBalls = hasOneBall && !firstStageGate.get();

        switch (state) {
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
                firstStage.set(ControlMode.Disabled, 0);
                secondStage.set(ControlMode.Disabled, 0);
                break;
            case UNJAMMING:
                    setOpenLoop(-0.25, -0.25);
                break;
            default:
                System.out.println("Fell through on Indexer states!");
            }
    }
}