package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Indexer.IndexerState;

public class Intake extends SubsystemBase {

    private static Intake INSTANCE = new Intake();

    // private Solenoid mDeploySolenoid;

    public enum IntakeState {
        IDLE, INTAKING, OUTTAKING, INDEXING,
    }

    private IntakeState mState = IntakeState.IDLE;

    // private final CANSparkMax mMaster;

 //   ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    Color detectedColor;

    public Intake() {
        // mMaster = new CANSparkMax(Constants.RobotMap.intake, MotorType.kBrushless);
        // mMaster.setIdleMode(IdleMode.kCoast);
        // mMaster.setInverted(true);
        // mDeploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RobotMap.intakeDeploy);
    }

    public synchronized static Intake getInstance() {
        return INSTANCE;
    }

    public synchronized IntakeState getState() {
        return mState;
    }

    public synchronized void setOpenLoop(double percentage) {
        // mMaster.set(percentage);
    }

    public void setState(IntakeState wanted_state) {
        mState = wanted_state;
    }

    public boolean isWrongBall() {
        if (!Robot.isRedAlliance && detectedColor.red >= 0.4)
            return true;
        else if (Robot.isRedAlliance && detectedColor.blue >= 0.4)
            return true;
        return false;
    }

    @Override
    public void periodic() {
        detectedColor = Color.kRed;

        synchronized (Intake.this) {
            switch (getState()) {
                case IDLE:
                    // mDeploySolenoid.set(false);
                    setOpenLoop(0);
                    break;
                case INTAKING:
                    if (Indexer.getInstance().getState().equals(IndexerState.ACTIVE_INDEXING))
                        // mDeploySolenoid.set(true);
                    setOpenLoop(.35);
                    break;
                case OUTTAKING:
                    if (!Indexer.getInstance().getState().equals(IndexerState.ACTIVE_INDEXING))
                        // mDeploySolenoid.set(true);
                    setOpenLoop(-.25);
                    break;
                default:
                    break;
            }
        }
    }
}