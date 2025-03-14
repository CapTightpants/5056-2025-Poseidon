package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Commands.CommandToggleThrottle;
import frc.robot.Constants.Setpoints.kLiftPosition;
import frc.robot.Vars.Throttles;

public class Lift extends SubsystemBase {
    private final SparkMax m_liftLeftSpark;
    private final SparkMax m_liftRightSpark;
    private final SparkClosedLoopController m_liftClosedLoopController;
    private final ShuffleboardTab m_sensorsTab = Shuffleboard.getTab("Sensors");
    private final GenericEntry liftEntry = m_sensorsTab.add("Lift Encoder", 0).getEntry();
    private Command setCreepMode;
    private Command setNormalMode;

    public Lift(int LeftLiftCanId, int RightLiftCanId) {
        setCreepMode = new CommandToggleThrottle(Throttles.kCreep);
        setNormalMode = new CommandToggleThrottle(Throttles.kNormal);

        m_liftLeftSpark = new SparkMax(LeftLiftCanId, MotorType.kBrushless);
        m_liftRightSpark = new SparkMax(RightLiftCanId, MotorType.kBrushless);
        m_liftClosedLoopController = m_liftLeftSpark.getClosedLoopController();

        m_liftLeftSpark.configure(
            Configs.DefaultNeo.liftLeftConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        m_liftRightSpark.configure(
            Configs.DefaultNeo.liftRightConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }


    /**
     * Checks to verify that the system is in the desired position.
     * @param position The position to check the system against.
     * @return Returns true if the system is in position.
     */
    public boolean getLiftPosition(kLiftPosition position) {
        return (MathUtil.isNear(position.LiftPose, m_liftLeftSpark.getAbsoluteEncoder().getPosition(), .12));
    }
    
    /**
     * Sets the lift's desired point
     * 
     * @param setpoint The desired setpoint from 0 to 10.
     */
    public void setLiftPosition(kLiftPosition targetPosition) {
        m_liftClosedLoopController.setReference(targetPosition.LiftPose, ControlType.kPosition);
        switch (targetPosition) {
            case Stage1, Stage2, Stage3, Algae2:
                setCreepMode.schedule();
                break;
            case Start, Station, Processor:
                setNormalMode.schedule();
                break;
            default:
                break;
        }
    }

    @Override
    public void periodic() {
        liftEntry.setDouble(m_liftLeftSpark.getEncoder().getPosition());
    }
}
