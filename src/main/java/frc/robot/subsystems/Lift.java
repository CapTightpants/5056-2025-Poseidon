package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.Setpoints.kHarpoonPosition;
import frc.robot.Constants.Setpoints.kLiftPosition;

public class Lift extends SubsystemBase {
    private final SparkMax m_liftLeftSpark;
    private final SparkMax m_liftRightSpark;
    private final SparkClosedLoopController m_liftClosedLoopController;

    public Lift(int LeftLiftCanId, int RightLiftCanId) {
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
        return (MathUtil.isNear(position.LiftPose, m_liftLeftSpark.getAbsoluteEncoder().getPosition(), .08));
    }
    
    /**
     * Sets the lift's desired point
     * 
     * @param setpoint The desired setpoint from 0 to 10.
     */
    public void setLiftPosition(kLiftPosition targetPosition) {
        m_liftClosedLoopController.setReference(targetPosition.LiftPose, ControlType.kPosition);
    }
}
