package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Harpoon extends SubsystemBase {
    private final SparkMax m_reelSpark;
    private final SparkMax m_deploySpark;
    private final SparkClosedLoopController m_deployClosedLoopController;

    public Harpoon(int reelCANId, int deployCANId) {
        m_reelSpark = new SparkMax(reelCANId, MotorType.kBrushless);
        m_deploySpark = new SparkMax(deployCANId, MotorType.kBrushless);


        // m_reelSpark.configure(Configs.DefaultNeo.neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // m_deploySpark.configure(Configs.DefaultNeo.neoHarpoonConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_deployClosedLoopController = m_deploySpark.getClosedLoopController();
    }

    /**
     * Tell the Harpoon to go to a certain position
     * 
     * @param setpoint The desired value in Degrees
     */
    public void OrientHarpoon(double setpoint) {
       m_deployClosedLoopController.setReference(setpoint, ControlType.kPosition);
    }

    /**
     * Tell Harpoon to retract/extend; will not go in reverse if the limit switch is triggered.
     * @param setpoint The desired value, from -1 to 1
     * @param limitSwitch The reference to the Harpoon's limit switch
     */
   public void HarpoonYeet(double setpoint) {
        m_reelSpark.set(setpoint);
   }
}