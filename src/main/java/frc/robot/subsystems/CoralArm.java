package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.Setpoints.kLiftPosition;

public class CoralArm extends SubsystemBase {
  private final SparkMax m_armSpark;
    private final SparkMax m_intakeSpark;
    private final DigitalInput m_intakeLimitSwitch;
    private final SparkClosedLoopController m_armClosedLoopController;
    private final ShuffleboardTab m_sensorsTab = Shuffleboard.getTab("Sensors");
    private final GenericEntry armEntry = m_sensorsTab.add("Coral Arm Encoder", 0).getEntry();

    public CoralArm(int intakeCANId, int armCANId, int intakeLimitSwitchID) {
        m_intakeSpark = new SparkMax(intakeCANId, MotorType.kBrushless);
        m_armSpark = new SparkMax(armCANId, MotorType.kBrushless);
        m_intakeLimitSwitch = new DigitalInput(intakeLimitSwitchID);


        // m_intakeSpark.configure(Configs.DefaultNeo.neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_armSpark.configure(Configs.DefaultNeo.neoArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_armClosedLoopController = m_armSpark.getClosedLoopController();
    } 


     /**
     * Tell the manipulator to gawk gawk the coral in.
     * 
     * @param setpoint The desired value from 0 to 1.
     */
    public void IntakeCoral(double setpoint) {
        m_intakeSpark.set(m_intakeLimitSwitch.get() && setpoint < 0
        ? setpoint
        : setpoint
    );
    }

    /**
     * Checks to verify that the system is in the desired position.
     * @param position The position to check the system against.
     * @return Returns true if the system is in position.
     */
    public boolean getCoralPosition(kLiftPosition position) {
        return (MathUtil.isNear(position.CoralPoseDeg, m_armSpark.getAbsoluteEncoder().getPosition(), .17));
    }
    
    /**
     * Around around around the world
     * @param targetPosition The target in degrees for arm, "good enough" ~ Jesse
     */
    public void CoralArmPosition(kLiftPosition targetPosition) {
        m_armClosedLoopController.setReference(targetPosition.CoralPoseDeg, ControlType.kPosition);
   }

   @Override
   public void periodic() {
    armEntry.setDouble(m_armSpark.getEncoder().getPosition());
   }
}
