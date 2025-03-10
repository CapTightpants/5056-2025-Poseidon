package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.Setpoints.kLiftPosition;

public class AlgaeArm extends SubsystemBase {
  private final SparkMax m_armSpark;
    private final VictorSPX m_intakeVictor;
    private final SparkClosedLoopController m_armClosedLoopController;
    private final ShuffleboardTab m_sensorsDB;
    public AlgaeArm(int intakeCANId, int armCANId) {
        m_intakeVictor = new VictorSPX(intakeCANId);
        m_armSpark = new SparkMax(armCANId, MotorType.kBrushless);
        m_sensorsDB = Shuffleboard.getTab("Sensors");



        // m_intakeVictor.configure(Configs.DefaultNeo.neoConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_armSpark.configure(Configs.DefaultNeo.neoArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_armClosedLoopController = m_armSpark.getClosedLoopController();
        m_sensorsDB.add("AlgaeArm", 0);

    } 


     /**
     * Tell the manipulator to gawk gawk the coral in.
     * 
     * @param setpoint The desired value from 0 to 1.
     */
    public void IntakeAlgae(double setpoint) {
        m_intakeVictor.set(ControlMode.PercentOutput, setpoint);;
    }
    
    /**
     * Checks to verify that the system is in the desired position.
     * @param position The position to check the system against.
     * @return Returns true if the system is in position.
     */
    public boolean getAlgaePosition(kLiftPosition position) {
        return (MathUtil.isNear(position.AlgaePoseDeg, m_armSpark.getAbsoluteEncoder().getPosition(), .1));
    }

    /**
     * Around around around the world
     * @param targetPosition The target in degrees for arm, "good enough" ~ Jesse
     */
    public void AlgaeArmPosition(kLiftPosition targetPosition) {
        m_armClosedLoopController.setReference(targetPosition.AlgaePoseDeg, ControlType.kPosition);
   }

   @Override
   public void periodic() {
    SmartDashboard.putNumber("Algae Arm", m_armSpark.getAbsoluteEncoder().getPosition());
   }
}
