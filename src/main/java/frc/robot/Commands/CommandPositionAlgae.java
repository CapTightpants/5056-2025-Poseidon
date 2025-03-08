package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints.kLiftPosition;
import frc.robot.subsystems.AlgaeArm;

public class CommandPositionAlgae extends Command {
    private final AlgaeArm m_algaeArm;
    private final kLiftPosition m_algaePosition;

    public CommandPositionAlgae(AlgaeArm algaeArm, kLiftPosition position) {
        m_algaeArm = algaeArm;
        m_algaePosition = position;
        addRequirements(m_algaeArm);
    }

    @Override 
    public void execute() {
        m_algaeArm.AlgaeArmPosition(m_algaePosition);
        SmartDashboard.putNumber("Algae Commanded", m_algaePosition.AlgaePoseDeg);
    }  

    @Override public boolean isFinished() {
        return m_algaeArm.getAlgaePosition(m_algaePosition);
    }

}
