package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints.kLiftPosition;
import frc.robot.subsystems.CoralArm;

public class CommandPositionCoral extends Command {
    private final CoralArm m_coralArm;
    private final kLiftPosition m_coralPosition;

    public CommandPositionCoral(CoralArm coralArm, kLiftPosition position) {
        m_coralArm = coralArm;
        m_coralPosition = position;
        addRequirements(m_coralArm);

    }

    @Override 
    public void execute() {
        m_coralArm.CoralArmPosition(m_coralPosition);
        SmartDashboard.putNumber("Coral Commanded", m_coralPosition.CoralPoseDeg);
        if (m_coralArm.getCoralPosition(m_coralPosition)) {
            isFinished();
        }
    }
    
    @Override public boolean isFinished() {
        return true;
    }
}
