package frc.robot.Commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints.kLiftPosition;
import frc.robot.subsystems.CoralArm;

public class CommandPositionCoral extends Command {
    private final CoralArm m_coralArm;
    private final kLiftPosition m_coralPosition;
    private final ShuffleboardTab m_sensorsTab = Shuffleboard.getTab("Sensors");

    public CommandPositionCoral(CoralArm coralArm, kLiftPosition position) {
        m_coralArm = coralArm;
        m_coralPosition = position;
        addRequirements(m_coralArm);

    }

    @Override 
    public void execute() {
        m_coralArm.CoralArmPosition(m_coralPosition);
    }
    
    @Override 
    public boolean isFinished() {
        return m_coralArm.getCoralPosition(m_coralPosition);
    }
}
