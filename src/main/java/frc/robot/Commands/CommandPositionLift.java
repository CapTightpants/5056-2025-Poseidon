package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints.kLiftPosition;
import frc.robot.subsystems.Lift;

public class CommandPositionLift extends Command {
    private final Lift m_lift;
    private final kLiftPosition m_position;


    public CommandPositionLift(Lift lift, kLiftPosition liftPosition) {
        m_lift = lift;
        m_position = liftPosition;
        addRequirements(m_lift);

    }

    @Override 
    public void execute() {
        m_lift.setLiftPosition(m_position);
    }

    @Override public boolean isFinished() {
        return m_lift.getLiftPosition(m_position);
    }
}
