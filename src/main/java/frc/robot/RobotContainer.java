// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.networktables.DoubleEntry;
// import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.*;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
// import frc.robot.Constants.Setpoints;
import frc.robot.Constants.SystemConstants;
import frc.robot.Vars.Tuning.kAimingPositions;
import frc.robot.Constants.Setpoints.kLiftPosition;
import frc.robot.Vars.Throttles;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.ReefLimelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Variables
  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems
  private final Pigeon2 m_gyro = new Pigeon2(SystemConstants.kGyroCanId);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro);
  private final Lift m_lift = new Lift(SystemConstants.kLeftLiftCanId, SystemConstants.kRightLiftCanId);
  private final CoralArm m_coralArm = new CoralArm(SystemConstants.kCoralIntakeCanId, SystemConstants.kCoralArmCanId, SystemConstants.kCoralLimitDIO);
//   private final AlgaeArm= new AlgaeArm(SystemConstants.kAlgaeIntakeCanId, SystemConstants.kAlgaeArmCanId);
  private final ReefLimelight m_reeReefLimelight = new ReefLimelight(m_robotDrive, m_gyro);
  
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverCommander = new CommandXboxController(OIConstants.kDriverControllerPort);
  
  // The operator's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  CommandXboxController m_operatorCommander = new CommandXboxController(OIConstants.kOperatorControllerPort);
  
  // Commands
    private final Command m_CommandMultiStart = new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Start);
    private final Command m_CommandMultiStage1 = new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Stage1);
    private final Command m_CommandMultiStage2 = new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Stage2);
    private final Command m_CommandMultiStage3 = new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Stage3);
    private final Command m_CommandMultiStation = new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Station);
    private final Command m_CommandMultiAlgae2 = new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Algae2);
    private final Command m_CommandMultiProcessor = new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Processor);
    private final Command m_CommandCoralIntake = new CommandCoralIntake(m_coralArm);
    private final Command m_CommandCoralOuttake = new CommandCoralOuttake(m_coralArm);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Register Named Commands
    NamedCommands.registerCommand("intake coral", m_CommandCoralIntake);
    NamedCommands.registerCommand("outtake coral", m_CommandCoralOuttake);
    NamedCommands.registerCommand("coral arm start", new CommandPositionCoral(m_coralArm, kLiftPosition.Start));
    NamedCommands.registerCommand("coral arm stage1", new CommandPositionCoral(m_coralArm, kLiftPosition.Stage1));
    NamedCommands.registerCommand("coral arm stage2", new CommandPositionCoral(m_coralArm, kLiftPosition.Stage2));
    NamedCommands.registerCommand("coral arm stage3", new CommandPositionCoral(m_coralArm, kLiftPosition.Stage3));
    NamedCommands.registerCommand("auto aim reef",
        new RunCommand(
            () -> m_reeReefLimelight.alignRobot(
                kAimingPositions.CoralLeft, false),
            m_reeReefLimelight)
        );

    // Register Auto Triggers
    new EventTrigger("station").onTrue(m_CommandMultiStation);
    new EventTrigger("start").onTrue(m_CommandMultiStart);
    new EventTrigger("stage1").onTrue(m_CommandMultiStage1);
    new EventTrigger("stage2").onTrue(m_CommandMultiStage2);
    new EventTrigger("align coral left").whileTrue(
        new RunCommand(
        () -> m_reeReefLimelight.alignRobot(
            kAimingPositions.CoralLeft, false),
        m_reeReefLimelight)
        );
    new EventTrigger("align coral right").whileTrue(
        new RunCommand(
        () -> m_reeReefLimelight.alignRobot(
            kAimingPositions.CoralRight, false),
        m_reeReefLimelight)
        );
    new EventTrigger("align intake").whileTrue(
        new RunCommand(
        () -> m_reeReefLimelight.alignRobot(
            kAimingPositions.CoralRight, false),
        m_reeReefLimelight)
        );

    // Configure SmartDashboard
    ShuffleboardTab commandsTab = Shuffleboard.getTab("Commands");
    // commandsTab.add("Algae Arm Position: Station", new CommandPositionAlgae kLiftPosition.Station));
    // commandsTab.add("Algae Arm Position: Stage1", new CommandPositionAlgae kLiftPosition.Stage1));
    // commandsTab.add("Algae Arm Position: Stage2", new CommandPositionAlgae kLiftPosition.Stage2));
    // commandsTab.add("Algae Arm Position: Stage3", new CommandPositionAlgae kLiftPosition.Stage3));
    // commandsTab.add("Algae Arm Position: Start",  new CommandPositionAlgae kLiftPosition.Start));
    commandsTab.add("Coral Arm Position: Station", new CommandPositionCoral(m_coralArm, kLiftPosition.Station));
    commandsTab.add("Coral Arm Position: Stage1", new CommandPositionCoral(m_coralArm, kLiftPosition.Stage1));
    commandsTab.add("Coral Arm Position: Stage2", new CommandPositionCoral(m_coralArm, kLiftPosition.Stage2));
    commandsTab.add("Coral Arm Position: Stage3", new CommandPositionCoral(m_coralArm, kLiftPosition.Stage3));
    commandsTab.add("Coral Arm Position: Start",  new CommandPositionCoral(m_coralArm, kLiftPosition.Start));
    commandsTab.add("Lift Position: Station",   new CommandPositionLift(m_lift, kLiftPosition.Station));
    commandsTab.add("Lift Position: Stage1", new CommandPositionLift(m_lift, kLiftPosition.Stage1));
    commandsTab.add("Lift Position: Stage2", new CommandPositionLift(m_lift, kLiftPosition.Stage3));
    commandsTab.add("Lift Position: Stage3", new CommandPositionLift(m_lift, kLiftPosition.Stage3));
    commandsTab.add("Lift Position: Start",  new CommandPositionLift(m_lift, kLiftPosition.Start));
    
    final ShuffleboardTab tab = Shuffleboard.getTab("Tuning");
        tab.add("Algae Station", kLiftPosition.Station.AlgaePoseDeg)
            .withProperties(Map.of("min", 0, "max", 10))
            .getEntry();
        tab.add("Algae Stages", kLiftPosition.Stage1.AlgaePoseDeg)
            .withProperties(Map.of("min", 0, "max", 360))
            .getEntry();
        tab.add("Algae Start", kLiftPosition.Start.AlgaePoseDeg)
            .withProperties(Map.of("min", 0, "max", 360))
            .getEntry();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * Throttles.kDriveThrottle, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * Throttles.kDriveThrottle, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * Throttles.kDriveThrottle, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
        m_driverCommander.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.05).whileTrue(new CommandCoralIntake(m_coralArm));
        // m_driverCommander.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.05).whileTrue(new CommandAlgaeIntake);
        m_operatorCommander.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.05).whileTrue(new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Processor));
    
    // m_lift

        // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}
   */

   // Driver Bindings
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new CommandCoralOuttake(m_coralArm));
    // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    //     .whileTrue(new CommandAlgaeOuttake);

    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new CommandHoldThrottle(Throttles.kBoost));
    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value)
        .whileTrue(new CommandToggleThrottle(Throttles.kCreep));

    // Operator Bindings
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Start));
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Stage1));
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Stage2));
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Stage3));
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Algae2));
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, kLiftPosition.Station));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(
            () -> m_reeReefLimelight.alignRobot(
                kAimingPositions.CoralRight, true),
                m_reeReefLimelight
            )
            .finallyDo(
                () -> m_reeReefLimelight.clearFeedbackOverride()
            )
        );

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_reeReefLimelight.alignRobot(
                kAimingPositions.CoralLeft, true),
                m_reeReefLimelight
            )
            .finallyDo(
                () -> m_reeReefLimelight.clearFeedbackOverride()
            )
        );

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(
            () -> m_reeReefLimelight.alignRobot(
                kAimingPositions.Intake, true),
                m_reeReefLimelight
            )
            .finallyDo(
                () -> m_reeReefLimelight.clearFeedbackOverride()
            )
        );

  }

public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
