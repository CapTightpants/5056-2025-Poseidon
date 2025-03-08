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
import frc.robot.Constants.Setpoints.kLiftPosition;
import frc.robot.Vars.Throttles;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Lift m_lift = new Lift(SystemConstants.kLeftLiftCanId, SystemConstants.kRightLiftCanId);
  private final CoralArm m_coralArm = new CoralArm(SystemConstants.kCoralIntakeCanId, SystemConstants.kCoralArmCanId, SystemConstants.kCoralLimitDIO);
  private final AlgaeArm m_algaeArm = new AlgaeArm(SystemConstants.kAlgaeIntakeCanId, SystemConstants.kAlgaeArmCanId);
  
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverCommander = new CommandXboxController(OIConstants.kDriverControllerPort);
  
  // The operator's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  CommandXboxController m_operatorCommander = new CommandXboxController(OIConstants.kOperatorControllerPort);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
      
    // Register auto commands
    NamedCommands.registerCommand("start", new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.Start));
    NamedCommands.registerCommand("stage1", new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.Stage1));
    NamedCommands.registerCommand("stage2", new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.Stage2));
    NamedCommands.registerCommand("stage3", new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.Stage3));
    NamedCommands.registerCommand("base", new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.Base));
    NamedCommands.registerCommand("algae2", new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.algae2));
    NamedCommands.registerCommand("processor", new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.processor));
    NamedCommands.registerCommand("intakeCoral", new CommandCoralIntake(m_coralArm));
    NamedCommands.registerCommand("intakeAlgae", new CommandAlgaeIntake(m_algaeArm));
    NamedCommands.registerCommand("outtakeCoral", new CommandCoralOuttake(m_coralArm));
    NamedCommands.registerCommand("outtakeAlgae", new CommandAlgaeOuttake(m_algaeArm));
    
    // Configure SmartDashboard
    SmartDashboard.putData("Algae Arm Position: Base",   new CommandPositionAlgae(m_algaeArm, kLiftPosition.Base));
    SmartDashboard.putData("Algae Arm Position: Stage1", new CommandPositionAlgae(m_algaeArm, kLiftPosition.Stage1));
    SmartDashboard.putData("Algae Arm Position: Stage2", new CommandPositionAlgae(m_algaeArm, kLiftPosition.Stage2));
    SmartDashboard.putData("Algae Arm Position: Stage3", new CommandPositionAlgae(m_algaeArm, kLiftPosition.Stage3));
    SmartDashboard.putData("Algae Arm Position: Start",  new CommandPositionAlgae(m_algaeArm, kLiftPosition.Start));
    SmartDashboard.putData("Coral Arm Position: Base",   new CommandPositionCoral(m_coralArm, kLiftPosition.Base));
    SmartDashboard.putData("Coral Arm Position: Stage1", new CommandPositionCoral(m_coralArm, kLiftPosition.Stage1));
    SmartDashboard.putData("Coral Arm Position: Stage2", new CommandPositionCoral(m_coralArm, kLiftPosition.Stage2));
    SmartDashboard.putData("Coral Arm Position: Stage3", new CommandPositionCoral(m_coralArm, kLiftPosition.Stage3));
    SmartDashboard.putData("Coral Arm Position: Start",  new CommandPositionCoral(m_coralArm, kLiftPosition.Start));
    SmartDashboard.putData("Lift Position: Base",   new CommandPositionLift(m_lift, kLiftPosition.Base));
    SmartDashboard.putData("Lift Position: Stage1", new CommandPositionLift(m_lift, kLiftPosition.Stage1));
    SmartDashboard.putData("Lift Position: Stage2", new CommandPositionLift(m_lift, kLiftPosition.Stage3));
    SmartDashboard.putData("Lift Position: Stage3", new CommandPositionLift(m_lift, kLiftPosition.Stage3));
    SmartDashboard.putData("Lift Position: Start",  new CommandPositionLift(m_lift, kLiftPosition.Start));
    
    // for (kLiftPosition position : kLiftPosition.values()) {
        final ShuffleboardTab tab = Shuffleboard.getTab("Lift Values");
            tab.add("Algae Base", kLiftPosition.Base.AlgaePoseDeg)
                .withProperties(Map.of("min", 0, "max", 10))
                .getEntry();
            tab.add("Algae Stages", kLiftPosition.Stage1.AlgaePoseDeg)
                .withProperties(Map.of("min", 0, "max", 360))
                .getEntry();
            tab.add("Algae Start", kLiftPosition.Start.AlgaePoseDeg)
                .withProperties(Map.of("min", 0, "max", 360))
                .getEntry();

    // }
    // kLiftPosition.Base.AlgaePoseDeg = SmartDashboard.getNumber("Base Value: Algae", kLiftPosition.Base.AlgaePoseDeg);
    // kLiftPosition.Stage1.AlgaePoseDeg = SmartDashboard.getNumber("Stages Value: Algae", kLiftPosition.Stage1.AlgaePoseDeg);
    // kLiftPosition.Start.AlgaePoseDeg = SmartDashboard.getNumber("Start Value: Algae", kLiftPosition.Start.AlgaePoseDeg);

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
        m_driverCommander.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.05).whileTrue(new CommandAlgaeIntake(m_algaeArm));
        m_operatorCommander.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.05).whileTrue(new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.processor));
    
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
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new CommandAlgaeOuttake(m_algaeArm));

    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new CommandSetBoost(Throttles.kBoost));
    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value)
        .whileTrue(new CommandSetCreep(Throttles.kCreep));

    // Operator Bindings
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.Start));
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.Stage1));
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.Stage2));
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.Stage3));
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.algae2));
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new CommandMultiPosition(m_lift, m_coralArm, m_algaeArm, kLiftPosition.Base));
  }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     // Create config for trajectory
//     TrajectoryConfig config = new TrajectoryConfig(
//         AutoConstants.kMaxSpeedMetersPerSecond,
//         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//         // Add kinematics to ensure max speed is actually obeyed
//         .setKinematics(DriveConstants.kDriveKinematics);

//     // An example trajectory to follow. All units in meters.
//     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(0, 0, new Rotation2d(0)),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(3, 0, new Rotation2d(0)),
//         config);

//     var thetaController = new ProfiledPIDController(
//         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//         exampleTrajectory,
//         m_robotDrive::getPose, // Functional interface to feed supplier
//         DriveConstants.kDriveKinematics,

//         // Position controllers
//         new PIDController(AutoConstants.kPXController, 0, 0),
//         new PIDController(AutoConstants.kPYController, 0, 0),
//         thetaController,
//         m_robotDrive::setModuleStates,
//         m_robotDrive);

//     // Reset odometry to the starting pose of the trajectory.
//     m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

//     // Run path following command, then stop at the end.
//     return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
//   }

public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
