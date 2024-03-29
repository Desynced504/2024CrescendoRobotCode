// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ActuatorCommands.ActuatorAngleTargetting;
// import frc.robot.commands.ActuatorCommands.AutoSpeakerActuating;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ActuatorSubsystem;
// import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
// import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.HarvesterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = TunerConstants.SwerveDriveTrain;
  private final HarvesterSubsystem m_HarvesterSubsystem = new HarvesterSubsystem();
  private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final ActuatorSubsystem m_ActuatorSubsystem = new ActuatorSubsystem();
  // private final CameraSubsystem m_CameraSubsystem = new CameraSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

  private final MasterCommands MasterCommands = new MasterCommands(m_SwerveSubsystem, m_HarvesterSubsystem, m_IndexerSubsystem, m_ShooterSubsystem, m_ClimberSubsystem);
  // Command Groups
  //   public final ParallelDeadlineGroup harvestSequence = new ParallelDeadlineGroup
  //       (
  //         m_IndexerSubsystem.CMDisNotePresent()
  //         , new ParallelCommandGroup
  //         (
  //           m_HarvesterSubsystem.CMDharvest()
  //           , m_IndexerSubsystem.CMDindexFeed()
  //         )
  //       ).cf9gv  
  //       finallyDo(m_IndexerSubsystem.CMDstopIndexer());
  //       .andThen(m_IndexerSubsystem.CMDstopIndexer()).andThen(m_HarvesterSubsystem.CMDstopHarvester());

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Joystick m_crescendoController =
      new Joystick(OperatorConstants.kAIcontroller);

  private DoubleSupplier joystickSlider = () -> m_crescendoController.getRawAxis(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // TODO: Register named Commands for Automs in PathPlanner
    NamedCommands.registerCommand("Shoot", MasterCommands.Shoot());
    NamedCommands.registerCommand("Harvest", MasterCommands.Harvest());

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Default Commands
    m_SwerveSubsystem.setDefaultCommand
    (
      m_SwerveSubsystem.drive(
          () -> -m_controller.getLeftY()
          , () -> -m_controller.getLeftX()
          , () -> -m_controller.getRightX()
          )
    );
    m_ClimberSubsystem.setDefaultCommand(m_ClimberSubsystem.stopClimber());

    // Event Trigger Commands
    m_controller.x().onTrue(MasterCommands.Harvest());
    m_controller.b().onTrue(MasterCommands.Shoot());
    
    m_controller.a().whileTrue(MasterCommands.interruptAll());
    m_controller.y().whileTrue(MasterCommands.ReverseHarvest());

    // TODO: Camera is 13 inches from center of robot
    m_controller.button(7).onTrue(new ActuatorAngleTargetting(m_ActuatorSubsystem, () -> 35));
    m_controller.button(8).onTrue(new ActuatorAngleTargetting(m_ActuatorSubsystem, () -> 53));
    // m_controller.button(8).whileTrue(new AutoSpeakerActuating(m_ActuatorSubsystem, 00));
    m_controller.rightBumper().onTrue(m_SwerveSubsystem.CMDzeroGyro());
 
    // new JoystickButton(m_crescendoController, 7).whileTrue(new ActuatorAngleTargetting(m_ActuatorSubsystem, () -> (((joystickSlider.getAsDouble()+1)*20) + 30)));
    new JoystickButton(m_crescendoController, 1).onTrue(MasterCommands.trapShot());

    new JoystickButton(m_crescendoController, 2).whileTrue(m_ClimberSubsystem.extend());
    new JoystickButton(m_crescendoController, 3).whileTrue(m_ClimberSubsystem.retract());

    
    // TODO: Test Controller Color LAST THING

    // m_controller.getHID().setOutput(0, false);;
    // m_controller.y().onTrue(m_ActuatorSubsystem.spinMotorTo(35));
    // m_controller.leftBumper().onTrue(m_ActuatorSubsystem.factoryResetActuator());
    // m_controller.leftTrigger(0.5).onTrue(m_ClimberSubsystem.CMD_setClimberTo(0));
    // m_controller.rightTrigger(0.5).onTrue(m_ClimberSubsystem.CMD_setClimberTo(75));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      MasterCommands.Shoot()
      // , new PathPlannerAuto("4NoteGrab5")
      // , new PathPlannerAuto("4 Note From Middle")
      // , MasterCommands.Shoot()
      );
    // return null;
      // return m_ActuatorSubsystem.factoryResetActuator();
    // return new PathPlannerAuto("DebugAuto");
    // return new SequentialCommandGroup
    // (
    //   // m_ActuatorSubsystem.factoryResetActuator()
    //   // , m_ActuatorSubsystem.spinMotorTo(35)
    //   // .andThen(MasterCommands.Shoot())
    //   // .andThen(new PathPlannerAuto("DebugAuto"))

    //   (new PathPlannerAuto("DebugAuto"))
    //   );
  }
}