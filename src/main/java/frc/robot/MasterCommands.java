package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutomaticTargetAligner;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HarvesterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PhotonVisionSwerveUtil;

/**
 * Class Containing lists of pre-determined command groups. (Same as Constants class just cleaner in separate file)
 */
public final class MasterCommands
{
    private final SwerveSubsystem m_SwerveSubsystem;
    private final HarvesterSubsystem m_HarvesterSubsystem;
    private final IndexerSubsystem m_IndexerSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;
    private final ActuatorSubsystem m_ActuatorSubsystem;
    private final ClimberSubsystem m_ClimberSubsystem;
    private final PhotonVisionSwerveUtil m_PhotonVisionSwerveUtil;

    private enum CommandSelector {
        ONE,
        TWO,
        THREE
    }

    /**
     * Groups of Commands to complete large tasks (i.e. harvesting, scoring, driving, etc.)
     * @param swerveSubsystem - Requires swerveSubsystem
     * @param harvesterSubsystem - Requires harvesterSubsystem
     * @param indexerSubsystem - Requires indexerSubsystem
     * @param shooterSubsystem - Requires shooterSubsystem
     * @param climberSubsystem - Requires climberSubsystem
     */
    public MasterCommands(SwerveSubsystem swerveSubsystem, HarvesterSubsystem harvesterSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, ActuatorSubsystem actuatorSubsystem, ClimberSubsystem climberSubsystem, PhotonVisionSwerveUtil photonVisionSwerveUtil) {
        m_SwerveSubsystem = swerveSubsystem;
        m_HarvesterSubsystem = harvesterSubsystem;
        m_IndexerSubsystem = indexerSubsystem;
        m_ShooterSubsystem = shooterSubsystem;
        m_ActuatorSubsystem = actuatorSubsystem;
        m_ClimberSubsystem = climberSubsystem;
        m_PhotonVisionSwerveUtil = photonVisionSwerveUtil;
    }

    /**
     * Stops all robot mechanisms and interrupts all other commands (Clean Reset)
     */
    public Command interruptAll()
    {
        return Commands.parallel
        (
            m_SwerveSubsystem.CMDxBrakes()
            , m_HarvesterSubsystem.CMD_stopHarvester()
            , m_IndexerSubsystem.CMD_stopIndexer()
            , m_ShooterSubsystem.CMD_stopShooting()
        ).repeatedly();
    }

    public Command Harvest()
    {
        // TODO: Add Proxy
        return Commands.deadline
        (
            m_IndexerSubsystem.CMD_harvestUntilFull()
            , m_HarvesterSubsystem.CMD_harvest()
        )
            .andThen(m_IndexerSubsystem.CMD_stopIndexer())
            .andThen(m_HarvesterSubsystem.CMD_stopHarvester());
    }

    public Command ReverseHarvest()
    {
        return Commands.deadline
        (
            m_IndexerSubsystem.CMD_indexReverse()
            , m_HarvesterSubsystem.CMD_reverseHarvest()
        ).repeatedly()
            .andThen(m_IndexerSubsystem.CMD_stopIndexer())
            .andThen(m_HarvesterSubsystem.CMD_stopHarvester());
    }

    // public Command StopAllHarvest()
    // {
    //     return Commands.parallel
    //     (
    //         m_IndexerSubsystem.CMD_stopIndexer()
    //         , m_HarvesterSubsystem.CMD_stopHarvester()
    //     );
    // }


    // OLD SHOOTING COMMAND
    // public Command Shoot()
    // {
    //     return Commands.sequence
    //     (
    //         m_ShooterSubsystem.CMD_shootNote()
    //         , new WaitCommand(1)
    //         , m_IndexerSubsystem.CMD_indexFeed()
    //         , new WaitCommand(0.5)
    //     )
    //         .andThen(m_IndexerSubsystem.CMD_stopIndexer())
    //         .andThen(m_ShooterSubsystem.CMD_stopShooting());
    // }

    public Command Shoot()
    {
        // Ask MORT about their shooting sequence? Timer or RPM. . .
        return Commands.sequence
        (
            m_ShooterSubsystem.CMD_shootNoteAtTargetVelocity()
            , m_IndexerSubsystem.CMD_indexFeed()
            , new WaitCommand(0.5)
        )
            .andThen(m_IndexerSubsystem.CMD_stopIndexer())
            .andThen(m_ShooterSubsystem.CMD_stopShooting());
    }

    public Command aimToSpeaker(Supplier<Double> leftX, Supplier<Double> leftY)
    {
        return new AutomaticTargetAligner(m_SwerveSubsystem, m_ActuatorSubsystem, m_PhotonVisionSwerveUtil, FieldElements.SPEAKER, true, true, leftX, leftY);
    }

    public Command aimWhileShoot(Supplier<Double> leftX, Supplier<Double> leftY)
    {
        return Commands.parallel(
            aimWhileShoot(leftX, leftY)
            , Shoot()
            );
    }

    public Command isShotReady()
    {
        // TODO: Create a Boolean Supplier to check if shot is ready for scoring.
        return null;
    }

    public Command extendClimber()
    {
        return m_ClimberSubsystem.CMD_setClimberTo(75);
    }

    public Command retractClimber()
    {
    return m_ClimberSubsystem.CMD_setClimberTo(0);
    }


    public Command trapShot()
    {
        return Commands.sequence
        (
            m_ShooterSubsystem.CMD_trapShot()
            , new WaitCommand(0.5)
            , m_IndexerSubsystem.CMD_indexFeed()
            , new WaitCommand(0.5)
        )
            .andThen(m_IndexerSubsystem.CMD_stopIndexer())
            .andThen(m_ShooterSubsystem.CMD_stopShooting());
    }
    


    /**
     * List of CommandGroups Containing All Shooter Related Tasks
     */
    public final class ShooterCommands
    {
        public final ParallelCommandGroup alignShooter = new ParallelCommandGroup
        (
            // new DriveFacingX().repeatedly()
            // , new ActuateToSpeakerAngle().repeatedly()
            // isShotReady()
            // Shoot()
            // Finish this . . .
        );
    }
}
