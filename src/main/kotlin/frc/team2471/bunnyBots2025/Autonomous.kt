package frc.team2471.bunnyBots2025

import choreo.Choreo
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.bunnyBots2025.tests.joystickTest
import frc.team2471.bunnyBots2025.tests.leftRightStaticFFTest
import frc.team2471.bunnyBots2025.tests.shooterSysIDLeft
import frc.team2471.bunnyBots2025.tests.shooterSysIDRight
import frc.team2471.bunnyBots2025.tests.slipCurrentTest
import frc.team2471.bunnyBots2025.tests.velocityVoltTest
import frc.team2471.bunnyBots2025.tests.zeroPivot
import frc.team2471.bunnyBots2025.tests.zeroTurret
import org.team2471.frc.lib.units.asSeconds
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.control.commands.toCommand
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team2471.frc.lib.control.commands.finallyWait
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnce
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.util.isRedAlliance
import kotlin.collections.forEach
import kotlin.io.path.listDirectoryEntries
import kotlin.io.path.name
import kotlin.jvm.optionals.getOrNull

object Autonomous {
    val paths: MutableMap<String, Trajectory<SwerveSample>> = findChoreoPaths()

    // Chooser for selecting autonomous commands
    private val autoChooser: LoggedDashboardChooser<AutoCommand?> = LoggedDashboardChooser<AutoCommand?>("Auto Chooser").apply {
        addOption("8 Foot Straight", AutoCommand(eightFootStraight()))
        addOption("6x6 Square", AutoCommand(squarePathTest()))
        addOption("Left to Center", AutoCommand(leftToCenter()))
        addOption("Right to Center", AutoCommand(rightToCenter()))
        addOption("Cycle Center Left", AutoCommand(cycleCenterLeft()))
        addOption("Cycle Center Right", AutoCommand(cycleCenterRight()))
    }
    // Chooser for test commands
    private val testChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Test Chooser").apply {
        // Set up SysId routines and test command options
        addOption("Drive Translation SysId ALL", Drive.sysIDTranslationAll())
        addOption("Drive Rotation SysId ALL", Drive.sysIDRotationAll())
        addOption("Drive Steer SysId ALL", Drive.sysIDSteerAll())
        addOption("Set Angle Offsets", Drive.setAngleOffsets())
        addOption("JoystickTest", joystickTest())
        addOption("Drive Slip Current Test", Drive.slipCurrentTest())
        addOption("Drive L/R Static FF Test", Drive.leftRightStaticFFTest())
        addOption("Drive Velocity Volt Test", Drive.velocityVoltTest())
        addOption("Zero Pivot", zeroPivot())
        addOption("Zero Turret", zeroTurret())
        addOption("Shooter sysID Left", shooterSysIDLeft())
        addOption("Shooter sysID Right", shooterSysIDRight())
    }

    var selectedAuto: AutoCommand? = null
        private set
    val autonomousCommand: Command? get() = if (!Drive.demoMode) selectedAuto?.command else ({ println("DEMO MODE: Not running auto, no killing kids today.") }).toCommand()
    val testCommand: Command? get() = testChooser.get()

    /**
     * Initial value determines which side all choreo paths are made for.
     * False = all choreo paths are made on the blue side.
     * True = all choreo paths are made on the red side.
     */
    var isPathsRed = false
        private set

    private var prevPathRed: Boolean? = null

    init {
        readAutoPaths()
    }

    fun updateSelectedAuto() {
        val startTime = RobotController.getMeasureFPGATime()
        val newAuto = autoChooser.get()
        if (selectedAuto != newAuto) {
            selectedAuto = autoChooser.get()
            println("selected auto changed ${autoChooser.sendableChooser.selected}")
            setDrivePositionToAutoStartPose()
            println("finished reading auto in ${(RobotController.getMeasureFPGATime() - startTime).asSeconds} seconds")
        }
    }

    fun setDrivePositionToAutoStartPose() {
        val startingPose = selectedAuto?.startingPoseSupplier?.invoke()
        if (startingPose != null) {
            println("resetting drive pose to auto start pose")
            Drive.pose = startingPose
        }
    }

    /**
     * Checks if the alliance color has changed and flips the paths if so
     */
    fun flipPathsIfAllianceChange() {
        if (prevPathRed != null) {
            if (prevPathRed != isRedAlliance) {
                flipPaths()
            }
        } else if (isRedAlliance != isPathsRed) { // Code always goes here for the first time because prevPathRed starts null
            flipPaths()
        }
    }

    fun readAutoPaths() {
        val startTime = RobotController.getMeasureFPGATime()
        val pathNameAndStartPose = mutableListOf<Pair<String, Pose2d>>()
        val segments = mutableListOf<ChassisSpeeds?>()
        paths.forEach {
            pathNameAndStartPose.add(Pair(
                it.value.name(),
                it.value.sampleAt(0.0, false).get().pose
            ))
            val pathSegment = it.value.totalTime / 10.0
            for (i in 0..10) {
                segments.add(it.value.sampleAt(i * pathSegment, false).getOrNull()?.chassisSpeeds)
            }
        }
        println("paths: $pathNameAndStartPose")
        println("reading ${paths.size} paths and ${segments.size} samples. Took ${(RobotController.getMeasureFPGATime() - startTime).asSeconds.round(4)} seconds.")
    }

    /**
     * Flip the path so it is correct for the alliance color
     */
    private fun flipPaths() {
        println("flipping paths")
        paths.replaceAll { _, t -> t.flipped() }
        isPathsRed = !isPathsRed
        prevPathRed = isPathsRed
        setDrivePositionToAutoStartPose()
        println(paths.map { it.value.sampleAt(0.0, false)?.get()?.pose})
        readAutoPaths()
    }

    /**
     * Find all the paths in the choreo directory and return a list of them
     */
    private fun findChoreoPaths(): MutableMap<String, Trajectory<SwerveSample>> {
        return try {
            val map: MutableMap<String, Trajectory<SwerveSample>> = mutableMapOf()
            Filesystem.getDeployDirectory().toPath().resolve("choreo").listDirectoryEntries("*.traj").forEach {
                try {
                    val name = it.name.removeSuffix(".traj")
                    val traj = Choreo.loadTrajectory(name).getOrNull()
                    if (traj != null) {
                        @Suppress("UNCHECKED_CAST")
                        map[name] = traj as Trajectory<SwerveSample>
                    }
                } catch (e: Exception) {
                    println("failed to load path at $it")
                    println(e)
                }
            }
            println("loaded ${map.size} paths")
            map
        } catch (e: Exception) {
            println("failed to load any auto paths $e"); mutableMapOf()
        }
    }

    private fun eightFootStraight (): Command {
        return Drive.driveAlongChoreoPath(paths["8 foot"]!!, resetOdometry = true)
    }
    private fun squarePathTest (): Command {
        return Drive.driveAlongChoreoPath(paths["square"]!!, resetOdometry = true)
    }

    private fun leftToCenter(): Command {
        return parallelCommand(
            Drive.driveAlongChoreoPath(paths["Left to center"]!!, resetOdometry = true),
            sequenceCommand(
                Intake.home(),
                runOnce {
                    Intake.deploy()
                    Intake.intakeState = Intake.IntakeState.SHOOTING
                }
            )
        )
    }

    private fun rightToCenter(): Command {
        return parallelCommand(
            Drive.driveAlongChoreoPath(paths["Right to center"]!!, resetOdometry = true),
            sequenceCommand(
                Intake.home(),
                runOnce {
                    Intake.deploy()
                    Intake.intakeState = Intake.IntakeState.SHOOTING
                }
            )
        )
    }

    private fun cycleCenterLeft(): Command {
        val path = paths["CycleCenterLeft"]!!
        return parallelCommand (
            Turret.aimAtGoal(),
            sequenceCommand(
                parallelCommand(
                    Drive.driveAlongChoreoPath(path.getSplit(0).get(), resetOdometry = true),
                    Intake.home()
                ),
                runOnce {
                    Intake.intakeState = Intake.IntakeState.SHOOTING
                }.finallyWait(2.5),
                parallelCommand(
                    Drive.driveAlongChoreoPath(path.getSplit(1).get()),
                    runOnce {
                        Intake.deploy()
                        Intake.intakeState = Intake.IntakeState.INTAKING
                    }
                ),
                runOnce {
                    Intake.intakeState = Intake.IntakeState.SHOOTING
                }.finallyWait(2.5),
                parallelCommand(
                    Drive.driveAlongChoreoPath(path.getSplit(2).get()),
                    runOnce {
                        Intake.stow()
                    }
                )
            )
        )
    }

    private fun cycleCenterRight(): Command {
        val path = paths["CycleCenterRight"]!!
        return parallelCommand (
            Turret.aimAtGoal(),
            sequenceCommand(
                parallelCommand(
                    Drive.driveAlongChoreoPath(path.getSplit(0).get(), resetOdometry = true),
                    Intake.home()
                ),
                runOnce {
                    Intake.intakeState = Intake.IntakeState.SHOOTING
                }.finallyWait(2.5),
                parallelCommand(
                    Drive.driveAlongChoreoPath(path.getSplit(1).get()),
                    runOnce {
                        Intake.deploy()
                        Intake.intakeState = Intake.IntakeState.INTAKING
                    }
                ),
                runOnce {
                    Intake.intakeState = Intake.IntakeState.SHOOTING
                }.finallyWait(2.5),
                parallelCommand(
                    Drive.driveAlongChoreoPath(path.getSplit(2).get()),
                    runOnce {
                        Intake.stow()
                    }
                )
            )
        )
    }

    private fun pathPlannerPath(): Command {
        return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("3 L4 Right", 1))
    }

    class AutoCommand(val command: Command, val startingPoseSupplier: (() -> Pose2d)? = null)
}
