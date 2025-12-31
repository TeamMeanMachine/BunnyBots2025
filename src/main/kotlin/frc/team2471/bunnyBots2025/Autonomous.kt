package frc.team2471.bunnyBots2025

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.bunnyBots2025.tests.joystickTest
import frc.team2471.bunnyBots2025.tests.leftRightStaticFFTest
import frc.team2471.bunnyBots2025.tests.shooterSysIDLeft
import frc.team2471.bunnyBots2025.tests.shooterSysIDRight
import frc.team2471.bunnyBots2025.tests.slipCurrentTest
import frc.team2471.bunnyBots2025.tests.velocityVoltTest
import frc.team2471.bunnyBots2025.tests.zeroPivot
import frc.team2471.bunnyBots2025.tests.zeroTurret
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team2471.frc.lib.control.Autonomi
import org.team2471.frc.lib.control.commands.finallyWait
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runOnce
import org.team2471.frc.lib.control.commands.sequenceCommand

object Autonomous: Autonomi() {

//    val paths: MutableMap<String, Trajectory<SwerveSample>> = findChoreoPaths()  <-- already inside Autonomi

    /** Supplier that sets the robot's pose. Used inside [setDrivePositionToAutoStartPose] */
    override val drivePoseSetter: (Pose2d) -> Unit = { Drive.pose = it }

    /** Chooser for selecting autonomous commands */
    override val autoChooser: LoggedDashboardChooser<AutoCommand?> = LoggedDashboardChooser<AutoCommand?>("Auto Chooser").apply {
        addOption("8 Foot Straight", AutoCommand(eightFootStraight()))
        addOption("6x6 Square", AutoCommand(squarePathTest()))
        addOption("Left to Center", AutoCommand(leftToCenter()))
        addOption("Right to Center", AutoCommand(rightToCenter()))
        addOption("Cycle Center Left", AutoCommand(cycleCenterLeft()))
        addOption("Cycle Center Right", AutoCommand(cycleCenterRight()))
    }
    /** Chooser for test commands */
    override val testChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("Test Chooser").apply {
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

    /** Autonomous commands */

    private fun eightFootStraight(): Command {
        return Drive.driveAlongChoreoPath(paths["8 foot"]!!, resetOdometry = true)
    }
    private fun squarePathTest(): Command {
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

}
