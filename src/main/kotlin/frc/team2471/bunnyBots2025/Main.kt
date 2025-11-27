@file:JvmName("Main") // set the compiled Java class name to "Main" rather than "MainKt"
package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.ctre.loggedTalonFX.MasterMotor
import org.team2471.frc.lib.units.asFeet
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team2471.frc.lib.util.RobotMode
import org.team2471.frc.lib.util.isSim
import org.team2471.frc.lib.util.robotMode
import java.net.NetworkInterface
import kotlin.collections.iterator

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@OptIn(DelicateCoroutinesApi::class)
object Robot : LoggedRobot() {
    val isCompBot = getCompBotBoolean()
    private var wasDisabled = true
    var beforeFirstEnable = true
        private set

    val commandScheduler = CommandScheduler.getInstance()

    private val enabledTimer = Timer()
    val timeSinceEnabled get() = enabledTimer.get()

    @get:JvmName("RobotIsEnabled")
    var isEnabled = false
        private set
    @get:JvmName("RobotIsAutonomous")
    var isAutonomous = false
        private set
    @get:JvmName("RobotIsDisabled")
    var isDisabled = false
        private set
    @get:JvmName("RobotIsAutonomousEnabled")
    var isAutonomousEnabled = false
        private set
    override fun isEnabled(): Boolean = isEnabled
    override fun isAutonomous(): Boolean = isAutonomous
    override fun isDisabled(): Boolean = isDisabled
    override fun isAutonomousEnabled(): Boolean = isAutonomousEnabled


    // Subsystems:
    // MUST define an individual variable for all subsystems inside this class or else @AutoLogOutput will not work -2025
    val drive = Drive
    val oi = OI
    val shooter = Shooter
    val intake = Intake
    val turret = Turret

    var allSubsystems = arrayOf(drive, turret, intake, shooter, oi)

    init {
        // Tells FRC we use Kotlin
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin)

        // Set up data receivers & replay source
        when (robotMode) {
            RobotMode.REAL -> { // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }
            RobotMode.SIM -> {
                Logger.addDataReceiver(NT4Publisher())
                Logger.addDataReceiver(WPILOGWriter())
            } // Running a physics simulator, log to NT
            RobotMode.REPLAY -> { // Replaying a log, set up replay source
                setUseTiming(false) // Run as fast as possible
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }

        DriverStation.silenceJoystickConnectionWarning(true)

        SignalLogger.setPath("")
        SignalLogger.start()

        // Start AdvantageKit logger
        Logger.start()
        // Call all subsystems, make sure their init's run
        allSubsystems.forEach { println("activating subsystem ${it.name}") }
        println("FieldManager thinks the field is ${FieldManager.fieldDimensions.asFeet} feet big")
        println("We see ${Autonomous.paths.size} paths and they are made on the ${if (Autonomous.isPathsRed) "red" else "blue"} side.")

        GlobalScope.launch {
            while (true) {
                isEnabled = DriverStation.isEnabled()
                isDisabled = !isEnabled
                isAutonomous = DriverStation.isAutonomous()
                isAutonomousEnabled = isAutonomous && isEnabled
                delay(10.toLong())
            }
        }
    }

    /** This function is called periodically during all modes.  */
    override fun robotPeriodic() {
        LoopLogger.reset()
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
//         Threads.setCurrentThreadPriority(true, 99);

        if (Robot.isEnabled) {
            if (wasDisabled) {
                beforeFirstEnable = false
                enabledInit()
                wasDisabled = false
            }
        } else {
            wasDisabled = true
        }


        LoopLogger.record("b4 CommandScheduler")
        commandScheduler.run()

        // Return to non-RT thread priority (do not modify the first argument)
//         Threads.setCurrentThreadPriority(false, 10);
        LoopLogger.record("Robot periodic()")
    }

    fun enabledInit() {
//        enabledTimer.restart()
        println("Enabled init $timeSinceEnabled")
        Drive.brakeMode()
    }

    /** This function is called once when the robot is disabled.  */
    override fun disabledInit() {
        Drive.coastMode()
        Autonomous.autonomousCommand?.cancel() // This makes sure that the autonomous stops running when teleop starts running.
        Autonomous.testCommand?.cancel()
    }

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {
        Autonomous.flipPathsIfAllianceChange()
        Autonomous.updateSelectedAuto()
        Intake.deployMotor.setControl(MotionMagicDutyCycle(Intake.deployMotor.position.valueAsDouble))
    }

    /** This function is called once when auto is enabled.  */
    override fun autonomousInit() {
        enabledTimer.restart()
        println("Autonomous init $timeSinceEnabled")
        if (isSim) Autonomous.flipPathsIfAllianceChange() // Only needed in sim
        Autonomous.setDrivePositionToAutoStartPose()
        println("scheduling auto command $timeSinceEnabled")
        (Autonomous.autonomousCommand ?: Commands.runOnce({ println("THE AUTONOMOUS COMMAND IS NULL") })).schedule()
        println("scheduled auto command $timeSinceEnabled")
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled.  */
    override fun teleopInit() {
        enabledTimer.restart()

    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {}

    /** This function is called once when test mode is enabled.  */
    override fun testInit() {
        enabledTimer.restart()
        CommandScheduler.getInstance().cancelAll() // Cancels all running commands at the start of test mode.
        (Autonomous.testCommand ?: Commands.runOnce({println("THE TEST COMMAND IS NULL")})).schedule()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {}

    /** This function is called periodically whilst in simulation.  */
    @OptIn(DelicateCoroutinesApi::class)
    override fun simulationPeriodic() {
        GlobalScope.launch {
            MasterMotor.simPeriodic()
        }
    }


    private fun getCompBotBoolean(): Boolean {
        var compBot = true
        if (robotMode == RobotMode.REAL) {
            val networkInterfaces =  NetworkInterface.getNetworkInterfaces()
            println("retrieving network interfaces")
            for (iFace in networkInterfaces) {
                println(iFace.name)
                if (iFace.name == "eth0") {
                    println("NETWORK NAME--->${iFace.name}<----")
                    var macString = ""
                    for (byteVal in iFace.hardwareAddress){
                        macString += String.format("%s", byteVal)
                    }
                    println("FORMATTED---->$macString<-----")

                    compBot = (macString == "0-128475710531")
                }
            }
        } else { println("Not real so I am compbot") }
        println("I am compbot = $compBot")
        return compBot
    }
}

/**
 * Main initialization function. Do not perform any initialization here
 * other than calling `RobotBase.startRobot`. Do not modify this file
 * except to change the object passed to the `startRobot` call.
 *
 * If you change the package of this file, you must also update the
 * `ROBOT_MAIN_CLASS` variable in the gradle build file. Note that
 * this file has a `@file:JvmName` annotation so that its compiled
 * Java class name is "Main" rather than "MainKt". This is to prevent
 * any issues/confusion if this file is ever replaced with a Java class.
 * See the [Package Level Functions](https://kotlinlang.org/docs/java-to-kotlin-interop.html#package-level-functions)
 * section on the *Calling Kotlin from Java* page of the Kotlin Docs.
 *
 * If you change your main frc.team2471.bunnyBots2025.Robot object (name), change the parameter of the
 * `RobotBase.startRobot` call below to the new name. (If you use the IDE's
 * Rename * Refactoring when renaming the object, it will get changed everywhere
 * including here.)
 */
fun main() = RobotBase.startRobot { Robot }
