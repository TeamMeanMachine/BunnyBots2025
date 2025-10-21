package frc.team2471.bunnyBots2025.tests

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team2471.bunnyBots2025.OI
import frc.team2471.bunnyBots2025.Drive
import org.team2471.frc.lib.control.Direction
import org.team2471.frc.lib.control.commands.beforeRun
import org.team2471.frc.lib.control.commands.beforeWait
import org.team2471.frc.lib.control.dPad
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.ApplyModuleStatesVoltage
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.degreesPerSecond
import org.team2471.frc.lib.units.feetPerSecond
import org.team2471.frc.lib.units.meters
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.util.translation


fun joystickTest(): Command {
    val joystickValues = mutableListOf<Translation2d>()


    return Commands.run({
        val translation = OI.rawDriveTranslation
        if (translation.norm > 0.5) {
            if (Math.random() > 0.5) {
//                joystickValues.clear()
                joystickValues.add(translation)
            }
            Logger.recordOutput("joystickValues", *joystickValues.toTypedArray())
            Logger.recordOutput("rawJoystickValue", Translation2d(OI.driveTranslationX, OI.driveTranslationY))
        }
    }, Drive)
}

fun Drive.velocityVoltTest(): Command {
    var v = 0.0
    var upPressed = false
    var downPressed = false
    return runCommand(Drive) {
        if (OI.driverController.dPad == Direction.UP) {
            upPressed = true
        } else if (OI.driverController.dPad == Direction.DOWN) {
            downPressed = true
        }
        if (OI.driverController.dPad != Direction.UP && upPressed) {
            upPressed = false
            v += 0.005
        }
        if (OI.driverController.dPad != Direction.DOWN && downPressed) {
            downPressed = false
            v -= 0.005
        }
        println("v: $v velocity: ${Drive.speeds.translation.norm}")
        Drive.driveVoltage(ChassisSpeeds(v, 0.0, 0.0))
    }
}

fun Drive.leftRightStaticFFTest(moduleAngle: Angle = 0.0.degrees): Command {
    var leftVolts = 0.0
    var rightVolts = 0.0

    var upPressed = false
    var downPressed = false
    var leftPressed = false
    var rightPressed = false
    return runCommand(Drive) {


        when (OI.driverController.dPad) {
            Direction.UP -> { upPressed = true }
            Direction.DOWN -> { downPressed = true }
            Direction.LEFT -> { leftPressed = true }
            Direction.RIGHT -> { rightPressed = true }
            else -> {}
        }

        if (OI.driverController.dPad != Direction.UP && upPressed) {
            upPressed = false
            leftVolts += 0.005
            rightVolts += 0.005
        }
        if (OI.driverController.dPad != Direction.DOWN && downPressed) {
            downPressed = false
            leftVolts -= 0.005
            rightVolts -= 0.005
        }
        if (OI.driverController.dPad != Direction.LEFT && leftPressed) {
            leftPressed = false
            leftVolts -= 0.005
        }
        if (OI.driverController.dPad != Direction.RIGHT && rightPressed) {
            rightPressed = false
            rightVolts -= 0.005
        }

        val leftState = SwerveModuleState(leftVolts, moduleAngle.asRotation2d)
        val rightState = SwerveModuleState(rightVolts, moduleAngle.asRotation2d)

        println("leftVolts: $leftVolts rightVolts: $rightVolts")

        Drive.setControl(ApplyModuleStatesVoltage(*arrayOf(leftState, rightState, leftState, rightState)))
    }
}

/** Measures the robot's wheel radius by spinning in a circle.
fun wheelRadiusCharacterization(): Command {
    val limiter = SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE)
    val state = WheelRadiusCharacterizationState()

    return Commands.parallel( // Drive control sequence
        Commands.sequence( // Reset acceleration limiter
            Commands.runOnce({
                limiter.reset(0.0)
            }),  // Turn in place, accelerating up to full speed

            Commands.run({
                val speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY)
                Drive.driveVelocity(ChassisSpeeds(0.0, 0.0, speed))
            },
                Drive
            )
        ),  // Measurement sequence

        Commands.sequence( // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),  // Record starting measurement

            Commands.runOnce(
                {
                    state.positions = Drive.wheelRadiusCharacterizationPositions
                    state.lastAngle = Drive.rotation
                    state.gyroDelta = 0.0
                }),  // Update gyro delta

            Commands.run(
                {
                    val rotation = Drive.rotation
                    state.gyroDelta += abs(rotation.minus(state.lastAngle).radians)
                    state.lastAngle = rotation
                }) // When cancelled, calculate and print results

                .finallyDo(
                    Runnable {
                        val positions = Drive.wheelRadiusCharacterizationPositions
                        var wheelDelta = 0.0
                        for (i in 0..3) {
                            wheelDelta += abs(positions[i] - state.positions[i]) / 4.0
                        }
                        val wheelRadius = (state.gyroDelta * TunerConstants.DRIVE_BASE_RADIUS) / wheelDelta

                        val formatter: NumberFormat = DecimalFormat("#0.000")
                        println("********** Wheel Radius Characterization Results **********")
                        println("\tWheel Delta: " + wheelDelta.radians.asDegrees + " degrees")
                        println("\tGyro Delta: " + state.gyroDelta.radians.asDegrees + " degrees")
                        println(("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, " + formatter.format(
                            Units.metersToInches(wheelRadius)) + " inches"))
                    })
        )
    )
}*/

/*
*
 * Measures the velocity feedforward constants for the drive motors.
 *
 *
 * This command should only be used in voltage control mode.
fun feedforwardCharacterization(): Command {
    val velocitySamples: MutableList<Double> = LinkedList<Double>()
    val voltageSamples: MutableList<Double> = LinkedList<Double>()
    val timer = Timer()

    return Commands.sequence( // Reset data
        Commands.runOnce(
            {
                velocitySamples.clear()
                voltageSamples.clear()
            }),  // Allow modules to orient

        Commands.run({ Drive.runCharacterization(0.0) }, Drive).withTimeout(FF_START_DELAY),  // Start timer

        Commands.runOnce({ timer.restart() }),  // Accelerate and gather data

        Commands.run(
            {
                val voltage = timer.get() * FF_RAMP_RATE
                Drive.runCharacterization(voltage)
                velocitySamples.add(Drive.fFCharacterizationVelocity)
                voltageSamples.add(voltage)
            },
            Drive
        ) // When cancelled, calculate and print results

            .finallyDo(
                Runnable {
                    val n = velocitySamples.size
                    var sumX = 0.0
                    var sumY = 0.0
                    var sumXY = 0.0
                    var sumX2 = 0.0
                    for (i in 0..<n) {
                        velocitySamples[i] = velocitySamples[i] + velocitySamples[i]
                        voltageSamples[i] = voltageSamples[i] + voltageSamples[i]
                        sumXY += velocitySamples[i] * voltageSamples[i]
                        sumX2 += velocitySamples[i] * velocitySamples[i]
                    }
                    val kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX)
                    val kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX)

                    val formatter: NumberFormat = DecimalFormat("#0.00000")
                    println("********** Drive FF Characterization Results **********")
                    println("\tkS: " + formatter.format(kS))
                    println("\tkV: " + formatter.format(kV))
                })
    )
}*/

fun Drive.slipCurrentTest(): Command {
    val timer = edu.wpi.first.wpilibj.Timer()
    return runCommand(Drive) {
        val volts = timer.get() * 0.01
        Logger.recordOutput("Test/StaterCurrent", Drive.modules.first().driveMotor.statorCurrent.valueAsDouble)
        Logger.recordOutput("Test/Volts", volts)
        Logger.recordOutput("Test/Velocity", Drive.modules.first().driveMotor.velocity.valueAsDouble)
        Drive.driveVoltage(ChassisSpeeds(volts, 0.0, 0.0))
    }.beforeRun() {
        timer.restart()
    }
}

fun Drive.questOffsetTest(): Command {
    val allPoints = mutableListOf<Pose2d>()
    return run {
        val questPose = questPose
        println(Drive.questPose)
        allPoints.add(questPose)
        Drive.driveVelocity(ChassisSpeeds(0.0.feetPerSecond, 0.0.feetPerSecond, 18.0.degreesPerSecond))
    }.withTimeout(20.0).andThen(runOnce {
        val sampleSize = allPoints.size
        var averagePoint = Translation2d()
        allPoints.forEach {
            averagePoint += it.translation / sampleSize.toDouble()
        }
        var radius = 0.0
        allPoints.forEach {
            radius += (it.translation - averagePoint).norm / sampleSize.toDouble()
        }
        println("quest distance from center: ${radius.meters.asFeet} feet. averagePoint: $averagePoint")
        val questTransform = Translation2d(radius, Drive.heading)
        println("quest transform: (±${questTransform.x.meters.asFeet}, ±${questTransform.y.meters.asFeet}) feet")




    }.beforeWait(2.0))
}