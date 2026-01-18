package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Relay
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.ctre.a
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.d
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.ctre.v
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asVolts
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.rotationsPerSecond
import org.team2471.frc.lib.units.seconds
import org.team2471.frc.lib.units.volts
import org.team2471.frc.lib.units.voltsPerSecond
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.times


object Shooter: SubsystemBase("Shooter") {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")

    val shootingVelocityEntry = table.getEntry("Shooting Velocity")

    val shootingVelocity get() = shootingVelocityEntry.getDouble(26.0)

    val shooterMotorRight = TalonFX(Falcons.SHOOTER_RIGHT, CANivores.TURRET_CAN)
    val shooterMotorLeft = TalonFX(Falcons.SHOOTER_LEFT, CANivores.TURRET_CAN)

    var ramping = false

    @get:AutoLogOutput(key = "Shooter/Shooter Motor Right")
    val motorSpeedRight
        get() = shooterMotorRight.velocity.valueAsDouble.rotationsPerSecond

    @get:AutoLogOutput(key = "Shooter/Shooter Motor Left")
    val motorSpeedLeft
        get() = shooterMotorLeft.velocity.valueAsDouble.rotationsPerSecond

    @get:AutoLogOutput
    var rightRpmSetpoint = 0.0
        set(value) {
            shooterMotorRight.setControl(VelocityVoltage(value))
            field =value
        }

    @get:AutoLogOutput
    var leftRpmSetpoint = 0.0
        set(value) {
            shooterMotorLeft.setControl(VelocityVoltage(value))
            field =value
        }

    @get:AutoLogOutput
    var shooterSetpoint = 0.0
        set(value) {
            rightRpmSetpoint = value
            leftRpmSetpoint = value
            field =value
        }

//    val testCurves = generateShooterCurve()
    val angleCurve = InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(2.0, 66.5427839415196)
        put(2.5, 61.89516823557782)
        put(3.0, 57.43114556161717)
        put(3.5, 53.43415109297916)
        put(4.0, 49.83345882618576)
        put(4.5, 46.60900197905791)
        put(5.0, 43.68144056106794)
    }

    val rpsCurve =  InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(2.0, 23.115933377035795)
        put(2.5, 23.931349315196634)
        put(3.0, 25.186739230395112)
        put(3.5, 26.513073298392694)
        put(4.0, 27.95678922097524)
        put(4.5, 29.29342702028068)
        put(5.0, 30.880209260312633)
    }


    init {

        if (!shootingVelocityEntry.exists()) shootingVelocityEntry.setDouble(26.0)

        shooterMotorRight.applyConfiguration {
            currentLimits(25.0,40.0,1.0)
            inverted(true)
            coastMode()

            p(0.1746)
            d(0.0)
            a(0.0413)
            v(0.117)
            s(0.177, StaticFeedforwardSignValue.UseVelocitySign)
        }
        shooterMotorLeft.applyConfiguration {
            currentLimits(25.0,40.0,1.0)
            inverted(false)
            coastMode()

            p(0.1748)
            d(0.0)
            a(0.0417)
            v(0.114)
            s(0.09, StaticFeedforwardSignValue.UseVelocitySign)
        }
    }

    fun shoot() {
        shooterSetpoint = shootingVelocity
        ramping = true
    }

    fun stop() {
        shooterSetpoint = 0.0
        ramping = false
    }


    val sysIDLeftRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            1.0.voltsPerSecond,
            7.0.volts,
            5.0.seconds
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdShooterLeft_State", state.toString())
            Logger.recordOutput("SysIdShooterLeft_State", state.toString())
            Logger.recordOutput("Shooter_Left_Position", shooterMotorLeft.position.valueAsDouble)
            Logger.recordOutput("Shooter_Left_Velocity", shooterMotorLeft.velocity.valueAsDouble)
        },
        SysIdRoutine.Mechanism({ output: Voltage ->
            shooterMotorLeft.setControl(VoltageOut(output.asVolts))
            /* also log the requested output for SysId */
            Logger.recordOutput("Shooter_Left_Voltage", output.asVolts + 0.0001 * Math.random())
        }, null, this)
    )

    val sysIDRightRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            1.0.voltsPerSecond,
            7.0.volts,
            12.0.seconds
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdShooterRight_State", state.toString())
            Logger.recordOutput("SysIdShooterRight_State", state.toString())
            Logger.recordOutput("Shooter_Right_Position", shooterMotorRight.position.valueAsDouble)
            Logger.recordOutput("Shooter_Right_Velocity", shooterMotorRight.velocity.valueAsDouble)
        },
        SysIdRoutine.Mechanism({ output: Voltage ->
            shooterMotorRight.setControl(VoltageOut(output.asVolts))
            /* also log the requested output for SysId */
            Logger.recordOutput("Shooter_Right_Voltage", output.asVolts + 0.0001 * Math.random())
        }, null, this)
    )

    //                              angle,                                    speed
    fun generateShooterCurve(): Pair<InterpolatingTreeMap<Double, Double>, InterpolatingTreeMap<Double, Double>> {
//                         meters,       m/s,  degrees
        val speedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble())
        val angleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble())
        for (i in 2..20) {
            val dist = i.toDouble().feet.asMeters
            val angleAndSpeed = getAngleAndRPS(dist)
            if (angleAndSpeed.first.isFinite() && !angleAndSpeed.first.isNaN() && angleAndSpeed.second.isFinite() && !angleAndSpeed.second.isNaN()) {
                angleCurve.put(dist, angleAndSpeed.first)
                speedCurve.put(dist, angleAndSpeed.second)
            }
        }

        return Pair(angleCurve, speedCurve)
    }

    // Performs newtons method in 2 dimensions to estimate
    fun getAngleAndRPS(distFromGoalM: Double): Pair<Double, Double> {

        val goalTime = 0.85

        val maxTError = 0.1
        val maxDError = 0.1

        // todo account for turret offset
        val toTarget: Translation2d = Translation2d(distFromGoalM, 64.0.inches.asMeters - 0.4)
//        println("ToTarger: ${toTarget}")

        // in vx and vy, will convert to angle and velocity at the end
        // this is derived from kinematic equations and assumes no drag
        var guess = Pair(toTarget.x / goalTime, (toTarget.y + 0.5 * g * goalTime.pow(2)) / goalTime)
        var guessIncremented = Pair(guess.first + 0.1, guess.second + 0.1)

//        println("Guesses")
//        println("Initial guess: ${guess.first.round(4)}, ${guess.second.round(4)}")

        var errors = calcFuelError(guess.first, guess.second, toTarget, goalTime)
        var errorSum = errors.first.absoluteValue + errors.second.absoluteValue

//        println("error 1")

        var errorsIncremented = calcFuelError(guessIncremented.first, guessIncremented.second, toTarget, goalTime)
        var errorSumIncremented = errorsIncremented.first.absoluteValue + errorsIncremented.second.absoluteValue


//        println("Calculating velocities")
//        println("Initial guess: ${guess.first.round(4)}, ${guess.second.round(4)}")
//        println("Initial error: ${errors.first.round(4)}, ${errors.second.round(4)}")
//        println("Errorsum: ${errorSum}\n")

        var num = 0
        while ((errors.first.absoluteValue > maxDError || errors.second.absoluteValue > maxTError) && num <= 5) {
            //guess is x1 & y1, errorsum is z1
            //guessincremented is x1 & y1, errorsumincremented is z1
            /*First, calculate a line between the two points in parametric form.
                x=x1+(x2-x1)t
                y=y1+(y2-y1)t
                z=z1+(z2-z1)t

                where 0<=t<=1

                I just need to store x and y
             */
            val x1 = guess.first
            val y1 = guess.second
            val x2 = guessIncremented.first
            val y2 = guessIncremented.second

//            println("($x1,$y1,$errorSum)")
//            println("($x2,$y2,$errorSumIncremented)")

            // Then find the t value for which z is 0
            //  t = -z1       / (           z2       -  z1      )
            val t = -errorSum / (errorSumIncremented - errorSum)

            guess = Pair(x1+(x2-x1) * t, y1+(y2-y1) * t)

            guessIncremented = Pair(guess.first + 0.1, guess.second + 0.1)

            errors = calcFuelError(guess.first, guess.second, toTarget, goalTime)
            errorSum = errors.first.absoluteValue + errors.second.absoluteValue

            errorsIncremented = calcFuelError(guessIncremented.first, guessIncremented.second, toTarget, goalTime)
            errorSumIncremented = errorsIncremented.first.absoluteValue + errorsIncremented.second.absoluteValue

//            println("guess: ${guess.first.round(4)}, ${guess.second.round(4)}")
//            println("error: ${errors.first.round(4)}, ${errors.second.round(4)}")
//            println("Errorsum: ${errorSum}\n")

            num++
        }
//        println("Dist: ${distFromGoalM}, ErrorSum: ${errorSum}, iterations: ${num}")

        val mps = sqrt(guess.first.pow(2) + guess.second.pow(2))

        println("Dist: ${distFromGoalM}, Angle: ${atan2(guess.second, guess.first).radians.asDegrees}, RPS: ${calculateRPS(mps)}")
        return Pair(atan2(guess.second, guess.first).radians.asDegrees, calculateRPS(mps))
    }

    //                    m/s
    fun calculateRPS(speed: Double): Double {
        val efficiency = 0.85
        val wheelDiameter = 4.0.inches.asMeters

        return speed / (efficiency * Math.PI * wheelDiameter)
    }
}