package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.CANdi
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ControlModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.PhoenixUtil
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.magnetSensorOffset
import org.team2471.frc.lib.ctre.motionMagic
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.remoteCANCoder
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.asRotations
import org.team2471.frc.lib.units.cos
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.units.unWrap
import org.team2471.frc.lib.units.wrap
import kotlin.math.IEEErem
import kotlin.math.abs
import kotlin.math.sin


/*
Should always be aiming at the goal by default (if nothing special is going on)
    Turret aiming at the goal should be done by using tag ty, tx on a pid, (or calculating an angle setpoint, whatever works best, pid works even when motor position is off).
    Pivot aiming based only on tag size.
    If it loses the tag, it should use trig or pose estimation to "best guess" remember where it was.
Has a function that uses the subsystem that allows for a joystick to manually aim, field centric.
    While this is happening, it is still looking for the apriltags on the goal, if it gets a better view of the tags, it should update its prediction on where goal is.
Predict location of goal only based on delta odometry and delta turret angle, this will reduce the requirement for an accurate absolute encoder.
 */

object Turret: SubsystemBase("Turret") {
    private val table = NetworkTableInstance.getDefault().getTable("Turret")
    private val turretEncoderOffsetEntry = table.getEntry("Turret Encoder Offset")
    private val pivotEncoderOffsetEntry = table.getEntry("Pivot Encoder Offset")


    val turretMotor = TalonFX(Falcons.TURRET_0)
    val pivotMotor = TalonFX(Falcons.PIVOT, CANivores.TURRET_CAN)
    val candi = CANdi(CANSensors.CANDI, CANivores.TURRET_CAN)
    val pivotEncoder = CANcoder(CANCoders.PIVOT, CANivores.TURRET_CAN)


    @get:AutoLogOutput(key = "Turret/rawLampreyAngle")
    val rawLampreyAngle: Angle
        get() = candi.pwM1Position.valueAsDouble.IEEErem(1.0).rotations.wrap()

    val defaultTurretEncoderOffset = 0.0
    val defaultPivotEncoderOffset = 0.0

    var lampreyEncoderOffset: Double = turretEncoderOffsetEntry.getDouble(defaultTurretEncoderOffset)
    val pivotEncoderOffset: Double = pivotEncoderOffsetEntry.getDouble(defaultPivotEncoderOffset)

    @get:AutoLogOutput(key = "Turret/unCorrectedLampreyAngle")
    val unCorrectedLampreyAngle: Angle
        get() = (rawLampreyAngle - lampreyEncoderOffset.degrees)

    @get:AutoLogOutput(key = "Turret/lampreyAlignmentOffset")
    val lampreyAlignmentOffset: Angle
        get() = abs((sin(unCorrectedLampreyAngle.asRadians) * 0.0)).degrees

    @get:AutoLogOutput(key = "Turret/turretEncoderAngle")
    val turretEncoderAngle: Angle
        get() = (unCorrectedLampreyAngle - lampreyAlignmentOffset)
    @get:AutoLogOutput(key = "Turret/turretEncoderFieldCentricAngle")
    val turretEncoderFieldCentricAngle: Angle
        get() = turretEncoderAngle + Drive.heading.measure

    @get:AutoLogOutput(key = "Turret/turretMotorAngle")
    val turretMotorAngle: Angle
        get() = turretMotor.position.valueAsDouble.rotations
    @get:AutoLogOutput(key = "Turret/turretMotorFieldCentricAngle")
    val turretMotorFieldCentricAngle: Angle
        get() = turretMotorAngle + Drive.heading.measure

    @get:AutoLogOutput(key = "Turret/pivotAngle")
    val pivotAngle: Angle
        get() = pivotMotor.position.valueAsDouble.rotations

    @get:AutoLogOutput(key = "Turret/turretFeedforward")
    val turretFeedforward: Double
        get() = 0.0//Drive.speeds.omegaRadiansPerSecond.radians.asRotations * 0.01

    @get:AutoLogOutput(key = "Turret/turretSetpoint")
    var turretSetpoint: Angle = turretMotorAngle
        set(value) {
            field = value.unWrap(turretMotorAngle)
            turretMotor.setControl(MotionMagicVoltage(field.asRotations))
        }
    @get:AutoLogOutput(key = "Turret/turretFieldCentricSetpoint")
    var turretFieldCentricSetpoint: Angle
        get() = turretSetpoint - Drive.heading.measure
        set(value) {
            turretSetpoint = value + Drive.heading.measure
        }

    @get:AutoLogOutput(key = "Turret/pivotEncoderAngle")
    val pivotEncoderAngle get() = pivotEncoder.position.valueAsDouble.rotations

    @get:AutoLogOutput(key = "Turret/pivotSetpoint")
    var pivotSetpoint: Angle = pivotAngle
        set(value) {
            field = value.coerceIn(0.0.degrees, 45.0.degrees)
            pivotMotor.setControl(PositionDutyCycle(field.asRotations).withFeedForward(pivotFeedForward))
        }
    @get:AutoLogOutput(key = "Turret/pivotFeedForward")
    val pivotFeedForward: Double
        get() = pivotAngle.cos() * 0.0

    var pivotPeriodicFeedforward: Boolean = true
    var turretPeriodicFeedforward: Boolean = true


    init {
        turretMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(false)
            coastMode()
            s(0.12, StaticFeedforwardSignValue.UseVelocitySign)
            p(50.0)



            Feedback.SensorToMechanismRatio = 1.0 / (10.0 / 233.0)
            motionMagic(1.2, 6.2)
        }
        turretMotor.addFollower(Falcons.TURRET_1)

        pivotEncoder.applyConfiguration {
            inverted(true)
            magnetSensorOffset(0.342041015625)
        }
        pivotMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(false)
            coastMode()
            s(0.005, StaticFeedforwardSignValue.UseVelocitySign)
            p(300.0)
            motionMagic(1.0, 0.25)
            Feedback.SensorToMechanismRatio = 216.0 * 2.0 / 1.263 * 1.0338
//            remoteCANCoder(pivotEncoder, 216.0)
        }

        turretMotor.setPosition(0.0)
        pivotMotor.setPosition(pivotEncoder.position.valueAsDouble)
    }

    override fun periodic() {
        // Are the motors running position control loops? Update the custom feedforward
        if (turretMotor.controlMode.value == ControlModeValue.MotionMagicVoltage) {
//            println("running ff")
            turretFieldCentricSetpoint = turretFieldCentricSetpoint
        }
        if (pivotMotor.controlMode.value == ControlModeValue.PositionDutyCycle) {
            pivotSetpoint = pivotSetpoint
        }
    }


    fun aimFieldCentricWithJoystick(): Command = runCommand {
        val joystickTranslation = Translation2d(-OI.driverController.rightY, OI.driverController.rightX)
        if (joystickTranslation.norm > 0.25) {
            val wantedAngle = joystickTranslation.angle.measure - Drive.heading.measure
            println("wantedAngle: ${wantedAngle.asDegrees}")
            turretFieldCentricSetpoint = wantedAngle
        }
    }

    fun aimAtGoal(): Command = runCommand {
        turretFieldCentricSetpoint = 0.0.degrees
    }

}