package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.CANdi
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ControlModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.bunnyBots2025.Vision.TURRET_TO_ROBOT_IN
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.*
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.angleTo
import java.lang.reflect.Field
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

object Turret : SubsystemBase("Turret") {
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

    val defaultTurretEncoderOffset = -128.496
    val defaultPivotEncoderOffset = 205.967

    var lampreyEncoderOffset: Double = turretEncoderOffsetEntry.getDouble(defaultTurretEncoderOffset)
        set(value) {
            turretEncoderOffsetEntry.setDouble(value)
            field = value
        }
    var pivotEncoderOffset: Double = pivotEncoderOffsetEntry.getDouble(defaultPivotEncoderOffset)
        set(value) {
            pivotEncoderOffsetEntry.setDouble(value)
            field = value
        }

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
        get() = Drive.heading.measure - turretMotorAngle

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
        get() = turretSetpoint + Drive.heading.measure
        set(value) {
            turretSetpoint = -Drive.heading.measure - value
        }

    @get:AutoLogOutput(key = "Turret/pivotEncoderAngle")
    val pivotEncoderAngle get() = pivotEncoder.position.valueAsDouble.rotations + pivotEncoderOffset.degrees

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

        if (!turretEncoderOffsetEntry.exists()) turretEncoderOffsetEntry.setDouble(defaultTurretEncoderOffset)
        if (!pivotEncoderOffsetEntry.exists()) pivotEncoderOffsetEntry.setDouble(defaultPivotEncoderOffset)

        pivotEncoderOffsetEntry.setPersistent()
        turretEncoderOffsetEntry.setPersistent()

        turretMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(true)
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

        turretMotor.setPosition(turretEncoderAngle)
        pivotMotor.setPosition(pivotEncoderAngle)
    }

    override fun periodic() {
        // Are the motors running position control loops? Update the custom feedforward
        if (turretMotor.controlMode.value == ControlModeValue.MotionMagicVoltage) {
//            println("running ff")
//            turretFieldCentricSetpoint = turretFieldCentricSetpoint
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

        val turretPos = if (Vision.rawLimelightPose != Pose2d()) Vision.rawLimelightPose.translation else Drive.pose.translation - Translation2d(
            TURRET_TO_ROBOT_IN.inches,
            0.0.inches
        ).rotateBy(Drive.heading.measure.asRotation2d)

        Logger.recordOutput("TurretPose", Pose2d(turretPos, turretEncoderFieldCentricAngle.asRotation2d))

        Logger.recordOutput("Goal Pos", Pose2d(FieldManager.goalPose, 0.0.degrees.asRotation2d  ))

        Logger.recordOutput("angle", turretPos.angleTo(FieldManager.goalPose))
//
        turretFieldCentricSetpoint = -turretPos.angleTo(FieldManager.goalPose)
    }

}