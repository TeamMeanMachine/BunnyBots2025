package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.CANdi
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ControlModeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.bunnyBots2025.Vision.TURRET_TO_ROBOT_IN
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.*
import org.team2471.frc.lib.ctre.loggedTalonFX.LoggedTalonFX
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.DynamicInterpolatingTreeMap
import org.team2471.frc.lib.util.angleTo
import java.lang.reflect.Field
import kotlin.compareTo
import kotlin.math.IEEErem
import kotlin.math.abs
import kotlin.math.absoluteValue
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


    val turretMotor = LoggedTalonFX(Falcons.TURRET_0)
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

    // ty -> rps??
    val shooterRPMCurve = InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(2.5, 19.0)
        put(-0.1, 20.0)
        put(-3.0, 26.0)
        put(-4.8, 26.0)
        put(-6.9, 30.0)
        put(-9.25, 35.0)
    }

    // ty -> degreesz
    val shooterPitchCurve = InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(2.5, 50.0)
        put(-0.1, 50.0)
        put(-3.0, 48.5)
        put(-4.8, 34.9)
        put(-6.9, 34.9)
        put(-9.25, 32.7)
    }

    val horizontalOffsetEntry = table.getEntry("Horizontal Offset")
    val horizontalOffset get() = horizontalOffsetEntry.getDouble(0.0)

    // ty -> degrees
    val horizontalOffsetCurve = InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(-0.1, 0.0)
        put(2.5, 0.0)
        put(-3.0, 0.0)
        put(-4.8, 0.0)
        put(-6.9, -2.5)
        put(-9.25, -2.5)
    }

    val joystickAimFilter = LinearFilter.singlePoleIIR(0.2, 0.02)


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

    val turretMotorAngleHistory = DynamicInterpolatingTreeMap(
        InverseInterpolator.forDouble(), Interpolator.forDouble(), 75)

    @get:AutoLogOutput(key = "Turret/turretMotorFieldCentricAngle")
    val turretMotorFieldCentricAngle: Angle
        get() = turretMotorAngle + Drive.heading.measure

    @get:AutoLogOutput(key = "Turret/pivotAngle")
    val pivotAngle: Angle
        get() = pivotMotor.position.valueAsDouble.rotations

    @get:AutoLogOutput(key = "Turret/turretFeedforward")
    val turretFeedforward: Double
        get() = -Drive.speeds.omegaRadiansPerSecond.radians.asRotations * 10.0

    @get:AutoLogOutput(key = "Turret/turretSetpoint")
    var turretSetpoint: Angle = turretMotorAngle
        set(value) {
            field = value.unWrap(turretMotorAngle)
            if ((turretMotorAngle - field).asDegrees.absoluteValue < 90.0) {
                turretMotor.setControl(PositionVoltage(field.asRotations).withFeedForward(turretFeedforward))
            } else {
                turretMotor.setControl(MotionMagicVoltage(field.asRotations).withFeedForward(turretFeedforward))
            }
        }

    @get:AutoLogOutput(key = "Turret/turretFieldCentricSetpoint")
    var turretFieldCentricSetpoint: Angle
        get() = turretSetpoint + Drive.heading.measure
        set(value) {
            turretSetpoint = -Drive.heading.measure - value
        }

    @get:AutoLogOutput(key = "Turret/turretSetpointError")
    val turretSetpointError: Angle
        get() = turretSetpoint - turretMotorAngle

    @get:AutoLogOutput(key = "Turret/turretSetpointErrorMotor")
    val turretSetpointErrorMotor: Angle
        get() = turretMotor.closedLoopError.valueAsDouble.rotations

    @get:AutoLogOutput(key = "Turret/turretMotorVoltage")
    val turretMotorVoltage: Double
        get() = turretMotor.motorVoltage.valueAsDouble

    @get:AutoLogOutput(key = "Turret/pivotEncoderAngle")
    val pivotEncoderAngle get() = pivotEncoder.position.valueAsDouble.rotations + pivotEncoderOffset.degrees

    @get:AutoLogOutput(key = "Turret/pivotSetpoint")
    var pivotSetpoint: Angle = pivotAngle
        set(value) {
            field = value.coerceIn(20.0.degrees, 60.0.degrees)
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

        horizontalOffsetEntry.setDouble(0.0)

        turretMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(true)
            coastMode()
            s(0.13, StaticFeedforwardSignValue.UseClosedLoopSign)
            p(50.0)



            Feedback.SensorToMechanismRatio = 1.0 / (10.0 / 233.0)
            motionMagic(2.1, 12.2)
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
        turretMotorAngleHistory.put(Timer.getFPGATimestamp(), turretMotorAngle.asDegrees)
        // Are the motors running position control loops? Update the custom feedforward
        if (turretMotor.controlMode.value == ControlModeValue.MotionMagicVoltage || turretMotor.controlMode.value == ControlModeValue.PositionVoltage) {
//            println("running ff")
            turretSetpoint = turretSetpoint
        }
        if (pivotMotor.controlMode.value == ControlModeValue.PositionDutyCycle) {
            pivotSetpoint = pivotSetpoint
        }
    }


    fun aimFieldCentricWithJoystick(): Command = runCommand(this) {
        val joystickTranslation = Translation2d(-OI.driverController.rightY, OI.driverController.rightX)
        if (joystickTranslation.norm > 0.25) {
            val wantedAngle = joystickTranslation.angle.measure
            Logger.recordOutput("Turret/wantedAngle", wantedAngle)
            val setpointAngle = (wantedAngle.unWrap(turretMotorFieldCentricAngle))
            Logger.recordOutput("Turret/joystickSetpointAngle", setpointAngle)
            turretFieldCentricSetpoint = setpointAngle
        }
    }

    fun aimAtGoal(): Command = runCommand(this) {
//        turretFieldCentricSetpoint = 0.0.degrees
//
//        val turretPos = if (Vision.rawLimelightPose != Pose2d()) Vision.rawLimelightPose.translation else Drive.pose.translation - Translation2d(
//            TURRET_TO_ROBOT_IN.inches,
//            0.0.inches
//        ).rotateBy(Drive.heading.measure.asRotation2d)
////
//        turretFieldCentricSetpoint = -turretPos.angleTo(FieldManager.goalPose)
        val camError = Vision.aimError2d
        var ty = 0.0
        if (camError != null && !Vision.hasJitteryTag && Vision.seeTags) {
            val tagDistance = Vision.tagDistance
            val aimError = atan((tagDistance.asInches * tan(camError)) / (tagDistance.asInches - 6.0)) + /*horizontalOffset.degrees*/ if (Vision.filteredTy <= 50.0) horizontalOffsetCurve.get(Vision.filteredTy).degrees else 0.0.degrees
            Logger.recordOutput("Turret/aimError", aimError)
            turretSetpoint = (turretMotorAngleHistory.get(Vision.inputs.aprilTagTimestamp)?.degrees ?: turretMotorAngle) - aimError
            ty = Vision.filteredTy
        } else {
            val turretPos = Vision.poseEstimator.estimatedPosition.translation - Translation2d(
            TURRET_TO_ROBOT_IN.inches,
            0.0.inches
            ).rotateBy(Drive.heading.measure.asRotation2d)

            turretFieldCentricSetpoint = /*horizontalOffset.degrees*/if (Vision.filteredTy <= 50.0) horizontalOffsetCurve.get(Vision.filteredTy).degrees else 0.0.degrees-turretPos.angleTo(FieldManager.goalPose)

            ty = Vision.distanceToTy(turretPos.getDistance(FieldManager.goalPose).meters).asDegrees
        }

        if (ty <= 50.0) {
            pivotSetpoint = shooterPitchCurve.get(ty).degrees
            Shooter.leftRpmSetpoint = shooterRPMCurve.get(ty).coerceIn(19.0, 35.0)
            Shooter.rightRpmSetpoint = shooterRPMCurve.get(ty).coerceIn(19.0, 35.0)
        }
    }

    fun turretBrakeMode() {
        turretMotor.brakeMode()
    }
    fun turretCoastMode() {
        turretMotor.coastMode()
    }

}