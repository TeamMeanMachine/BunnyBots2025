package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.CANdi
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ControlModeValue
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
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
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.*
import org.team2471.frc.lib.ctre.loggedTalonFX.LoggedTalonFX
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.math.DynamicInterpolatingTreeMap
import org.team2471.frc.lib.util.angleTo
import kotlin.math.IEEErem
import kotlin.math.abs
import kotlin.math.absoluteValue
import org.team2471.frc.lib.coroutines.periodic
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


    val turretMotor = LoggedTalonFX(Falcons.TURRET_0, CANivores.TURRET_CAN)
    val pivotMotor = TalonFX(Falcons.PIVOT, CANivores.TURRET_CAN)
    val candi = CANdi(CANSensors.CANDI, CANivores.TURRET_CAN)
    val pivotEncoder = CANcoder(CANCoders.PIVOT, CANivores.TURRET_CAN)
    val turretPigeon = Pigeon2(CANSensors.TURRET_PIGEON, CANivores.TURRET_CAN)


    @get:AutoLogOutput(key = "Turret/rawLampreyAngle")
    val rawLampreyAngle: Angle
        get() = candi.pwM1Position.valueAsDouble.IEEErem(1.0).rotations.wrap()

    val defaultTurretEncoderOffset = -128.496
    val defaultPivotEncoderOffset = -38.75

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
        put(-1.5, 22.0)
        put(-3.0, 27.0)
        put(-4.0, 29.0)
        put(-5.0, 30.0)
        put(-6.0, 33.0)
        put(-7.0, 33.0)
        put(-7.5, 55.0)
    }

    // ty -> degrees
    val shooterPitchCurve = InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(2.5, 50.0)
        put(-0.1, 50.0)
        put(-1.5, 50.0)
        put(-3.0, 50.0)
        put(-4.0, 50.0)
        put(-5.0, 50.0)
        put(-6.0, 50.0)
        put(-7.0, 50.0)
        put(-7.5, 45.0)
    }

    val horizontalOffsetEntry = table.getEntry("Horizontal Offset")
    val horizontalOffset get() = horizontalOffsetEntry.getDouble(0.0)

    // ty -> degrees
    val horizontalOffsetCurve = InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble()).apply {
        put(2.5, 0.0)
        put(-0.1, 0.0)
        put(-1.5, -1.0)
        put(-3.0, -2.5)
        put(-4.0, -3.0)
        put(-5.0, -2.0)
        put(-6.0, -1.0)
    }

    val AUTO_HORIZONTAL_OFFSET = 4.0

    val joystickAimFilter = LinearFilter.singlePoleIIR(0.2, 0.02)


    @get:AutoLogOutput(key = "Turret/unCorrectedLampreyAngle")
    val unCorrectedLampreyAngle: Angle
        get() = (rawLampreyAngle - lampreyEncoderOffset.degrees)

    @get:AutoLogOutput(key = "Turret/lampreyAlignmentOffset")
    val lampreyAlignmentOffset: Angle
        get() = abs((sin(unCorrectedLampreyAngle.asRadians) * 0.0)).degrees

    @get:AutoLogOutput(key = "Turret/turretEncoderAngle")
    val turretEncoderAngle: Angle
        get() = (unCorrectedLampreyAngle - lampreyAlignmentOffset).wrap()

    @get:AutoLogOutput(key = "Turret/turretEncoderFieldCentricAngle")
    val turretEncoderFieldCentricAngle: Angle
        get() = (turretEncoderAngle + Drive.heading.measure).wrap()

    @get:AutoLogOutput(key = "Turret/turretMotorRotorFieldCentricAngle")
    val turretMotorRotorFieldCentricAngle: Angle
        get() = ((turretMotor.rotorPosition.valueAsDouble * (10.0 / 58.0 / 234.0)).rotations + Drive.heading.measure).wrap()

    @get:AutoLogOutput(key = "Turret/turretMotorAngle")
    val turretMotorAngle: Angle
        get() = (turretMotorFieldCentricAngle - Drive.heading.measure).wrap()//turretMotor.position.valueAsDouble.rotations

    val turretMotorAngleHistory = DynamicInterpolatingTreeMap(
        InverseInterpolator.forDouble(), Interpolator.forDouble(), 75)

    @get:AutoLogOutput(key = "Turret/turretMotorFieldCentricAngle")
    val turretMotorFieldCentricAngle: Angle
        get() = turretMotor.position.valueAsDouble.rotations//turretMotorAngle + Drive.heading.measure

    @get:AutoLogOutput(key = "Turret/pivotAngle")
    val pivotAngle: Angle
        get() = pivotMotor.position.valueAsDouble.rotations

    @get:AutoLogOutput(key = "Turret/turretFeedforward")
    val turretFeedforward: Double
        get() = -Drive.speeds.omegaRadiansPerSecond.radians.asRotations * 2.5

    @get:AutoLogOutput(key = "Turret/turretSetpoint")
    var turretSetpoint: Angle// = turretMotorAngle
        get() = turretFieldCentricSetpoint - Drive.heading.measure
        set(value) {
            turretFieldCentricSetpoint = (Drive.heading.measure + value).wrap()
        }

    val TURRET_TOP_LIMIT = 210.0.degrees
    val TURRET_BOTTOM_LIMIT = -210.0.degrees

    val rawTurretMotorRotorAngle: Angle
        get() = turretMotor.rotorPosition.valueAsDouble.rotations / 24.0

    @get:AutoLogOutput(key = "Turret/turretMotorRotorAngleOffset")
    var turretMotorRotorPositionOffset: Angle = 0.0.degrees

    @get:AutoLogOutput(key = "Turret/turretMotorRotorAngle")
    val turretMotorRotorAngle: Angle
        get() = rawTurretMotorRotorAngle + turretMotorRotorPositionOffset

    var turretWrapping = false

    @get:AutoLogOutput(key = "Turret/turretFieldCentricSetpoint")
    var turretFieldCentricSetpoint: Angle = turretMotorAngle
//        get() = turretSetpoint + Drive.heading.measure
        set(value) {
//            println("setting setpoint to ${value.asDegrees}")
            val turretMotorFieldCentricAngle = this.turretMotorFieldCentricAngle
            val fieldCentricSetpoint = value.unWrap(turretMotorFieldCentricAngle)
            val positionError = fieldCentricSetpoint - turretMotorFieldCentricAngle
            val robotCentricSetpoint = turretMotorRotorAngle + positionError
            field = if (robotCentricSetpoint > TURRET_TOP_LIMIT) {
                fieldCentricSetpoint - 360.0.degrees
            } else if (robotCentricSetpoint < TURRET_BOTTOM_LIMIT) {
                fieldCentricSetpoint + 360.0.degrees
            } else {
                fieldCentricSetpoint
            }

            if ((turretMotorFieldCentricAngle - field).asDegrees.absoluteValue < 90.0) {
                turretMotor.setControl(PositionVoltage(field.asRotations).withFeedForward(turretFeedforward))
            } else {
                turretMotor.setControl(PositionVoltage(field.asRotations).withFeedForward(turretFeedforward))
            }

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
    val pivotEncoderAngle: Angle
        get() = pivotEncoder.absolutePosition.valueAsDouble.rotations + pivotEncoderOffset.degrees

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

    var tempHeadingResetAngle: Angle? = null


    init {

        if (!turretEncoderOffsetEntry.exists()) turretEncoderOffsetEntry.setDouble(defaultTurretEncoderOffset)
        if (!pivotEncoderOffsetEntry.exists()) pivotEncoderOffsetEntry.setDouble(defaultPivotEncoderOffset)

        pivotEncoderOffsetEntry.setPersistent()
        turretEncoderOffsetEntry.setPersistent()

        horizontalOffsetEntry.setDouble(0.0)

//        turretMotor.setPosition(turretEncoderAngle)

        turretMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(true)
            coastMode()
            s(0.13, StaticFeedforwardSignValue.UseClosedLoopSign)
            p(30.0)



//            Feedback.SensorToMechanismRatio = 1.0 / (10.0 / 233.0)
            motionMagic(2.1, 12.2)
            alternateFeedbackSensor(turretPigeon.deviceID, FeedbackSensorSourceValue.RemotePigeon2Yaw, (10.0 / 58.0 / 234.0))

            ClosedLoopGeneral.ContinuousWrap = false
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
        setTurretOffset(Drive.heading.measure)



        GlobalScope.launch {
            periodic {
                if (turretMotor.controlMode.value in PhoenixUtil.positionControlModes) {
                    turretFieldCentricSetpoint = turretFieldCentricSetpoint
                }
            }
        }
        GlobalScope.launch {
            periodic {
                val tempResetAngle = tempHeadingResetAngle
                if (tempResetAngle != null) {
                    tempHeadingResetAngle = null
                    Drive.headingAngleUnwrapped = tempResetAngle
                    GlobalScope.launch {
                        println("setting turret pigeon yaw")
                        turretPigeon.setYaw(turretEncoderFieldCentricAngle)
                        println("finished setting turret pigeon yaw")
                    }
                }
                Drive.headingAngleUnwrapped = Drive.heading.measure.unWrap(Drive.headingAngleUnwrapped)
            }
        }
    }

    override fun periodic() {
        turretMotorAngleHistory.put(Timer.getFPGATimestamp(), turretMotorAngle.asDegrees)

        if (pivotMotor.controlMode.value == ControlModeValue.PositionDutyCycle) {
            pivotSetpoint = pivotSetpoint
        }

        if (Robot.isDisabled) {
            turretMotorRotorPositionOffset = turretEncoderAngle - rawTurretMotorRotorAngle
//            turretPigeon.setYaw(turretEncoderFieldCentricAngle.asRotations)
        }

    }

    fun setTurretOffset(robotHeading: Angle) {
        tempHeadingResetAngle = robotHeading
    }


    fun aimFieldCentricWithJoystick(): Command = runCommand(this) {
        val joystickTranslation = Translation2d(-OI.driverController.rightY, OI.driverController.rightX)
        if (joystickTranslation.norm > 0.25) {
            val wantedAngle = joystickTranslation.angle.measure
            Logger.recordOutput("Turret/wantedAngle", wantedAngle)
            val setpointAngle = (wantedAngle)
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
//        val camError = Vision.aimError2d
//        var ty = 0.0
//        if (camError != null /*&& !Vision.hasJitteryTag*/ && Vision.seeTags) {
//            val tagDistance = Vision.tagDistance
//            var aimError =
//                atan((tagDistance.asInches * tan(camError)) / (tagDistance.asInches - 6.0)) + /*horizontalOffset.degrees*/ if (Vision.filteredTy <= 50.0) horizontalOffsetCurve.get(
//                    Vision.filteredTy
//                ).degrees else 0.0.degrees
//
//            if (Robot.isAutonomousEnabled) {
//                aimError += AUTO_HORIZONTAL_OFFSET.degrees * sign(Drive.velocity.y.asMetersPerSecond)
//            }
//
//            Logger.recordOutput("Turret/aimError", aimError)
//            turretSetpoint =
//                (turretMotorAngleHistory.get(Vision.inputs.aprilTagTimestamp)?.degrees ?: turretMotorAngle) - aimError
//            ty = Vision.filteredTy
////            pivotSetpoint = shooterPitchCurve.get(ty).degrees
////            Shooter.leftRpmSetpoint = shooterRPMCurve.get(ty).coerceIn(19.0, 35.0)
////            Shooter.rightRpmSetpoint = shooterRPMCurve.get(ty).coerceIn(19.0, 35.0)
//        } else if (Vision.hasSeenTag && Vision.pose.translation != Translation2d()) {
            val turretPos = Vision.pose.translation + Drive.velocity * 0.85 - Translation2d(
            TURRET_TO_ROBOT_IN.inches,
            0.0.inches
            ).rotateBy(Drive.heading.measure.asRotation2d)

            Logger.recordOutput("Aimpoint", FieldManager.rebuiltBlueGoalPose)
            Logger.recordOutput("TurretPos", Pose2d(turretPos, turretMotorFieldCentricAngle.asRotation2d))

            turretFieldCentricSetpoint = /*horizontalOffset.degrees*//*(if (Vision.filteredTy <= 50.0) horizontalOffsetCurve.get(Vision.filteredTy).degrees else 0.0.degrees) -*/ turretPos.angleTo(FieldManager.rebuiltBlueGoalPose)

        val distToGoal = turretPos.getDistance(FieldManager.rebuiltBlueGoalPose)
//            val ty = Vision.distanceToTy(turretPos.getDistance(FieldManager.goalPose).meters).asDegrees
//        }

//        if (ty <= 50.0 && Vision.hasSeenTag) {
//            pivotSetpoint = shooterPitchCurve.get(ty).degrees
//            Shooter.leftRpmSetpoint = shooterRPMCurve.get(ty).coerceIn(19.0, 35.0)
//            Shooter.rightRpmSetpoint = shooterRPMCurve.get(ty).coerceIn(19.0, 35.0)
//        }

        Logger.recordOutput("GoalDist", distToGoal)
        if (OI.driverController.a().asBoolean) {
            pivotSetpoint = Shooter.angleCurve.get(distToGoal).degrees
            Shooter.leftRpmSetpoint = Shooter.rpsCurve.get(distToGoal)
            Shooter.rightRpmSetpoint = Shooter.rpsCurve.get(distToGoal)

        }
    }

    fun  aimAt90(): Command = runCommand(this) {
        turretFieldCentricSetpoint = 90.0.degrees
    }


    fun turretBrakeMode() {
        turretMotor.brakeMode()
    }
    fun turretCoastMode() {
        turretMotor.coastMode()
    }

}