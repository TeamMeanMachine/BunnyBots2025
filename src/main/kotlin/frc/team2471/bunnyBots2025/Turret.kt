package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.CANdi
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.magnetSensorOffset
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.remoteCANCoder
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.asRotations
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.radians
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
    val pivotMotor = TalonFX(Falcons.PIVOT)
    val candi = CANdi(CANSensors.CANDI)
    val pivotEncoder = CANcoder(CANCoders.PIVOT)


    val rawLampreyAngle: Angle
        get() = candi.pwM1Position.valueAsDouble.IEEErem(1.0).rotations.wrap()

    val defaultTurretEncoderOffset = 0.0
    val defaultPivotEncoderOffset = 0.0

    var lampreyEncoderOffset: Double = turretEncoderOffsetEntry.getDouble(defaultTurretEncoderOffset)
    val pivotEncoderOffset: Double = pivotEncoderOffsetEntry.getDouble(defaultPivotEncoderOffset)

    val unCorrectedLampreyAngle: Angle
        get() = (rawLampreyAngle - lampreyEncoderOffset.degrees)

    val lampreyAlignmentOffset: Angle
        get() = abs((sin(unCorrectedLampreyAngle.asRadians) * 0.0)).degrees

    val turretEncoderAngle: Angle
        get() = (unCorrectedLampreyAngle - lampreyAlignmentOffset)
    val turretEncoderFieldCentricAngle: Angle
        get() = turretEncoderAngle + Drive.heading.measure

    val turretMotorAngle: Angle
        get() = turretMotor.position.valueAsDouble.rotations
    val turretMotorFieldCentricAngle: Angle
        get() = turretMotorAngle + Drive.heading.measure

    val pivotAngle: Angle
        get() = pivotMotor.position.valueAsDouble.rotations

    val turretFeedforward: Double
        get() = 0.0//Drive.speeds.omegaRadiansPerSecond.radians.asRotations * 0.01

    var turretSetpoint: Angle = turretMotorAngle
        set(value) {
            field = value.unWrap(turretMotorAngle)
            turretMotor.setControl(PositionVoltage(field.asRotations))
        }
    var turretFieldCentricSetpoint: Angle
        get() = turretSetpoint - Drive.heading.measure
        set(value) {
            turretSetpoint = value + Drive.heading.measure
        }

    var pivotSetpoint: Angle = pivotAngle
        set(value) {
            field = value.coerceIn(0.0.degrees, 90.0.degrees)
            pivotMotor.setControl(PositionVoltage(field.asRotations))
        }


    init {
        turretMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(false)
            coastMode()
        }
        turretMotor.addFollower(Falcons.TURRET_1)

        pivotEncoder.applyConfiguration {
            inverted(false)
            magnetSensorOffset(pivotEncoderOffset)
        }
        pivotMotor.applyConfiguration {

            currentLimits(30.0, 40.0, 1.0)
            inverted(false)
            brakeMode()
            remoteCANCoder(pivotEncoder, 1.0)
        }

        turretMotor.setPosition(turretEncoderAngle.asRotations)
    }

    override fun periodic() {
        Logger.recordOutput("Turret/lampreyAlignmentOffset", lampreyAlignmentOffset.asDegrees)
        Logger.recordOutput("Turret/rawLampreyAngle", rawLampreyAngle.asDegrees)
        Logger.recordOutput("Turret/unCorrectedLampreyAngle", unCorrectedLampreyAngle.asDegrees)
        Logger.recordOutput("Turret/turretEncoderAngle", turretEncoderAngle.asDegrees)
        Logger.recordOutput("Turret/turretEncoderFieldCentricAngle", turretEncoderFieldCentricAngle.asDegrees)
        Logger.recordOutput("Turret/turretMotorAngle", turretMotorAngle.asDegrees)
        Logger.recordOutput("Turret/turretMotorFieldCentricAngle", turretMotorFieldCentricAngle.asDegrees)

    }


    fun aimFieldCentricWithJoystick(): Command = runCommand {
        val joystickTranslation = Translation2d(OI.driverController.rightY, OI.driverController.rightX)
        val wantedAngle = joystickTranslation.angle.measure
        turretFieldCentricSetpoint = wantedAngle
    }

    fun aimAtGoal(): Command = runCommand {
        turretSetpoint = unCorrectedLampreyAngle
    }

}