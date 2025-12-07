package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.Utils
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.units.asInches
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.atan
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.tan
import org.team2471.frc.lib.vision.limelight.VisionIO
import org.team2471.frc.lib.vision.limelight.VisionIOLimelight

object Vision : SubsystemBase() {
    val io: VisionIO = VisionIOLimelight("limelight-turret") { Turret.turretEncoderFieldCentricAngle }

    const val TURRET_TO_ROBOT_IN = 7.39

    val inputs = VisionIO.VisionIOInputs()
    var rawLimelightPose = Pose2d()


    @get:AutoLogOutput(key = "error")
    var aimError2d: Angle? = null

    @get:AutoLogOutput(key = "tag distance")
    var tagDistance: Distance = Double.MAX_VALUE.inches

    @get:AutoLogOutput(key = "ty")
    var ty: Double = 1000.0

    var oldty: Double = ty

    val tyFilter = LinearFilter.singlePoleIIR(0.1, 0.02)

    @get:AutoLogOutput(key = "seeTags")
    var seeTags = false

    private val seeTagsDebouncer = Debouncer(0.5)
    private var seeTagsRaw = false
        set(value) {
            seeTags = seeTagsDebouncer.calculate(value)
            field = value
        }

    @get:AutoLogOutput(key = "filtered ty")
    var filteredTy: Double = 1000.0

    override fun periodic() {
        io.updateInputs(inputs)

        seeTagsRaw = inputs.seesTag

        if (inputs.aprilTagPoseEstimate != Pose2d()) {
            rawLimelightPose = inputs.aprilTagPoseEstimate

            if (inputs.trimmedFiducials.size == 2) {
                val avgTx = (inputs.trimmedFiducials[0].second.first + inputs.trimmedFiducials[1].second.first) / 2.0

                aimError2d = avgTx.degrees
            } else if (inputs.trimmedFiducials.size == 1) {
                aimError2d = inputs.trimmedFiducials[0].second.first.degrees
            } else {
                aimError2d = null
            }

            Logger.recordOutput("RobotPos", Pose2d(
                rawLimelightPose.translation + Translation2d(
                    TURRET_TO_ROBOT_IN.inches,
                    0.0.inches
                ).rotateBy((Drive.headingHistory.get(inputs.aprilTagTimestamp) ?: 0.0).degrees.asRotation2d),
                Drive.heading
            ))
            Drive.addVisionMeasurement(
                Pose2d(
                    rawLimelightPose.translation + Translation2d(
                        TURRET_TO_ROBOT_IN.inches,
                        0.0.inches
                    ).rotateBy((Drive.headingHistory.get(inputs.aprilTagTimestamp) ?: 0.0).degrees.asRotation2d),
                    Drive.heading
                ),
                Utils.fpgaToCurrentTime(inputs.aprilTagTimestamp), VecBuilder.fill(0.0000001, 0.0000001, 1000000000.0)
            )
            val trimmedFiducials = inputs.trimmedFiducials
            if (trimmedFiducials.isNotEmpty()) {
                ty = trimmedFiducials[0].second.second
                oldty = ty
                filteredTy = tyFilter.calculate(ty)
                tagDistance = 20.0.inches / tan((15.0 + trimmedFiducials[0].second.second).degrees)
            } else {
                ty = 1000.0
                filteredTy = tyFilter.calculate(oldty)
            }

        }

    }

    fun distanceToTy(distance: Distance): Angle {
        return atan(20.0/distance.asInches) - 15.0.degrees
    }

    fun gyroReset() {
        io.gyroReset()
    }

    fun onEnable() = io.enable()

    fun onDisable() = io.disable()
}