package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.Utils
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.vision.limelight.VisionIO
import org.team2471.frc.lib.vision.limelight.VisionIOLimelight

object Vision : SubsystemBase() {
    val io: VisionIO = VisionIOLimelight("limelight-turret") { Turret.turretEncoderFieldCentricAngle }

    const val TURRET_TO_ROBOT_IN = 7.39

    val inputs = VisionIO.VisionIOInputs()
    var rawLimelightPose = Pose2d()


    var aimError2d: Angle? = null

    override fun periodic() {
        io.updateInputs(inputs)

        if (inputs.aprilTagPoseEstimate != Pose2d()) {
            rawLimelightPose = inputs.aprilTagPoseEstimate

            if (inputs.trimmedFiducials.size == 2) {
                val avgTx = (inputs.trimmedFiducials[0].first + inputs.trimmedFiducials[1].first) / 2.0

                aimError2d = avgTx.degrees
            } else if (inputs.trimmedFiducials.size == 1) {
                aimError2d = inputs.trimmedFiducials[0].first.degrees
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
        }

    }

    fun gyroReset() {
        io.gyroReset()
    }

    fun onEnable() = io.enable()

    fun onDisable() = io.disable()
}