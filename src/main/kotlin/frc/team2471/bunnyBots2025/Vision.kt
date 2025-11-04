package frc.team2471.bunnyBots2025

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.vision.limelight.LimelightMode
import org.team2471.frc.lib.vision.limelight.VisionIO
import org.team2471.frc.lib.vision.limelight.VisionIOLimelight
import org.littletonrobotics.junction.Logger
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.sign

object Vision : SubsystemBase() {
    val io: Array<VisionIO> = arrayOf(
        VisionIOLimelight("limelight") { Turret.turretEncoderFieldCentricAngle },
    )

    val inputs = Array(io.size) { VisionIO.VisionIOInputs() }
    var rawAprilTagPose = Pose2d()

    override fun periodic() {
        for (i in io.indices) {
            io[i].updateInputs(inputs[i])

            if (inputs[i].aprilTagPoseEstimate != Pose2d()) {
                rawAprilTagPose = inputs[i].aprilTagPoseEstimate
                Drive.addVisionMeasurement(rawAprilTagPose, inputs[i].aprilTagTimestamp, VecBuilder.fill(0.01, 0.01, 1000000000.0))
            }

        }
    }

    fun gyroReset() {
        io.forEach { it.gyroReset() }
    }

    fun onEnable() = io.forEach { it.enable() }

    fun onDisable() = io.forEach { it.disable() }



}