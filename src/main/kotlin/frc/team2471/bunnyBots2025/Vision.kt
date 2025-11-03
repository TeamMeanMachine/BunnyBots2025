package frc.team2471.bunnyBots2025

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.ctre.PhoenixUtil
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.vision.limelight.VisionIO
import org.team2471.frc.lib.vision.limelight.VisionIOLimelight

object Vision: SubsystemBase("Vision") {
    val io = VisionIOLimelight("limelight-front") { Drive.heading.measure }
    val inputs = VisionIO.VisionIOInputs()

    val cameraTranslation = Translation3d(0.0, 0.0, 1.0)
    val goalTagHeight = 43.875.inches
    /*
    only uses delta turret angle, delta odometry, tx, and tag size to predict and calculate goal pose
     */
    val predictedRobotRelativeGoalPose = Translation2d(0.0, 0.0)

    init {

    }

    override fun periodic() {
        io.updateInputs(inputs)

        val measurementTimestamp = PhoenixUtil.currentToFpgaTime(inputs.aprilTagTimestamp)
        if (inputs.isConnected && inputs.hasTargets && measurementTimestamp - Timer.getTimestamp() < 10.0) {

        }

    }
}