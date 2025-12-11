package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.Utils
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.units.asInches
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.atan
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.tan
import org.team2471.frc.lib.vision.limelight.VisionIO
import org.team2471.frc.lib.vision.limelight.VisionIOLimelight

object Vision : SubsystemBase() {
    val io: VisionIO = VisionIOLimelight("limelight-turret") { Turret.turretEncoderFieldCentricAngle }.apply { imuMode = 3; imuAssistAlpha = 1.0 }
    val inputs = VisionIO.VisionIOInputs()

    val poseEstimator = SwerveDrivePoseEstimator(Drive.kinematics, Drive.heading, Drive.modulePositions, Drive.pose, Drive.QUEST_STD_DEVS,
        VecBuilder.fill(0.01, 0.01, Double.MAX_VALUE))

    const val TURRET_TO_ROBOT_IN = 7.39

    var rawLimelightPose = Pose2d()


    @get:AutoLogOutput(key = "Vision/AimError2d")
    var aimError2d: Angle? = null

    @get:AutoLogOutput(key = "Vision/TagDistance")
    var tagDistance: Distance = Double.MAX_VALUE.inches

    @get:AutoLogOutput(key = "Vision/Ty")
    var ty: Double = 1000.0

    var oldty: Double = ty

    val tyFilter = LinearFilter.singlePoleIIR(0.2, 0.02)

    @get:AutoLogOutput(key = "Vision/SeeTags")
    var seeTags = false

    var numTagsHistory: MutableList<Int> = mutableListOf()

    @get:AutoLogOutput(key = "Vision/HasJitteryTag")
    val hasJitteryTag: Boolean
        get() {
            var numChanges = 0
            for (i in 1..numTagsHistory.size - 1) {
                if (numTagsHistory[i - 1] != numTagsHistory[i]) {
                    numChanges++
                }
            }
            return numChanges > 2
        }

    private val seeTagsDebouncer = Debouncer(0.5)
    private var seeTagsRaw = false
        set(value) {
            seeTags = seeTagsDebouncer.calculate(value)
            field = value
        }

    @get:AutoLogOutput(key = "Vision/FilteredTy")
    var filteredTy: Double = 1000.0

    override fun periodic() {
        LoopLogger.record("Start of Vision periodic")
        io.updateInputs(inputs)

        LoopLogger.record("Updated Vision Inputs")

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

            LoopLogger.record("Calculated aimError2d")

            Logger.recordOutput("Vision/RobotPos", Pose2d(
                rawLimelightPose.translation + Translation2d(
                    TURRET_TO_ROBOT_IN.inches,
                    0.0.inches
                ).rotateBy((Drive.headingHistory.get(inputs.aprilTagTimestamp) ?: 0.0).degrees.asRotation2d),
                Drive.heading
            ))

            LoopLogger.record("Logged Calculated robotPos")

            poseEstimator.addVisionMeasurement(
                Pose2d(
                    rawLimelightPose.translation + Translation2d(
                        TURRET_TO_ROBOT_IN.inches,
                        0.0.inches
                    ).rotateBy((Drive.headingHistory.get(inputs.aprilTagTimestamp) ?: 0.0).degrees.asRotation2d),
                    Drive.heading
                ),
                inputs.aprilTagTimestamp, VecBuilder.fill(0.0000001, 0.0000001, 1000000000.0)
            )

            LoopLogger.record("Updated PoseEstimator")

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

            LoopLogger.record("Performed Calculations with trimmedFiducials")

            numTagsHistory.add(trimmedFiducials.size)

            if (filteredTy > -1.0) {
                io.imuAssistAlpha = 1.0
            } else {
                io.imuAssistAlpha = 0.0000000000000000000001
            }

            LoopLogger.record("Updated imuassist")

        }
        else {
            numTagsHistory.add(0)
        }

        if (numTagsHistory.size > 25) {
            numTagsHistory.removeAt(0)
        }

        LoopLogger.record("Updated num tag history")

        Logger.recordOutput("Vision/PoseEstimatorPose", poseEstimator.estimatedPosition)

//        if (Robot.isDisabled) {
//            io.disabledGyroReset()
//        }

    }

    fun distanceToTy(distance: Distance): Angle {
        return atan(20.0/distance.asInches) - 15.0.degrees
    }

    fun gyroReset() {
        io.gyroReset()
    }

    fun onEnable() {
        io.enable()
    }

    fun onDisable() {
        io.disable()
    }

}