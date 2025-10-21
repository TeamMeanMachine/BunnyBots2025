package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveRequest
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Timer
import gg.questnav.questnav.QuestNav
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.ctre.PhoenixUtil
import org.team2471.frc.lib.localization.PoseLocalizer
import org.team2471.frc.lib.math.cube
import org.team2471.frc.lib.math.square
import org.team2471.frc.lib.swerve.SwerveDriveSubsystem
import org.team2471.frc.lib.units.asMetersPerSecondPerSecond
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.util.isBlueAlliance
import org.team2471.frc.lib.util.isReal
import org.team2471.frc.lib.util.isRedAlliance
import org.team2471.frc.lib.vision.Fiducial
import org.team2471.frc.lib.vision.QuixVisionCamera
import org.team2471.frc.lib.vision.QuixVisionSim


object Drive: SwerveDriveSubsystem(TunerConstants.drivetrainConstants, *TunerConstants.moduleConfigs) {

    // To reset position use this, also add other pose sources that need reset here.
    override var pose: Pose2d
        get() = savedState.Pose
        set(value) {
            tempQuestPose = value.transformBy(robotToQuestTransformMeters)
            resetQuestTranslation = true
            resetPose(value)
            localizer.resetPose(value) // Possibly not needed, but good for a quick response.
        }

    override var heading: Rotation2d
        get() = pose.rotation
        set(value) {
//            println("resting heading to ${value.degrees}")
            resetRotation(value)
            localizer.resetRotation(value) // Not needed and redundant but may prevent some heading bugs
            if (resetQuestTranslation) {
                quest.setPose(Pose2d(tempQuestPose.translation, value + robotToQuestTransformMeters.rotation))
                resetQuestTranslation = false
            } else {
                quest.setPose(Pose2d(questPose.transformBy(robotToQuestTransformMeters).translation, value + robotToQuestTransformMeters.rotation))
            }
        }

    private var tempQuestPose = Pose2d()
    private var resetQuestTranslation = false

    // Vision
    val cameras: List<QuixVisionCamera> = listOf(
//        LimelightCamera(
//            "limelight-test",
//            Constants.limelightPose,
//            Optional.of(Matrix(N3(), N3(), doubleArrayOf(737.3071162812872,0.0,658.6108346810324,0.0,738.2142014335819,411.36513253891655,0.0,0.0,1.0))),
//            Optional.of(Matrix(N8(), N1(), doubleArrayOf(0.11977275268536702,-0.13935436337469195,-0.000907565402687582,0.0005097193890789445,-0.052047409417428844,0.0,0.0,0.0)))
//            ),
//        PhotonVisionCamera("FrontLeft", Constants.frontLeftCamPose, arrayOf(PipelineConfig())),
//        PhotonVisionCamera("FrontRight", Constants.frontRightCamPose, arrayOf(PipelineConfig())),
//        PhotonVisionCamera("BackLeft", Constants.backLeftCamPose, arrayOf(PipelineConfig())),
//        PhotonVisionCamera("BackRight", Constants.backRightCamPose, arrayOf(PipelineConfig())),
    )

    val quest = QuestNav()

    var simulateQuest = true
    val questConnected: Boolean
        get() = if (isReal) quest.isConnected else simulateQuest
    val robotToQuestTransformMeters = Transform2d(-12.0.inches, 12.0.inches, 180.0.degrees.asRotation2d)

    var questPose: Pose2d = Pose2d()
        private set

    // Trust down to 2 cm in XY and 2 degrees in rotational. Units in meters and radians.
    val QUEST_STD_DEVS: Matrix<N3?, N1?> = VecBuilder.fill(0.025, 0.025, 0.052)

    // Class that handles the merging of multiple vision sources and the odometry.
    val localizer = PoseLocalizer(Fiducial.constructFiducialList(FieldManager.allAprilTags), cameras)

    private val translationRateTimer = Timer()
    private var prevTranslation = Translation2d()

    // Drive Feedback controllers
    override val autoPilot = createAPObject(Double.POSITIVE_INFINITY, 100.0, 2.0, 0.5.inches, 1.0.degrees)
    val fastAutoPilot = createAPObject(Double.POSITIVE_INFINITY, 100.0, 5.0, 0.5.inches, 1.0.degrees)
    val slowAutoPilot = createAPObject(Double.POSITIVE_INFINITY, 100.0, 0.5, 0.25.inches, 1.0.degrees)

    override val pathXController = PIDController(7.0, 0.0, 0.0)
    override val pathYController = PIDController(7.0, 0.0, 0.0)
    override val pathThetaController = PIDController(7.0, 0.0, 0.0)

    override val autoDriveToPointController = PIDController(3.0, 0.0, 0.1)
    override val teleopDriveToPointController = PIDController(3.0, 0.0, 0.1)

    override val driveAtAnglePIDController = PhoenixPIDController(7.7, 0.0, 0.072)

    override val isDisabledSupplier: () -> Boolean = { Robot.isDisabled }

    init {
        println("inside Drive init")

        // MUST start inside the field on bootup for accurate heading measurements due to a vision localizer bug.
        pose = Pose2d(3.0, 3.0, heading)

//        zeroGyro()

        println("max acceleration ${TunerConstants.kMaxAcceleration.asMetersPerSecondPerSecond}")

        localizer.trackAllTags()

        AutoBuilder.configure(
            ::pose,
            ::pose.setter,
            ::robotRelativeSpeeds,
            { wantedSpeeds: ChassisSpeeds, feedforwards: DriveFeedforwards ->
                setControl(SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(wantedSpeeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons)
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons)
                )
            },
            PPHolonomicDriveController(
                PIDConstants(7.0, 0.0),
                PIDConstants(7.7, 0.072)
            ),
            RobotConfig.fromGUISettings(),
            ::isRedAlliance,
            this
        )

        finalInitialization()
    }

    override fun periodic() {
        LoopLogger.record("Inside Drive periodic")

        // Apply quest measurements
        if (questConnected) {
            if (isReal) {
                quest.allUnreadPoseFrames.forEach {
                    val pose = it.questPose.transformBy(robotToQuestTransformMeters.inverse())
                    val ctreTimestamp = Utils.fpgaToCurrentTime(it.dataTimestamp)

                    Logger.recordOutput("Drive/Quest/DataTimestamp", it.dataTimestamp)
                    Logger.recordOutput("Drive/Quest/CtreTimestamp", ctreTimestamp)
                    addVisionMeasurement(pose, ctreTimestamp, QUEST_STD_DEVS)
                    questPose = pose
                }
            } else {
                // Simulate quest data
                addVisionMeasurement(pose, stateTimestamp, QUEST_STD_DEVS)
                questPose = pose
            }
        }

        LoopLogger.record("b4 Drive piodc")
        super.periodic() // Must call this
        LoopLogger.record("super Drive piodc")

        // Update Vision
        cameras.forEach {
            it.updateInputs()
        }
        LoopLogger.record("Drive camera updateInputs")
        // Update poses with processed particle filter estimates.
        localizer.updateWithLatestPoseEstimate()
        LoopLogger.record("Drive updateWithLatestPose")
        // Create an odom measurement with a timestamp converted from phoenix time to fpga time.
        val poseMeasurement = PoseLocalizer.OdometryMeasurement(pose, PhoenixUtil.currentToFpgaTime(stateTimestamp))
        // Publish the latest camera data to NT and also update pose from swerve odometry measurements.
        localizer.update(poseMeasurement, cameras.map { it.latestMeasurement }, speeds)
        LoopLogger.record("Drive localizer")

        quest.commandPeriodic()
        LoopLogger.record("Drive quest periodic")

        Logger.recordOutput("Drive/Quest/isConnected", questConnected)

        LoopLogger.record("Drive questConnected")

        // Log all the poses for debugging
        Logger.recordOutput("Swerve/Odometry", localizer.odometryPose)
        Logger.recordOutput("Swerve/InterpolatedOdometry", localizer.interpolatedOdometryPose)
        Logger.recordOutput("Swerve/InterpolatedPose", localizer.interpolatedPose)
        Logger.recordOutput("Swerve/Quest", questPose)
        Logger.recordOutput("Swerve/Localizer Raw", localizer.rawPose)
        Logger.recordOutput("Swerve/Localizer", localizer.pose)
        Logger.recordOutput("Swerve/SingleTagPose", localizer.singleTagPose)

        LoopLogger.record("Drive pirdc")
    }

    /**
     * Returns [ChassisSpeeds] with a percentage power from the driver controller.
     */
    override fun getJoystickPercentageSpeeds(): ChassisSpeeds {
        val rawJoystick = OI.rawDriveTranslation
        // Square drive input and apply demoSpeed
        val power = rawJoystick.norm.square() * demoSpeed
        // Apply modified power to joystick vector and flip depending on alliance
        val joystickTranslation = rawJoystick * power * if (isBlueAlliance) -1.0 else 1.0

        val rawJoystickRotation = OI.driveRotation
        // Cube rotation input and apply demoSpeed
        val omega = rawJoystickRotation.cube() * demoSpeed

        return ChassisSpeeds(joystickTranslation.x, joystickTranslation.y, omega)
    }

    fun resetOdometryToAbsolute() {
        println("resetting odometry to localizer pose")
        val localizerPose = localizer.pose
        pose = Pose2d(localizerPose.translation, pose.rotation)
    }

    @OptIn(DelicateCoroutinesApi::class)
    override fun simulationPeriodic() {
        LoopLogger.record("b4 Drive Sim piodic")
        GlobalScope.launch {
            updateSimState(0.02, 12.0)
            QuixVisionSim.updatePose(pose)
        }
        LoopLogger.record("Drive Sim piodic")
    }
}