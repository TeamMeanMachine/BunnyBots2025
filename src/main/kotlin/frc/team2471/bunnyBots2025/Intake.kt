package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
import org.team2471.frc.lib.control.commands.onlyRunWhileTrue
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.motionMagic
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.ctre.v
import org.team2471.frc.lib.units.degreesPerSecond
import org.team2471.frc.lib.units.rotationsPerSecond

object Intake: SubsystemBase("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")
    private val frontIntakePowerEntry = table.getEntry("Front Intake Power")
    private val topCenteringPowerEntry = table.getEntry("Top Centering Power")
    private val leftCenteringPowerEntry = table.getEntry("Left Centering Power")
    private val rightCenteringPowerEntry = table.getEntry("Right Centering Power")
    private val cycloneFeedPowerEntry = table.getEntry("Cyclone Feed Power")
    private val shooterFeedPowerEntry = table.getEntry("Shooter Feed Power")
    private val deployPoseEntry = table.getEntry("Deploy Pose")
    private val stowPoseEntry = table.getEntry("Stow Pose")



    val deployMotor = TalonFX(Falcons.INTAKE_DEPLOY)
    val frontMotor = TalonFX(Falcons.INTAKE_FRONT)
    val centeringMotorRight = TalonFX(Falcons.CENTERING_RIGHT)
    val centeringMotorLeft = TalonFX(Falcons.CENTERING_LEFT)
    val centeringMotorTop = TalonFX(Falcons.INDEXER)
    val cycloneMotor = TalonFX(Falcons.CYCLONE, CANivores.TURRET_CAN)
    val shooterFeederMotor = TalonFX(Falcons.SHOOTER_FEEDER, CANivores.TURRET_CAN)

    val deployMotorPosition get() = deployMotor.position.valueAsDouble

    val canRange = CANrange(CANSensors.CENTERING_CAN_RANGE)
    val hasPieceInIndexer get() = canRange.distance.valueAsDouble < CAN_RANGE_DISTANCE_THRESHOLD

    val intakeStopSensor = DigitalInput(DigitalSensors.INTAKE_STOP_SENSOR)

    val hardStopDebouncer = Debouncer(0.02)

    val centeringTimer = Timer()
    var alternateCenteringMotors = 0

    @get:AutoLogOutput(key = "Intake/Hit Hard Stop")
    val hitHardStop get() = !intakeStopSensor.get()

    val deploy_pose get() = deployPoseEntry.getDouble(18.0)
    val stow_pose get() = stowPoseEntry.getDouble(0.0)

    const val CAN_RANGE_DISTANCE_THRESHOLD = 0.07 // meters

    val INTAKE_POWER get() = frontIntakePowerEntry.getDouble(0.7)
    val TOP_CENTERING_POWER get() = topCenteringPowerEntry.getDouble(0.7)
    val LEFT_CENTERING_POWER get() = leftCenteringPowerEntry.getDouble(0.7)
    val RIGHT_CENTERING_POWER get() = rightCenteringPowerEntry.getDouble(0.7)
    val CYCLONE_POWER get() = cycloneFeedPowerEntry.getDouble(1.0)
    val SHOOTER_FEEDER_POWER get() = shooterFeedPowerEntry.getDouble(1.0)

    val CYCLONE_SPEED: AngularVelocity
        get() = 95.0.rotationsPerSecond * CYCLONE_POWER

    const val BULLDOZING_POWER = -0.2
    const val HOMING_POWER = 0.1

    val alternateFrames: Int = 10

    var frontIntakePowerSetpoint: Double = 0.0
        set(value) {
            field = /*if (deployMotorPosition > deploy_pose - 5.0)*/ value.coerceIn(-1.0, 1.0) /*else 0.0*/
            frontMotor.setControl(DutyCycleOut(field))
        }
    var topCenteringPowerSetpoint: Double = 0.0
        set(value) {
            field = /*if (deployMotorPosition > deploy_pose - 5.0)*/ value.coerceIn(-1.0, 1.0)/* else 0.0*/
            centeringMotorTop.setControl(DutyCycleOut(field))
        }


    var cycloneVelocitySetpoint: AngularVelocity = 0.0.degreesPerSecond
        set(value) {
            field = value
            cycloneMotor.setControl(VelocityVoltage(field))
        }

    @get:AutoLogOutput(key = "Intake/Cyclone Velocity")
    val cycloneVelocity: AngularVelocity
        get() = cycloneMotor.velocity.valueAsDouble.rotationsPerSecond
    @get:AutoLogOutput(key = "Intake/Cyclone Voltage")
    val cycloneVoltage: Double
        get() = cycloneMotor.motorVoltage.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Current State")
    var intakeState = IntakeState.HOLDING

    init {
        frontIntakePowerEntry.setDouble(INTAKE_POWER)
        topCenteringPowerEntry.setDouble(TOP_CENTERING_POWER)
        leftCenteringPowerEntry.setDouble(LEFT_CENTERING_POWER)
        rightCenteringPowerEntry.setDouble(RIGHT_CENTERING_POWER)
        cycloneFeedPowerEntry.setDouble(CYCLONE_POWER)
        shooterFeedPowerEntry.setDouble(SHOOTER_FEEDER_POWER)

        deployPoseEntry.setDouble(deploy_pose)
        stowPoseEntry.setDouble(stow_pose)

        frontIntakePowerEntry.setPersistent()
        topCenteringPowerEntry.setPersistent()
        leftCenteringPowerEntry.setPersistent()
        rightCenteringPowerEntry.setPersistent()
        cycloneFeedPowerEntry.setPersistent()
        shooterFeedPowerEntry.setPersistent()
        deployPoseEntry.setPersistent()
        stowPoseEntry.setPersistent()

        centeringTimer.restart()



        deployMotor.applyConfiguration {
            currentLimits(20.0,30.0,1.0)
            coastMode()
            p(3.0)
            s(0.03, StaticFeedforwardSignValue.UseVelocitySign)
            motionMagic(300.0, 500.0)
        }
        frontMotor.applyConfiguration {
            currentLimits(20.0,30.0,1.0)
            coastMode()
        }
        centeringMotorLeft.applyConfiguration {
            currentLimits(20.0,30.0,1.0)
            inverted(true)
            coastMode()
        }
        centeringMotorRight.applyConfiguration {
            currentLimits(20.0,30.0,1.0)
            coastMode()
        }
        centeringMotorTop.applyConfiguration {
            currentLimits(20.0,30.0,1.0)
            inverted(true)
            coastMode()
        }
        cycloneMotor.applyConfiguration {
            currentLimits(39.0, 50.0, 0.5)
            coastMode()
            s(0.12, StaticFeedforwardSignValue.UseVelocitySign)
            v(0.126)
            p(1.0)
        }
        shooterFeederMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            coastMode()
        }
    }

    fun deploy() {
        deployMotor.setControl(MotionMagicDutyCycle(deploy_pose))
//        currentState = State.INTAKING
    }

    fun stow() {
        deployMotor.setControl(MotionMagicDutyCycle(stow_pose))
//        currentState = State.HOLDING
    }

    fun home(): Command = sequenceCommand(
        runCommand(this) {
            deployMotor.setControl(DutyCycleOut(HOMING_POWER))
        }.onlyRunWhileTrue { hardStopDebouncer.calculate(hitHardStop) },
        runCommand(this) {
            deployMotor.setControl(DutyCycleOut(-HOMING_POWER))
        }.onlyRunWhileFalse { hardStopDebouncer.calculate(hitHardStop) }.finallyRun {
            deployMotor.setControl(DutyCycleOut(0.0))
            deployMotor.setPosition(0.0)
            stow()
        }
    )

    override fun periodic() {
        when (intakeState) {
            IntakeState.INTAKING -> {
                frontIntakePowerSetpoint = INTAKE_POWER
                if (hasPieceInIndexer) {
                    centeringMotorLeft.setControl(DutyCycleOut(0.0))
                    centeringMotorRight.setControl(DutyCycleOut(0.0))
                } else {
                    alternateCenteringLogic()
//                    centeringMotorLeft.setControl(DutyCycleOut(LEFT_CENTERING_POWER))
//                    centeringMotorRight.setControl(DutyCycleOut(RIGHT_CENTERING_POWER))
                }
                topCenteringPowerSetpoint = TOP_CENTERING_POWER

                cycloneMotor.setControl(DutyCycleOut(0.0))
                shooterFeederMotor.setControl(DutyCycleOut(0.0))
            }

            IntakeState.HOLDING -> {
                frontIntakePowerSetpoint = 0.0
                centeringMotorLeft.setControl(DutyCycleOut(0.0))
                centeringMotorRight.setControl(DutyCycleOut(0.0))
                topCenteringPowerSetpoint = 0.0
                cycloneMotor.setControl(DutyCycleOut(0.0))
                shooterFeederMotor.setControl(DutyCycleOut(0.0))
            }

            IntakeState.REVERSING -> {
                frontIntakePowerSetpoint = -INTAKE_POWER
                centeringMotorLeft.setControl(DutyCycleOut(-LEFT_CENTERING_POWER))
                centeringMotorRight.setControl(DutyCycleOut(-RIGHT_CENTERING_POWER))
                topCenteringPowerSetpoint = -TOP_CENTERING_POWER
                cycloneMotor.setControl(DutyCycleOut(-CYCLONE_POWER))
//                cycloneVelocitySetpoint = -CYCLONE_SPEED
                shooterFeederMotor.setControl(DutyCycleOut(-SHOOTER_FEEDER_POWER))
            }

            IntakeState.PARTIAL_REVERSE -> {
                frontIntakePowerSetpoint = 0.0
                centeringMotorLeft.setControl(DutyCycleOut(-LEFT_CENTERING_POWER))
                centeringMotorRight.setControl(DutyCycleOut(-RIGHT_CENTERING_POWER))
                topCenteringPowerSetpoint = -TOP_CENTERING_POWER
                cycloneMotor.setControl(DutyCycleOut(-CYCLONE_POWER))
//                cycloneVelocitySetpoint = -CYCLONE_SPEED
                shooterFeederMotor.setControl(DutyCycleOut(-SHOOTER_FEEDER_POWER))
            }

            IntakeState.SHOOTING -> {
                frontIntakePowerSetpoint = INTAKE_POWER
//                centeringMotorLeft.setControl(DutyCycleOut(LEFT_CENTERING_POWER))
//                centeringMotorRight.setControl(DutyCycleOut(RIGHT_CENTERING_POWER))
                alternateCenteringLogic()
                topCenteringPowerSetpoint = TOP_CENTERING_POWER
                cycloneMotor.setControl(DutyCycleOut(CYCLONE_POWER))
//                cycloneVelocitySetpoint = CYCLONE_SPEED
                shooterFeederMotor.setControl(DutyCycleOut(SHOOTER_FEEDER_POWER))
            }

            IntakeState.BULLDOZING -> {
                frontIntakePowerSetpoint = -INTAKE_POWER
                centeringMotorLeft.setControl(DutyCycleOut(0.0))
                centeringMotorRight.setControl(DutyCycleOut(0.0))
                topCenteringPowerSetpoint = 0.0
                cycloneMotor.setControl(DutyCycleOut(0.0))
                shooterFeederMotor.setControl(DutyCycleOut(0.0))
            }
        }

        Logger.recordOutput("Intake/Deploy Motor Position", deployMotorPosition)
        Logger.recordOutput("Intake/Cyclone Current", cycloneMotor.supplyCurrent.valueAsDouble)
    }

    fun alternateCenteringLogic() {
//        val t = centeringTimer.get()

        if (alternateCenteringMotors % alternateFrames < (alternateFrames / 2)) {
            centeringMotorLeft.setControl(DutyCycleOut(LEFT_CENTERING_POWER))
            centeringMotorRight.setControl(DutyCycleOut(0.0))
        } else {
            centeringMotorLeft.setControl(DutyCycleOut(0.0))
            centeringMotorRight.setControl(DutyCycleOut(RIGHT_CENTERING_POWER))
        }
        alternateCenteringMotors++
    }

    enum class IntakeState {
        INTAKING,
        HOLDING,
        REVERSING,
        SHOOTING,
        PARTIAL_REVERSE,
        BULLDOZING
    }
}