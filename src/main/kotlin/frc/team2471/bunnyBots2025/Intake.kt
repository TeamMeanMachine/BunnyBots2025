package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
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
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.motionMagic
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s

object Intake: SubsystemBase("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")
    val deployMotor = TalonFX(Falcons.INTAKE_DEPLOY)
    val frontMotor = TalonFX(Falcons.INTAKE_FRONT)

    val centeringMotorRight = TalonFX(Falcons.CENTERING_RIGHT)
    val centeringMotorLeft = TalonFX(Falcons.CENTERING_LEFT)
    val indexerMotor = TalonFX(Falcons.INDEXER)

    val deployMotorPosition get() = deployMotor.position.valueAsDouble

    val canRange = CANrange(CANSensors.CENTERING_CAN_RANGE)
    val hasPieceInIndexer get() = canRange.distance.valueAsDouble < CAN_RANGE_DISTANCE_THRESHOLD

    val intakeStopSensor = DigitalInput(DigitalSensors.INTAKE_STOP_SENSOR)

    val hardStopDebouncer = Debouncer(0.02)

    @get:AutoLogOutput(key = "Intake/Hit Hard Stop")
    val hitHardStop get() = !intakeStopSensor.get()

    const val DEPLOY_POSE = 15.5
    const val STOW_POSE = -0.5

    const val CAN_RANGE_DISTANCE_THRESHOLD = 0.07 // meters

    const val INTAKE_POWER = 0.7
    const val INDEXING_POWER = 0.7
    const val CENTERING_POWER = 0.7
    const val BULLDOZING_POWER = -0.2
    const val HOMING_POWER = 0.1

    @get:AutoLogOutput(key = "Intake/Current State")
    var currentState = State.HOLDING

    init {
        deployMotor.applyConfiguration {
            currentLimits(20.0,30.0,1.0)
            coastMode()
            p(3.0)
            s(0.03, StaticFeedforwardSignValue.UseVelocitySign)
            motionMagic(300.0, 500.0)
        }
        frontMotor.applyConfiguration {
            currentLimits(10.0,20.0,1.0)
            coastMode()
        }
        centeringMotorLeft.applyConfiguration {
            currentLimits(20.0,30.0,1.0)
            coastMode()
        }
        centeringMotorRight.applyConfiguration {
            currentLimits(20.0,30.0,1.0)
            coastMode()
        }
        indexerMotor.applyConfiguration {
            currentLimits(10.0,20.0,1.0)
            brakeMode()
        }
    }

    fun deploy() {
        deployMotor.setControl(MotionMagicDutyCycle(DEPLOY_POSE))
//        currentState = State.INTAKING
    }

    fun stow() {
        deployMotor.setControl(MotionMagicDutyCycle(STOW_POSE))
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
        when (currentState) {
            State.INTAKING -> {
                frontMotor.setControl(DutyCycleOut(INTAKE_POWER))

                if (hasPieceInIndexer) {
                    centeringMotorLeft.setControl(DutyCycleOut(0.0))
                } else {
                    centeringMotorLeft.setControl(DutyCycleOut(CENTERING_POWER))
                }

                indexerMotor.setControl(DutyCycleOut(0.0))
            }

            State.HOLDING -> {
                frontMotor.setControl(DutyCycleOut(0.0))
                centeringMotorLeft.setControl(DutyCycleOut(0.0))
                indexerMotor.setControl(DutyCycleOut(0.0))
            }

            State.REVERSING -> {
                frontMotor.setControl(DutyCycleOut(-INTAKE_POWER))
                centeringMotorLeft.setControl(DutyCycleOut(-CENTERING_POWER))
                indexerMotor.setControl(DutyCycleOut(-INDEXING_POWER))
            }

            State.SHOOTING -> {
                frontMotor.setControl(DutyCycleOut(INTAKE_POWER))
                centeringMotorLeft.setControl(DutyCycleOut(CENTERING_POWER))
                indexerMotor.setControl(DutyCycleOut(INDEXING_POWER))
            }

            State.BULLDOZING -> {
                frontMotor.setControl(DutyCycleOut(BULLDOZING_POWER))
                centeringMotorLeft.setControl(DutyCycleOut(0.0))
                indexerMotor.setControl(DutyCycleOut(0.0))
            }
        }

        Logger.recordOutput("Intake/Deploy Motor Position", deployMotorPosition)
    }

    enum class State {
        INTAKING,
        HOLDING,
        REVERSING,
        SHOOTING,
        BULLDOZING
    }
}