package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s

object Intake: SubsystemBase("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")
    val deployMotor = TalonFX(Falcons.INTAKE_DEPLOY)
    val frontMotor = TalonFX(Falcons.INTAKE_FRONT)

    val centeringMotorRight = TalonFX(Falcons.CENTERING_RIGHT)
    val centeringMotorLeft = TalonFX(Falcons.CENTERING_LEFT)
    val indexerMotor = TalonFX(Falcons.INDEXER)

    val canRange = CANrange(CANSensors.CENTERING_CAN_RANGE)
    val pieceInCenteringThing get() = canRange.distance.valueAsDouble < CAN_RANGE_DISTANCE_THRESHOLD

    const val DEPLOY_POSE = 1.0
    const val HOME_POSE = 0.0

    const val CAN_RANGE_DISTANCE_THRESHOLD = 0.02 // meters

    const val INTAKE_POWER = 0.7
    const val INDEXING_POWER = 0.7
    const val CENTERING_POWER = 0.7
    const val BULLDOZING_POWER = -0.2

    @get:AutoLogOutput(key = "Intake/Current State")
    var currentState = State.HOLDING

    init {
        deployMotor.applyConfiguration {
            currentLimits(20.0,30.0,1.0)
            coastMode()
            p(0.0)
            s(0.0, StaticFeedforwardSignValue.UseVelocitySign)
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
        deployMotor.setControl(MotionMagicVoltage(DEPLOY_POSE))
        currentState = State.INTAKING
    }

    fun home() {
        deployMotor.setControl(MotionMagicVoltage(HOME_POSE))
        currentState = State.HOLDING
    }

    override fun periodic() {
        when (currentState) {
            State.INTAKING -> {
                frontMotor.setControl(DutyCycleOut(INTAKE_POWER))

                if (pieceInCenteringThing) {
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
    }

    enum class State {
        INTAKING,
        HOLDING,
        REVERSING,
        SHOOTING,
        BULLDOZING
    }
}