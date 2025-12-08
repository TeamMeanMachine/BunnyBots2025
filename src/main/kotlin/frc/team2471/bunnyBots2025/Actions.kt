package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.controls.DutyCycleOut
import edu.wpi.first.wpilibj2.command.Command
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnce
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.units.degrees


fun spinnySpit(): Command {
    var initTurretAngle = 0.0.degrees
    return sequenceCommand(
        runOnce {
            initTurretAngle = Turret.turretMotorAngle
        },
        runCommand(Turret) {
            Turret.turretMotor.setControl(DutyCycleOut(0.2))
            Intake.intakeState = Intake.IntakeState.REVERSING
        }.finallyRun {
            Turret.turretSetpoint = initTurretAngle
            Intake.intakeState = Intake.IntakeState.HOLDING
        }
    )
}

fun spinnyPartialSpit(): Command {
    var initTurretAngle = 0.0.degrees
    return sequenceCommand(
        runOnce {
            initTurretAngle = Turret.turretMotorAngle
        },
        runCommand(Turret) {
            Turret.turretMotor.setControl(DutyCycleOut(0.2))
            Intake.intakeState = Intake.IntakeState.PARTIAL_REVERSE
        }.finallyRun {
            Turret.turretSetpoint = initTurretAngle
            Intake.intakeState = Intake.IntakeState.HOLDING
        }
    )
}

