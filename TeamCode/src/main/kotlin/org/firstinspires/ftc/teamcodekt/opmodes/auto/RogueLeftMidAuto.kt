package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.AutoData

@Autonomous
class RogueLeftMidAuto : RogueBaseAuto() {
    override val startPose = GlobalUnits.pos(-84, -169, 90)

    override fun mainTraj(startPose: Pose2d) =
            Anvil.formTrajectory(bot.drive, startPose)
                .forward(15)
                .turn(180)
                .inReverse {
                    splineTo(-84.5, -36.5, 90)
                    turn(44)
                }

                .splineTo(-73.2, -47.8, -44)
                .waitPlease()

                .doTimes(5) {
                    inReverse {
                        splineTo(-156, -30, 180)
                        waitPlease()
                    }

                    splineTo(-73.2, -47.8, -44)
                    waitPlease()
                }

    private fun Anvil.waitPlease() = this
            .waitTime(350)

    private fun Anvil.initialDepositPrep() = this
        .addTemporalMarker {
            bot.lift.goToHigh()
            bot.claw.close()
            bot.arm.setToForwardsPos()
            bot.wrist.setToForwardsPos()
        }

    private fun Anvil.awaitInitialGoToDeposit() = this
        .splineToSplineHeading(-82.5, -12.75, 50, 65)

    private fun Anvil.awaitGoToDeposit(it: Int) = when (it) {
        0 -> splineToSplineHeading(-85.100, -10.250, 44.250, 25)
        1 -> splineToSplineHeading(-81.800, -14.905, 53.275, 25)
        2 -> splineToSplineHeading(-78.750, -15.050, 52.900, 25)
        3 -> splineToSplineHeading(-81.124, -14.175, 54.625, 25)
        4 -> splineToSplineHeading(-81.550, -14.550, 51.600, 25)
        else -> this
    }

    private fun Anvil.awaitGoToIntake(it: Int) = when (it) {
        0 -> splineTo(-163.9000, -21.250, 180)
        1 -> splineTo(-164, -22.575, 180)
        2 -> splineTo(-162.9100, -22.400, 180)
        3 -> splineTo(-161.9975, -23.575, 180)
        4 -> splineTo(-160.9900, -23.750, 180)
        else -> noop
    }.doInReverse()

    private fun Anvil.deposit() = this
        .addTemporalMarker(-165) {
            bot.lift.clippedHeight -= AutoData.DEPOSIT_DROP_AMOUNT
        }

        .addTemporalMarker(-100) {
            bot.claw.openForDeposit()
        }

    private fun Anvil.regularIntakePrep(iterations: Int) = this
        .addTemporalMarker(185) {
            bot.lift.clippedHeight = liftOffsets[iterations]
            bot.wrist.setToBackwardsPos()
            bot.arm.setToBackwardsPosButLikeSliiiightlyHigher()
        }

        .addTemporalMarker(325) {
            bot.claw.openForIntakeWide()
        }

    private fun Anvil.fastIntakePrep(iterations: Int) = this
        .addTemporalMarker(185) {
            bot.lift.clippedHeight  = liftOffsets[iterations]

            bot.arm.setToBackwardsPos()
            bot.wrist.setToBackwardsPos()

            bot.claw.openForIntakeNarrow()
            bot.intake.enable()
        }

    private fun Anvil.awaitRegularIntake() = this
        .addTemporalMarker {
            bot.intake.disable()
            bot.claw.close()
        }

        .addTemporalMarker(275) {
            bot.lift.goToHigh()
        }

        .addTemporalMarker(425) {
            bot.arm.setToForwardsPos()
            bot.wrist.setToForwardsPos()
        }

        .waitTime(300)

    private fun Anvil.awaitFastIntake() = this
        .addTemporalMarker(-75) {
            bot.intake.disable()
            bot.claw.close()
        }

        .addTemporalMarker(15) {
            bot.arm.setToForwardsPos()
            bot.lift.goToHigh()
        }

        .addTemporalMarker(100) {
            bot.wrist.setToForwardsPos()
        }

        .waitTime(120)

    private fun Anvil.resetBot() = this
        .addTemporalMarker {
            bot.arm.setToRestingPos()
            bot.wrist.setToRestingPos()
            bot.lift.goToZero()
            bot.claw.close()
        }

    private fun parkTraj(startPose: Pose2d) =
        Anvil.formTrajectory(bot.drive, startPose) {
            resetBot()

            when (signalID) {
                1 -> inReverse {
                    splineTo(-160, -33, 180)
                }
                2 -> inReverse {
                    splineTo(-95.5, -30, 180)
                }
                3 -> {
                    splineToLinearHeading(-30, -34, 90, -38.375)
                }
            }

            this
        }
}