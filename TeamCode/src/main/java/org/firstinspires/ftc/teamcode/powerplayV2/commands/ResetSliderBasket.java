package org.firstinspires.ftc.teamcode.powerplayV2.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.powerplayV2.BasketSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.ElevatorCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.ElevatorSubsystem;

public class ResetSliderBasket extends CommandBase {
    private ElevatorSubsystem elevator;
    private BasketSubsystem basket;

    private ParallelCommandGroup actions;

    public ResetSliderBasket(ElevatorSubsystem elevator, BasketSubsystem basket) {
        this.elevator = elevator;
        this.basket = basket;

        actions = new ParallelCommandGroup(
                new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW),
                new InstantCommand(basket::setTravel, basket)
        );

        addRequirements(elevator, basket);
    }

    @Override
    public void initialize() {
        actions.schedule();
    }

    @Override
    public boolean isFinished() {
        return actions.isFinished();
    }
}