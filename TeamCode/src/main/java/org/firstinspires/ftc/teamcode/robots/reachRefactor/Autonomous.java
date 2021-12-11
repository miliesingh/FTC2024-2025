package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

public class Autonomous {
    public VisionProvider visionProvider;
    private Position mostFrequentPosition;
    private Robot robot;

    public Autonomous(Robot robot) {
        this.robot = robot;
    }

    private StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> {})
                .stateEndAction(() -> {})
                .stage(stage);
    }

    public void createVisionProvider(int visionProviderIndex) {
        try {
            visionProvider = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance();
        } catch(IllegalAccessException | InstantiationException e) {
            throw new RuntimeException("Error while instantiating vision provider");
        }
    }

    // Autonomous articulations
    private Stage autonomousRedStage = new Stage();
    public StateMachine autonomousRed = getStateMachine(autonomousRedStage)
            .addState(() -> robot.driveTrain.driveAbsoluteDistance(1,0,true,1,.2))
            .addTimedState(.5f, () -> {}, () -> {})
            .addState(() -> robot.driveTrain.rotateIMU(90,2))
            .addTimedState(2f, () -> {}, () -> {})
            .addTimedState(2f, () -> {
                robot.driveTrain.handleDuckSpinnerToggle(robot.getAlliance().getMod());
            }, () -> {})
            .build();

    private Stage autonomousBlueStage = new Stage();
    public StateMachine autonomousBlue = getStateMachine(autonomousBlueStage)
            // TODO: insert autonomous blue states here
            .build();

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------

    public Position getMostFrequentPosition() { return mostFrequentPosition; }

    public void setMostFrequentPosition(Position mostFrequentPosition) { this.mostFrequentPosition = mostFrequentPosition; }

}
