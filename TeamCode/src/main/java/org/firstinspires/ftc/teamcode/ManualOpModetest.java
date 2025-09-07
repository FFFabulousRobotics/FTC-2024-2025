package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.auto.Command;
import org.firstinspires.ftc.teamcode.auto.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.auto.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.auto.commands.ForwardCommand;
import org.firstinspires.ftc.teamcode.auto.commands.GotoPosWithHeadingCommand;
import org.firstinspires.ftc.teamcode.auto.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.auto.commands.SetLiftPositionCommand;
import org.firstinspires.ftc.teamcode.auto.commands.SleepCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotChassis;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@TeleOp
public class ManualOpModetest extends LinearOpMode {
    enum ArmState {
        IDLE, WITHDRAWING, TURNING_OUT, TURNED, TURNING_BACK, LOCKED, LOCKING, UNLOCKING
    }

    enum LiftState {
        DISABLED, RUNNING
    }

    RobotTop robotTop;
    RobotChassis robotChassis;
    ArmState armState;
    LiftState liftState;
    Gamepad previousGamepad1;
    Gamepad previousGamepad2;
    RobotAuto robotAuto;



    //Constants
    final int STRETCH_BACK_POSITION = 70;
    final int STRETCH_OUT_POSITION = 1500;
    final double SPIN_DEFAULT_POSITION_L = 0.85;
    final double SPIN_DEFAULT_POSITION_R = 0.15;
    final double SPIN_HOVERING_POSITION_L = 0.25;
    final double SPIN_HOVERING_POSITION_R = 0.78;
    final double SPIN_LEAN_RIGHT_L = 0;
    final double SPIN_LEAN_RIGHT_R = 0.48;
    final double SPIN_LEAN_LEFT_L = 0.48;
    final double SPIN_LEAN_LEFT_R = 1;
    final double SPIN_DOWN_POSITION = 0;
    final double TURN_BACK_POSITION = 0.5;
    final double TURN_LOCK_POSITION = 0.68;
    final double TURN_HOVERING_POSITION = 0.75;
    final double TURN_DOWN_POSITION = 0.85;
    final double GRAB_OPEN_POSITION = 0.4;
    final double GRAB_CLOSE_POSITION = 0.92;
    final double TOP_BACK = 0.03;
    final double TOP_OUT = 0.66;
    final private double[] posHanging = {-35, 28, 180};//挂的地方

    // Variables
    boolean isGrabbing;
    boolean topServoOut;
    boolean backGrabOpen;
    int liftPosition;
    double targetTurnPosition;
    double currentTurnPosition;
    boolean spinLeft;
    boolean spinRight;
    boolean AUTO;
    int spinIndex = 0;

    @Override
    public void runOpMode() {
        previousGamepad1 = new Gamepad();
        previousGamepad1.copy(gamepad1);
        previousGamepad2 = new Gamepad();
        previousGamepad2.copy(gamepad2);
        this.robotTop = new RobotTop(this);
        this.robotChassis = new RobotChassis(this);
        this.armState = ArmState.IDLE;
        this.liftState = LiftState.DISABLED;
        this.isGrabbing = false;
        this.topServoOut = false;
        this.liftPosition = 0;
        this.backGrabOpen = false;
        this.targetTurnPosition = TURN_BACK_POSITION;
        spinLeft = false;
        spinRight = false;
        robotAuto = new RobotAuto(this);
        robotAuto.resetCoordinates();
        waitForStart();
        // robotTop.setTurnPosition(TURN_BACK_POSITION);
        robotTop.setTopServoPosition(TOP_BACK);
        robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION_L);
        robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION_R);
        int armStretchStartPos = robotTop.getArmStretchPosition();
        telemetry.addData("aSTARTpos", armStretchStartPos);
        telemetry.update();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robotChassis.driveRobot(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);
            if (gamepad1.a && !previousGamepad1.a) {
                if (backGrabOpen) {
                    robotTop.setLiftServoPosition(0);
                } else {
                    robotTop.setLiftServoPosition(0.6);
                }
                backGrabOpen = !backGrabOpen;
            }
            switch (armState) {
                case IDLE:
                    handleIdleState();
                    break;
                case TURNING_OUT:
                    handleTurningOutState();
                    break;
                case TURNED:
                    handleTurnedState();
                    break;
                case TURNING_BACK:
                    handleTurningBackState();
                    break;
                case WITHDRAWING:
                    handleWithdrawingState();
                    break;
                case LOCKED:
                    handleLockedState();
                    break;
                case LOCKING:
                    handleLockingState();
                    break;
                case UNLOCKING:
                    handleUnlockingState();
                    break;
            }
            switch (liftState) {
                case DISABLED:
                    handleDisabledState();
                    break;
                case RUNNING:
                    handleRunningState();
                    break;
            }
            if(gamepad2.y && !previousGamepad2.y){
                runAutoHanging();
            }
            if(gamepad2.a && !previousGamepad2.a){
                runAutoBack();
            }
            if(gamepad2.b){
                robotAuto.resetPosition();
            }

            telemetry.addData("arm", armState);
            telemetry.addData("lift", liftState);
            telemetry.addData("armPos", robotTop.getTurnPosition());
            telemetry.addData("armStretchPosition", robotTop.getArmStretchPosition());
            telemetry.addData("leftSpin", robotTop.getArmLeftSpinPosition());
            telemetry.addData("rightSpin", robotTop.getArmRightSpinPosition());
            telemetry.update();
            previousGamepad1.copy(gamepad1);
            previousGamepad2.copy(gamepad2);
            SparkFunOTOS.Pose2D a = robotAuto.getPosition();
            telemetry.addData("x", a.x);
            telemetry.addData("y", a.y);
            telemetry.addData("h", a.h);
            sleep(10);
            robotAuto.update();
        }

    }

    protected void handleIdleState() {
        if (gamepad1.x && !previousGamepad1.x) {
            robotTop.setStretchPower(0.9);
            targetTurnPosition = TURN_HOVERING_POSITION;
//            robotTop.setTurnPosition(TURN_HOVERING_POSITION);
            robotTop.setArmLeftSpinPosition(SPIN_HOVERING_POSITION_L);
            robotTop.setArmRightSpinPosition(SPIN_HOVERING_POSITION_R);
            armState = ArmState.TURNING_OUT;
        }
        if (gamepad1.b && !previousGamepad1.b) {
            if (isGrabbing) {
                robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
            } else {
                robotTop.setArmGrabPosition(GRAB_OPEN_POSITION);
            }
        }
        if (liftState == LiftState.RUNNING) {
            liftState = LiftState.DISABLED;
        }
        if(gamepad1.dpad_right){
            robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.008));
            robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.008));
        }
        if(gamepad1.dpad_left){
            robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.008));
            robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.008));
        }
        if(gamepad1.dpad_down){
            robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.008));
            robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.008));
        }
        if(gamepad1.dpad_up){
            robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.008));
            robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.008));
        }
    }

    protected void handleTurningOutState() {
        if (robotTop.getArmStretchPosition() >= STRETCH_OUT_POSITION) {
            robotTop.setStretchPower(0);
            armState = ArmState.TURNED;
            spinIndex = 0;
        }
        currentTurnPosition = robotTop.getTurnPosition();
        if (currentTurnPosition > targetTurnPosition + 0.014) {
            robotTop.setTurnPosition(currentTurnPosition - 0.014);
        } else if (currentTurnPosition < targetTurnPosition - 0.014) {
            robotTop.setTurnPosition(currentTurnPosition + 0.014);
        } else {
            robotTop.setTurnPosition(targetTurnPosition);
        }
//        if(robotTop.getTurnPosition() >= TURN_DOWN_POSITION){
//            robotTop.setStretchPower(0);
//            armState = ArmState.TURNED;
//        }
//        robotTop.setTurnPosition(robotTop.getTurnPosition() + 0.005);
    }

    protected void handleTurnedState() {
        if (gamepad1.x && !previousGamepad1.x) {
            robotTop.setStretchPower(-0.9);
            targetTurnPosition = TURN_BACK_POSITION;
            // robotTop.setTurnPosition(TURN_BACK_POSITION);
            spinRight = false;
            spinLeft = false;
            armState = ArmState.WITHDRAWING;
        }
        if (gamepad1.b && !previousGamepad1.b) {
            if (isGrabbing) {
                robotTop.setTurnPosition(TURN_DOWN_POSITION);
                sleep(200);
                robotTop.setArmGrabPosition(GRAB_CLOSE_POSITION);
                sleep(300);
                robotTop.setTurnPosition(TURN_HOVERING_POSITION);
                isGrabbing = !isGrabbing;
            } else {
                robotTop.setArmGrabPosition(GRAB_OPEN_POSITION);
                isGrabbing = !isGrabbing;
            }
        }

        if(gamepad1.dpad_right){
            robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.008));
            robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.008));
        }
        if(gamepad1.dpad_left){
            robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.008));
            robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.008));
        }
        if(gamepad1.dpad_down){
            robotTop.setArmLeftSpinPosition(Math.min(1, robotTop.getArmLeftSpinPosition() + 0.008));
            robotTop.setArmRightSpinPosition(Math.max(0, robotTop.getArmRightSpinPosition() - 0.008));
        }
        if(gamepad1.dpad_up){
            robotTop.setArmLeftSpinPosition(Math.max(0, robotTop.getArmLeftSpinPosition() - 0.008));
            robotTop.setArmRightSpinPosition(Math.min(1, robotTop.getArmRightSpinPosition() + 0.008));
        }
        if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
//            spinLeft = !spinLeft;
//            spinRight = false;
            spinIndex = Math.max(spinIndex - 1, -1);
            if(spinIndex == -1){
                robotTop.setArmLeftSpinPosition(SPIN_LEAN_LEFT_L);
                robotTop.setArmRightSpinPosition(SPIN_LEAN_LEFT_R);
            } else if (spinIndex == 0) {
                robotTop.setArmLeftSpinPosition(SPIN_HOVERING_POSITION_L);
                robotTop.setArmRightSpinPosition(SPIN_HOVERING_POSITION_R);
            } else if (spinIndex == 1) {
                robotTop.setArmLeftSpinPosition(SPIN_LEAN_RIGHT_L);
                robotTop.setArmRightSpinPosition(SPIN_LEAN_RIGHT_R);
            }
        }
        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
//            spinRight = !spinRight;
//            spinLeft = false;
            spinIndex = Math.min(spinIndex + 1, 1);
            if(spinIndex == -1){
                robotTop.setArmLeftSpinPosition(SPIN_LEAN_LEFT_L);
                robotTop.setArmRightSpinPosition(SPIN_LEAN_LEFT_R);
            } else if (spinIndex == 0) {
                robotTop.setArmLeftSpinPosition(SPIN_HOVERING_POSITION_L);
                robotTop.setArmRightSpinPosition(SPIN_HOVERING_POSITION_R);
            } else if (spinIndex == 1) {
                robotTop.setArmLeftSpinPosition(SPIN_LEAN_RIGHT_L);
                robotTop.setArmRightSpinPosition(SPIN_LEAN_RIGHT_R);
            }
        }
//        if (spinLeft) {
//            robotTop.setArmLeftSpinPosition(SPIN_LEAN_LEFT_L);
//            robotTop.setArmRightSpinPosition(SPIN_LEAN_LEFT_R);
//        } else if (spinRight) {
//            robotTop.setArmLeftSpinPosition(SPIN_LEAN_RIGHT_L);
//            robotTop.setArmRightSpinPosition(SPIN_LEAN_RIGHT_R);
//        }

    }

    protected void handleTurningBackState() {
        //if necessary
    }

    protected void handleWithdrawingState() {
        if (robotTop.getArmStretchPosition() <= STRETCH_BACK_POSITION) {
            robotTop.setStretchPower(0);
            robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION_L);
            robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION_R);
            armState = ArmState.IDLE;
        }
        currentTurnPosition = robotTop.getTurnPosition();
        if (currentTurnPosition > targetTurnPosition + 0.014) {
            robotTop.setTurnPosition(currentTurnPosition - 0.014);
        } else if (currentTurnPosition < targetTurnPosition - 0.014) {
            robotTop.setTurnPosition(currentTurnPosition + 0.014);
        } else {
            robotTop.setTurnPosition(targetTurnPosition);
        }
//        if(robotTop.getTurnPosition() <= TURN_BACK_POSITION){
//            robotTop.setStretchPower(0);
//            armState = ArmState.IDLE;
//        }
//        robotTop.setTurnPosition(robotTop.getTurnPosition() - 0.005);
    }

    protected void handleLockedState() {
        if (gamepad1.x && !previousGamepad1.x) {
            liftState = LiftState.DISABLED;
            armState = ArmState.UNLOCKING;
            // robotTop.setTurnPosition(TURN_BACK_POSITION);
            targetTurnPosition = TURN_BACK_POSITION;
        }
    }

    protected void handleLockingState() {
        currentTurnPosition = robotTop.getTurnPosition();
        if (currentTurnPosition > targetTurnPosition + 0.019) {
            robotTop.setTurnPosition(currentTurnPosition - 0.019);
        } else if (currentTurnPosition < targetTurnPosition - 0.019) {
            robotTop.setTurnPosition(currentTurnPosition + 0.019);
        } else {
            robotTop.setTurnPosition(targetTurnPosition);
            armState = ArmState.LOCKED;
        }
    }

    protected void handleUnlockingState() {
        currentTurnPosition = robotTop.getTurnPosition();
        if (currentTurnPosition > targetTurnPosition + 0.014) {
            robotTop.setTurnPosition(currentTurnPosition - 0.014);
        } else if (currentTurnPosition < targetTurnPosition - 0.014) {
            robotTop.setTurnPosition(currentTurnPosition + 0.014);
        } else {
            robotTop.setTurnPosition(targetTurnPosition);
            armState = ArmState.IDLE;
        }
    }

    protected void handleDisabledState() {
        if (armState != ArmState.IDLE || armState != ArmState.LOCKED) {
            liftState = LiftState.RUNNING;
        }
        if (gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0|| gamepad2.right_trigger != 0 || gamepad2.left_trigger != 0 ) {
            // robotTop.setTurnPosition(TURN_DOWN_POSITION - 0.1);
            targetTurnPosition = TURN_LOCK_POSITION;
            armState = ArmState.LOCKING;
            liftState = LiftState.RUNNING;
        }
    }

    protected void handleRunningState() {
        if (gamepad1.right_trigger != 0 || gamepad2.right_trigger != 0 ) {
            robotTop.setLiftPower(0.7);
            robotTop.setTopServoPosition(TOP_BACK);
            robotTop.setLiftTargetPos(robotTop.getLiftPosition());
        } else if (gamepad1.left_trigger!= 0 || gamepad2.left_trigger != 0) {
            robotTop.setLiftPower(-0.25);
            robotTop.setTopServoPosition(TOP_BACK);
            robotTop.setLiftTargetPos(robotTop.getLiftPosition());
        } else {
            if (robotTop.getLiftPosition() >= 200) {
                robotTop.updateLiftPID();
            } else {
                robotTop.setLiftPower(0);
            }
        }
        if (gamepad2.left_bumper && !previousGamepad2.left_bumper) {
            if (topServoOut) {
                robotTop.setTopServoPosition(TOP_BACK);
            } else {
                robotTop.setTopServoPosition(TOP_OUT);
            }
            topServoOut = !topServoOut;
        }
    }

    // *--------------- Auto in Manual ----------------*
    public void runAutoHanging(){
        robotAuto.resetPosition();
        sleep(200);
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging[0],posHanging[1],posHanging[2]),
                        new ForwardCommand(robotAuto,-8)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
                        new SleepCommand(300),
                        new SetLiftPositionCommand(robotAuto,800)
                )
        );
        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.4)),
                new SleepCommand(300),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release),
                new InstantCommand(() -> robotTop.setLiftPower(0.4)),
                new SleepCommand(100),
                new InstantCommand(() -> robotTop.setLiftPower(0))
        );
        // run autoMode
        cmd1.init();
        while (cmd1.hasNext() && !gamepad2.x){
            cmd1.iterate();
        }
        cmd1.finish();
        robotChassis.stopMotor();
        robotTop.setLiftPower(0);
//        cmd2.init();
//        while (cmd1.hasNext() && gamepad2.y){
//            cmd1.iterate();
//        }
//        cmd2.finish();
    }

    public void runAutoBack(){
        robotAuto.resetPosition();
        sleep(200);
        Command cmd3 = new GotoPosWithHeadingCommand(robotAuto,posHanging[0],posHanging[1],posHanging[2]);
        cmd3.init();
        while (cmd3.hasNext() && !gamepad2.x){
            cmd3.iterate();
        }
        cmd3.finish();
        robotChassis.stopMotor();
        robotTop.setLiftPower(0);
    }
}
