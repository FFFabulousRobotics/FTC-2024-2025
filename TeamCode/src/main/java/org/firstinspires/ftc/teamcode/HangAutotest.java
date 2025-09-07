package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.auto.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.auto.commands.ForwardCommand;
import org.firstinspires.ftc.teamcode.auto.commands.GotoPosCommand;
import org.firstinspires.ftc.teamcode.auto.commands.GotoPosWithHeadingCommand;
import org.firstinspires.ftc.teamcode.auto.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.auto.commands.SetLiftPositionCommand;
import org.firstinspires.ftc.teamcode.auto.commands.SleepCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotAuto;
import org.firstinspires.ftc.teamcode.hardware.RobotTop;

@Autonomous
public class HangAutotest extends LinearOpMode {
    final int STRETCH_BACK_POSITION = 70;
    final int STRETCH_OUT_POSITION = 1500;
    final double SPIN_DEFAULT_POSITION_L = 0.54;
    final double SPIN_DEFAULT_POSITION_R = 0.51;
    final double SPIN_HOVERING_POSITION_L = 0.25;
    final double SPIN_HOVERING_POSITION_R = 0.78;
    final double SPIN_DOWN_POSITION = 0;
    final double TURN_BACK_POSITION = 0.5;
    final double TURN_LOCK_POSITION = 0.68;
    final double TURN_HOVERING_POSITION = 0.75;
    final double TURN_DOWN_POSITION = 0.85;
    final double GRAB_OPEN_POSITION = 0.4;
    final double GRAB_CLOSE_POSITION = 0.92;
    final double TOP_BACK = 0.03;
    final double TOP_OUT = 0.66;
    final double BACK_CLOSE = 0.7;
    final double BACK_OPEN = 0.1;

    final private double[] posHanging = {10.88, -26.5, 0};//挂的地方
    final private double[] posHanging2 = {14.41, -25.56, 0};//第二次挂的地方(有误差)
    final private double[] posHanging3 = {8.70, -27.56, 0};//第三次挂的地方(有误差)
    final private double[] posHanging4 = {6.70, -27.56, 0};//第四次挂的地方(有误差)
    final private double[] PosReadyForPushzhe1 = {-19.2766,-31.5603,0};//推第一下的停靠点
    final private double[] posMiddleStop = {-13.53, -10.84, 0};//中间停靠点
    final private double[] posReadyForPush = {-27, -51.72, 0};//准备推的位置
    //final private double[] posReadyForPushzhe2 ={-21.54,-45.08,0};//推第二次的转折点
    final private double[] posReadyForPush2 = {-39.46,-50.44,0};//推第二个的位置
    final private double[] posPushed = {-30.85, -8.17, -4};//推到这个位置
    final private double[] posBack = {-25.65, -19.12, 180};//回一点位置
    final private double[] posGet = {-18,-3,180};//拿方块的地方
    final private double[] posGet4 = {-18,-3,180};//拿第四个方块的地方
    //final private double[] parkPosition = {-40.0, 13.0, 270};//停靠

    RobotTop robotTop;
    RobotAuto robotAuto;

    @Override
    public void runOpMode(){
        robotTop = new RobotTop(this);
        robotAuto = new RobotAuto(this);
        waitForStart();
        robotTop.setArmLeftSpinPosition(SPIN_DEFAULT_POSITION_L);
        robotTop.setArmRightSpinPosition(SPIN_DEFAULT_POSITION_R);
        robotTop.setLiftServoPosition(BACK_CLOSE);
        hangSample1();
        pushSample();
        hangSample2();
        pushSample2();
        getSample();
        hangSample3();
        getSample2();
        hangsample4();
        //park();
    }
    protected void hangSample1(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging[0],posHanging[1],posHanging[2]),
                        new ForwardCommand(robotAuto,-5)


                ),
                new SequentialCommandGroup(
//                        new SleepCommand(100),
//                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
//                        new SleepCommand(300),
                        new SetLiftPositionCommand(robotAuto,900)
                )
        );
        cmd1.runCommand();
        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.6)),
                new SleepCommand(200),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                //new SetLiftPositionCommand(robotAuto,900),
                new InstantCommand(robotAuto::release),
//                new ForwardCommand(robotAuto,2),
        new InstantCommand(() -> robotTop.setTurnPosition(TURN_BACK_POSITION))
        );
        cmd2.runCommand();
    }
    protected void pushSample(){
        SequentialCommandGroup cmd3 = new SequentialCommandGroup(
                //new GotoPosCommand(robotAuto,posMiddleStop[0],posMiddleStop[1],true),
                new GotoPosWithHeadingCommand(robotAuto,PosReadyForPushzhe1 [0],PosReadyForPushzhe1[1],PosReadyForPushzhe1[2],true),
                new GotoPosWithHeadingCommand(robotAuto,posReadyForPush[0],posReadyForPush[1],posReadyForPush[2],true),
                new GotoPosWithHeadingCommand(robotAuto,posPushed[0],posPushed[1],posPushed[2],true),
                //new GotoPosWithHeadingCommand(robotAuto,posBack[0],posBack[1],posBack[2]),
                new GotoPosWithHeadingCommand(robotAuto,posGet[0],posGet[1],posGet[2]),
                new ForwardCommand(robotAuto,-9),
                new InstantCommand(robotAuto::grab),
                new ForwardCommand(robotAuto,1)//,
                //new SleepCommand(500)
        );
        cmd3.runCommand();
    }
    protected void pushSample2(){
        SequentialCommandGroup cmd3 = new SequentialCommandGroup(
                //new GotoPosCommand(robotAuto,posMiddleStop[0],posMiddleStop[1],true),
                new GotoPosWithHeadingCommand(robotAuto,PosReadyForPushzhe1[0],PosReadyForPushzhe1[1],PosReadyForPushzhe1[2],true),
                new GotoPosWithHeadingCommand(robotAuto,posReadyForPush2[0],posReadyForPush2[1],posReadyForPush2[2],true),
                new GotoPosWithHeadingCommand(robotAuto,posPushed[0],posPushed[1],posPushed[2],true),
                //new GotoPosWithHeadingCommand(robotAuto,posBack[0],posBack[1],posBack[2]),
                new GotoPosWithHeadingCommand(robotAuto,posGet[0],posGet[1],posGet[2]));
                //new ForwardCommand(robotAuto,-10));

                //new SleepCommand(500)
        cmd3.runCommand();
        }
    protected void hangSample2(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging2[0],posHanging2[1],posHanging2[2]),
                        new ForwardCommand(robotAuto,-5)




                ),
                new SequentialCommandGroup(
//                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
//                        new SleepCommand(300),
                        new SetLiftPositionCommand(robotAuto,850)
                )
        );
        cmd1.runCommand();
        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.6)),
                new SleepCommand(200),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release),
                new InstantCommand(() -> robotTop.setLiftPower(0.45)),
                new SleepCommand(100),
                new InstantCommand(() -> robotTop.setLiftPower(0))//,
                //new ForwardCommand(robotAuto,2)
        );
        cmd2.runCommand();

    }
    protected void getSample(){
        SequentialCommandGroup cmd5 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new GotoPosWithHeadingCommand(robotAuto,posGet[0],posGet[1],posGet[2]),
                new ForwardCommand(robotAuto,-9),
                new InstantCommand(robotAuto::grab),
                new ForwardCommand(robotAuto,1)
                //new SleepCommand(500)
        );
        cmd5.runCommand();
    }
    protected void hangSample3(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging3[0],posHanging3[1],posHanging3[2]),
                        new ForwardCommand(robotAuto,-6.5)
                ),
                new SequentialCommandGroup(
//                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
//                        new SleepCommand(300),
                        new SetLiftPositionCommand(robotAuto,800)
                )
        );
        cmd1.runCommand();
        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.5)),
                new SleepCommand(150),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release),
                new InstantCommand(() -> robotTop.setLiftPower(0.5)),
                new SleepCommand(150),
                new InstantCommand(() -> robotTop.setLiftPower(0))
        );
        cmd2.runCommand();
    }
//    protected void park(){
//        ParallelCommandGroup cmd7 = new ParallelCommandGroup(
//                new SequentialCommandGroup(
//                        new GotoPosWithHeadingCommand(robotAuto,parkPosition[0],parkPosition[1],parkPosition[2])
//                ),
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_BACK_POSITION))
//                )
//        );
//        cmd7.runCommand();
//    }

    protected void getSample2(){
        SequentialCommandGroup cmd5 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new GotoPosWithHeadingCommand(robotAuto,posGet4[0],posGet4[1],posGet4[2]),
                new ForwardCommand(robotAuto,-9),
                new InstantCommand(robotAuto::grab),
                new ForwardCommand(robotAuto,1)//,
                //new SleepCommand(500)
        );
        cmd5.runCommand();
    }
    protected void hangsample4(){
        ParallelCommandGroup cmd1 = new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new GotoPosWithHeadingCommand(robotAuto,posHanging4[0],posHanging4[1],posHanging4[2]),
                        new ForwardCommand(robotAuto,-6.5)

                ),
                new SequentialCommandGroup(
//                        new InstantCommand(() -> robotTop.setTurnPosition(TURN_LOCK_POSITION)),
//                        new SleepCommand(300),
                        new SetLiftPositionCommand(robotAuto,800)
                )
        );
        cmd1.runCommand();
        SequentialCommandGroup cmd2 = new SequentialCommandGroup(
                new InstantCommand(() -> robotTop.setLiftPower(-0.6)),
                new SleepCommand(200),
                new InstantCommand(() -> robotTop.setLiftPower(0)),
                new InstantCommand(robotAuto::release),
                new InstantCommand(() -> robotTop.setLiftPower(0.45)),
                new SleepCommand(100),
                new InstantCommand(() -> robotTop.setLiftPower(0))
        );
        cmd2.runCommand();
    }
}

