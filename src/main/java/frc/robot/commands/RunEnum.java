package frc.robot.commands;

public enum RunEnum {
        
        RunLoadingZone, //Starts in community around charge station, heads to loading zone
        RunBottomChargeStation, //Starts in loading zone, heads to bottom of charge station
        RunTopChargeStation, //Starts in loading zone, heads to top of charge station

        MoveBottomGrid, //Moves to bottom of community
        MoveMiddleGrid, //Moves to middle of community
        MoveTopGrid //Moves to top of community

}
