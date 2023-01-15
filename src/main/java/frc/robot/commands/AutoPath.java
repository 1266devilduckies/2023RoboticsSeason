package frc.robot.commands;

public enum AutoPath {
    
    HIGH_DOCKHIGH("HighDockingHighPath"),
    HIGH_DOCKLOW("HighDockingLowPath"),
    HIGH_FORWARD("HighForwardPath"),

    LOW_DOCKHIGH("LowDockingHighPath"),
    LOW_DOCKLOW("LowDockingLowPath"),
    LOW_FORWARD("LowForwardPath");

    private final String pathName;

    private AutoPath(String pathName){
        this.pathName = pathName;
    }

    public String getPathName(){
        return this.pathName;
    }
}
