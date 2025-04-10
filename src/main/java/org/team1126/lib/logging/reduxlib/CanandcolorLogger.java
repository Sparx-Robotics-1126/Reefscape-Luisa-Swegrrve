package org.team1126.lib.logging.reduxlib;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorStatus;
import com.reduxrobotics.sensors.canandcolor.ColorData;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(Canandcolor.class)
public class CanandcolorLogger extends ClassSpecificLogger<Canandcolor> {

    public CanandcolorLogger() {
        super(Canandcolor.class);
    }

    @Override
    public void update(EpilogueBackend backend, Canandcolor canandcolor) {
        backend.log("color", canandcolor.getColor(), ColorData.struct);
        backend.log("proximity", canandcolor.getProximity());
        backend.log("connected", canandcolor.isConnected());
        backend.log("status", canandcolor.getStatusFrame().getValue(), CanandcolorStatus.struct);
    }
}
