
/*
 * va_FilterOutLastPositions.java
 */
package com.github.hy.utils;

import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.communication.*;
import java.util.HashSet;
import java.util.Set;
import java.util.Collections;

public class va_FilterOutLastPositions extends NodeVec2Array {
    /**
     * Turn debug printing on or off.
     */
    public static final boolean DEBUG = false;
    NodeMsgArray msgs;

    Vec2 last_val[] = new Vec2[0];
    long lasttime = 0;

    /**
     * Instantiate a va_VisualObjects_r node.
     *
     * @param msgs NodeMsgArray, the raw Messages to be filtered out.
     */
    public va_FilterOutLastPositions(NodeMsgArray msgs) {
        if (DEBUG)
            System.out.println("va_FilterOutLastPositions: instantiated");
        this.msgs = msgs;
    }

    /**
     * Return an array of Positions from the last PositionsMessage.
     *
     * @param timestamp long, only get new information if timestamp > than last call
     *                  or timestamp == -1.
     * @return the messages recieved
     */
    public Vec2[] Value(long timestamp) {
        if ((timestamp > lasttime) || (timestamp == -1)) {
            /*--- reset the timestamp ---*/
            if (timestamp > 0) {
                lasttime = timestamp;
            }

            /*--- count ---*/
            Message[] arr_all = msgs.Value(timestamp);
            Vec2[] elements = new Vec2[0];
            Set<Vec2> positions = new HashSet<Vec2>();
            for (int i = arr_all.length - 1; i >= 0; i--) {
                if (arr_all[i] instanceof PositionsMessage) {
                    elements = ((PositionsMessage) arr_all[i]).val;
                    Collections.addAll(positions, elements);
                }
            }
            last_val = new Vec2[positions.size()];
            last_val = positions.toArray(last_val);
        }

        Vec2[] retval = new Vec2[last_val.length];
        for (int i = 0; i < last_val.length; i++)
            retval[i] = new Vec2(last_val[i].x, last_val[i].y);

        return (retval);
    }
}
