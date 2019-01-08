
/*
 * v_Closest_va.java
 */
package com.github.hy.broadcast;

import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.clay.*;

public class v_Momentum extends NodeVec2 {
    /**
     * Turns debugging on or off.
     */
    public static final boolean DEBUG = false;
    private double agging_rate = 0;

    /**
     * Instantiate a v_Momentum node.
     *
     * @param im1 NodeVec2, the embedded node that generates a list of items to
     *            scan.
     */
    public v_Momentum(NodeVec2 src, double agging_rate) {
        if (DEBUG)
            System.out.println("v_Momentum: instantiated.");
        this.src = src;
        last_val.setr(0);
        this.agging_rate = agging_rate;
    }

    NodeVec2 src;
    Vec2 last_val = new Vec2();

    long lasttime = 0;

    /**
     * Return a Vec2 representing the Momentum preserved.
     *
     * @param timestamp long, only get new information if timestamp > than last call
     *                  or timestamp == -1.
     * @return the vector.
     */
    public Vec2 Value(long timestamp) {
        if (last_val.r < 1e-2) {
            last_val.r = 0;
        } else if (last_val.r > 0) {
            last_val.setr(last_val.r * agging_rate);
        }

        if ((timestamp > lasttime) || (timestamp == -1)) {
            /*--- reset the timestamp ---*/
            if (timestamp > 0) {
                lasttime = timestamp;
            }
            Vec2 src_val = src.Value(timestamp);
            last_val.add(src_val);
        }

        return (new Vec2(last_val.x, last_val.y));
    }
}
