/*
 * v_ShortestPathNextPosition.java
 */

package com.github.hy.map;

import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.clay.*;

public class v_ShortestPathNextPosition extends NodeVec2 {
    /**
     * Turns debug printing on or off.
     */
    public static final boolean DEBUG = /* true; */ Node.DEBUG;
    private NodeVec2 start_local, end_local, global_base;
    private double controlled_zone = 1.0;
    private MapControllor mapControllor;

    /**
     * Instantiate a v_ShortestPathNextPosition schema.
     *
     * @param czr double, controlled zone radius.
     */
    public v_ShortestPathNextPosition(double controlled_zone, MapControllor mapControllor, NodeVec2 global_base,
            NodeVec2 start_local, NodeVec2 end_local) {
        if (DEBUG)
            System.out.println("v_ShortestPathNextPosition: instantiated.");
        if (controlled_zone <= 0) {
            return;
        }
        this.start_local = start_local;
        this.end_local = end_local;
        this.controlled_zone = controlled_zone;
        this.mapControllor = mapControllor;
        this.global_base = global_base;
    }

    Vec2 last_val = new Vec2();
    long lasttime = 0;

    /**
     * Return a Vec2 representing the direction to go towards the goal. Magnitude
     * varies with distance.
     *
     * @param timestamp long, only get new information if timestamp > than last call
     *                  or timestamp == -1.
     * @return the movement vector.
     */
    public Vec2 Value(long timestamp) {
        if ((timestamp > lasttime) || (timestamp == -1)) {
            if (DEBUG)
                System.out.println("v_ShortestPathNextPosition:");

            /*--- reset the timestamp ---*/
            if (timestamp > 0)
                lasttime = timestamp;

            Vec2 start_local_val = start_local.Value(timestamp);
            Vec2 end_local_val = end_local.Value(timestamp);
            Vec2 global_base_val = global_base.Value(timestamp);

            start_local_val.add(global_base_val);
            end_local_val.add(global_base_val);

            Vec2[] path = mapControllor.shortestPath(start_local_val, end_local_val);

            if (path.length > 0) {
                // System.out.println("path length:" + path.length);
                last_val = (Vec2) end_local_val.clone();
                last_val.sub(global_base_val);

                for (int i = 0; i < path.length; ++i) {
                    path[i].sub(global_base_val);
                    if (path[i].r > 1.0) {
                        last_val = (Vec2) path[i].clone();
                        System.out.println("path length:" + path.length);
                        break;
                    }
                }
                // last_val = (Vec2) path[9].clone();
                last_val.setr(1.0);
            } else {
                last_val = (Vec2) end_local_val.clone();
                last_val.sub(global_base_val);
            }

        }
        return (new Vec2(last_val.x, last_val.y));
    }
}
