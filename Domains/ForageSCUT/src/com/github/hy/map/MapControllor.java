
/*
 * MapControllor.java
 */
package com.github.hy.map;

import com.github.hy.utils.PositionsMessageType;

import EDU.gatech.cc.is.abstractrobot.MultiForageN150;
import EDU.gatech.cc.is.util.Vec2;
import com.github.hy.utils.*;
import EDU.gatech.cc.is.communication.*;

public class MapControllor {
    static final double sqrt2 = Math.sqrt(2);
    double granularity; /*
                         * the granularity of the map. If it is 1, then the real position of robot
                         * (1.1,2.5) will save as (1, 3) in tht map for '1' = '1.1/1' and '3=2.5/1'. If
                         * it is 0.1, then (1.1,2.5) will saved as (11,25)
                         */
    Vec2[] pos_reachable_cache; // the reachable position cached
    int pos_reachable_size;

    SparseMap<MapStatus> sparse_map = new SparseMap<MapStatus>();// a grid map based on sparse matrix

    protected MultiForageN150 abstract_robot;

    public MapControllor(MultiForageN150 abstract_robot, int pos_reachable_size, double granularity) {
        this.abstract_robot = abstract_robot;
        this.granularity = granularity;
        this.pos_reachable_cache = new Vec2[pos_reachable_size];
        this.pos_reachable_size = 0;
    }

    /*
     * Collect the position of the robot and cache it. Broadcast the positions
     * cached when the space is full.
     */
    public void collectAndBroadcastCurrentPos() {
        // the reachable position
        Vec2[] obstacles = abstract_robot.getObstacles(-1);
        double min_r = abstract_robot.VISION_RANGE;
        for (int i = 0; i < obstacles.length; ++i) {
            if (obstacles[i].r < min_r) {
                min_r = obstacles[i].r;
            }
        }

        // the edge of square in the circle with radius min_r, which is reachable
        double a = min_r / sqrt2;

        Vec2 pos = abstract_robot.getPosition(-1);
        double l = Math.ceil((pos.x - a) / granularity) * granularity;
        double r = Math.floor((pos.x + a) / granularity) * granularity;
        double u = Math.floor((pos.y - a) / granularity) * granularity;
        double d = Math.floor((pos.y + a) / granularity) * granularity;

        for (double i = l; i <= r; i += granularity) {
            for (double j = d; j <= u; j += granularity) {
                collectAndBroadcastCurrentPos(i, j);
            }
        }

    }

    private void collectAndBroadcastCurrentPos(Vec2 pos) {
        collectAndBroadcastCurrentPos(pos.x, pos.y);
    }

    private void collectAndBroadcastCurrentPos(double x, double y) {
        if (pos_reachable_size >= pos_reachable_cache.length) {
            copyAndBroadcast(pos_reachable_cache, PositionsMessageType.MAP_REACHABLE);
            pos_reachable_size = 0;
        }
        if (sparse_map.get(convert(x), convert(y)) != MapStatus.REACHABLE) {
            pos_reachable_cache[pos_reachable_size++] = new Vec2(x, y);
            updateMap(sparse_map, x, y, MapStatus.REACHABLE);
        }
    }

    private void copyAndBroadcast(Vec2[] cache, PositionsMessageType message_type) {
        Vec2[] p = new Vec2[cache.length];
        for (int i = 0; i < cache.length; i++) {
            p[i] = (Vec2) cache[i].clone();
        }
        Message m = new PositionsMessage(p, message_type);
        abstract_robot.broadcast(m);
    }

    public void updateMap(Vec2[] positions, MapStatus status) {
        // convert position according to granularity and put to sparse_map;
        updateMap(sparse_map, positions, status);
    }

    // set the 'status' of the positions in map
    private void updateMap(SparseMap<MapStatus> m, Vec2[] positions, MapStatus status) {
        // convert position according to granularity and put to sparse_map;
        for (int i = 0; i < positions.length; ++i) {
            updateMap(m, positions[i].x, positions[i].y, status);
        }
    }

    private void updateMap(SparseMap<MapStatus> m, double x, double y, MapStatus status) {
        // convert position according to granularity and put to sparse_map;
        m.put(convert(x), convert(y), status);
    }

    private int convert(double x) {
        // convert x according granularity
        return Math.toIntExact(Math.round(x / granularity));
    }

    private SparseMap<MapStatus> getRealtimeMap() {
        SparseMap<MapStatus> ret = sparse_map.clone();
        Vec2[] obstacles = abstract_robot.getObstacles(-1);
        updateMap(ret, obstacles, MapStatus.UNREACHABLE);
        return sparse_map;
    }

    public Vec2[] shortestPath(Vec2 start, Vec2 end) {
        // return the shortest path from the start point to the end point;
        // 1. convert Vec2 to (int,int)
        start.setx(convert(start.x));
        start.setx(convert(start.y));
        end.setx(convert(end.x));
        end.setx(convert(end.y));

        // 2. run algorithm on sparse_map
        SparseMap<MapStatus> realtime_map = getRealtimeMap();

        Vec2[] ret = Algorithm.AStar(realtime_map, start, end);

        // 3. convert (int,int) to Vec2
        for (int i = 0; i < ret.length; ++i) {
            ret[i].setx(ret[i].x * granularity);
            ret[i].setx(ret[i].y * granularity);
        }

        return ret;
    }

}
