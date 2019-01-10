
/*
 * Algorithm.java
 */
package com.github.hy.utils;

import java.util.PriorityQueue;
import com.github.hy.map.MapStatus;

import java.util.*;

public class Algorithm {

    static int[] dx = new int[] { 1, -1, 0, 0, 1, 1, -1, -1 };
    static int[] dy = new int[] { 0, 0, 1, -1, 1, -1, 1, -1 };

    static class AStarCell {
        public Pair<Integer, Integer> parent;
        public double f, h, g;

        public AStarCell(Pair<Integer, Integer> parent, double f, double h, double g) {
            this.parent = parent;
            this.f = f;
            this.g = g;
            this.h = h;
        }
    }

    static class QueueComparator implements Comparator<Pair<Double, Pair<Integer, Integer>>> {

        // Overriding compare()method of Comparator
        public int compare(Pair<Double, Pair<Integer, Integer>> s1, Pair<Double, Pair<Integer, Integer>> s2) {
            if (s1.first < s2.first)
                return 1;
            else if (s1.first > s2.first)
                return -1;
            return 0;
        }
    }

    static public List<Pair<Integer, Integer>> AStar(SparseMap<MapStatus> sparse_map, Pair<Integer, Integer> start,
            Pair<Integer, Integer> end) {
        // return the shortest path from the start point to the end point using A start
        // algorithm.

        // set all REACHABLE to UNKNOWN
        sparse_map.changeAll(MapStatus.REACHABLE, MapStatus.UNKNOWN);
        Pair<Integer, Integer> new_end = new Pair<Integer, Integer>(start.first, start.second);
        getLatest(sparse_map, start, end, new_end);
        end = new_end;

        // do A star
        SparseMap<Boolean> closed = new SparseMap<Boolean>();
        SparseMap<AStarCell> cells = new SparseMap<AStarCell>();

        PriorityQueue<Pair<Double, Pair<Integer, Integer>>> // item: <f,<x,y>>
        open = new PriorityQueue<Pair<Double, Pair<Integer, Integer>>>(new QueueComparator());

        cells.put(start.first, start.second, new AStarCell(start, 0, 0, 0));

        open.add(new_p_p(0, start));
        while (!open.isEmpty()) {
            Pair<Double, Pair<Integer, Integer>> p = open.poll();
            double p_f = p.first;
            Pair<Integer, Integer> p_pos = p.second;
            closed.put(p_pos.first, p_pos.second, true);

            double g_new, f_new, h_new;
            for (int i = 0; i < dx.length; ++i) {
                int x = p_pos.first + dx[i];
                int y = p_pos.second + dy[i];
                Pair<Integer, Integer> p_Successor = new Pair<Integer, Integer>(x, y);

                if (x == end.first && y == end.second) {
                    cells.put(x, y, new AStarCell(p_pos, 0, 0, 0));
                    return tracePath(cells, end);
                } else if (!Boolean.TRUE.equals(closed.get(x, y)) && sparse_map.get(x, y) == MapStatus.REACHABLE) {
                    g_new = cells.get(p_pos.first, p_pos.second).g + 1.0;
                    h_new = heuristic(p_Successor, end);
                    f_new = g_new + h_new;

                    AStarCell tmp = cells.get(x, y);
                    if (tmp == null || tmp.f > f_new) {
                        open.add(new_p_p(f_new, p_Successor));
                        cells.put(x, y, new AStarCell(p_pos, f_new, h_new, g_new));
                    }

                }

            }
        }

        return null;
    }

    // A Utility Function to trace the path from the source to destination
    static List<Pair<Integer, Integer>> tracePath(SparseMap<AStarCell> cells, Pair<Integer, Integer> end) {
        int row = end.first;
        int col = end.second;

        Stack<Pair<Integer, Integer>> path = new Stack<Pair<Integer, Integer>>();

        AStarCell tmp = cells.get(row, col);
        Pair<Integer, Integer> last = new Pair<Integer, Integer>(row, col);
        while (!(tmp.parent.first == row && tmp.parent.second == col)) {
            path.push(last);
            row = tmp.parent.first;
            col = tmp.parent.second;
            last = tmp.parent;
            tmp = cells.get(row, col);
        }

        path.push(last);

        List<Pair<Integer, Integer>> ret = new ArrayList<Pair<Integer, Integer>>(path.size());
        while (!path.empty()) {

            // Pair<Integer, Integer> tmp_p = path.pop();
            // System.out.println("-> (" + tmp_p.first + "," + tmp_p.second + ")");
            // ret.add(tmp_p);
            ret.add(path.pop());
        }

        return ret;
    }

    static Pair<Double, Pair<Integer, Integer>> new_p_p(double f, Pair<Integer, Integer> p) {
        return new Pair<Double, Pair<Integer, Integer>>(f, p);
    }

    static private void getLatest(SparseMap<MapStatus> sparse_map, Pair<Integer, Integer> current,
            Pair<Integer, Integer> end, Pair<Integer, Integer> closest) {
        // get the closest point to 'end', which is reachable start from 'start'
        sparse_map.put(current.first, current.second, MapStatus.REACHABLE);
        if (current.first == end.first && current.second == end.second) {
            closest.first = end.first;
            closest.second = end.second;
            return;
        } else if (distance(current, end) < distance(closest, end)) {
            closest.first = current.first;
            closest.second = current.second;
        }
        for (int i = 0; i < dx.length; ++i) {
            int x = current.first + dx[i];
            int y = current.second + dy[i];
            if (sparse_map.get(x, y) == MapStatus.UNKNOWN) {
                getLatest(sparse_map, new Pair<Integer, Integer>(x, y), end, closest);
            }
        }
    }

    static double heuristic(Pair<Integer, Integer> x, Pair<Integer, Integer> end) {
        return distance(x, end);
    }

    static private double distance(Pair<Integer, Integer> a, Pair<Integer, Integer> b) {
        return chebyshev_distance(a, b);
    }

    static private double chebyshev_distance(Pair<Integer, Integer> a, Pair<Integer, Integer> b) {
        // Chebyshev distance.
        int dx = Math.abs(a.first - b.first);
        int dy = Math.abs(a.second - b.second);
        int D1 = 1, D2 = 1;
        return D1 * (dx + dy) + (D2 - 2 * D1) * Math.min(dx, dy);
    }

    public static void main(String argv[]) {
        // TODO: TEST if Iterger pass by value?
        Integer a = 1;
        Integer b = a;
        b = 2;
        System.out.print(a);
        System.out.print(b); // expected 1 2

        SparseMap<MapStatus> sparse_map = new SparseMap<MapStatus>();
        sparse_map.put(0, 0, MapStatus.REACHABLE);
        sparse_map.put(1, 1, MapStatus.REACHABLE);
        sparse_map.put(2, 2, MapStatus.REACHABLE);
        sparse_map.put(3, 3, MapStatus.REACHABLE);
        sparse_map.put(3, 4, MapStatus.REACHABLE);
        sparse_map.put(4, 3, MapStatus.REACHABLE);

        Pair<Integer, Integer> start = new Pair<Integer, Integer>(0, 0);
        Pair<Integer, Integer> end = new Pair<Integer, Integer>(5, 5);

        Algorithm.AStar(sparse_map, start, end);
        return;
    }

}
