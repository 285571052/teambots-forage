
/*
 * SparseMap.java
 */
package com.github.hy.utils;

import java.io.FileWriter;
import java.util.*;
import java.io.*;

public class SparseMap<T> implements Cloneable {
    Map<Integer, Map<Integer, T>> elements = new HashMap<Integer, Map<Integer, T>>();

    public void put(Integer x, Integer y, T v) {
        Map<Integer, T> row = elements.get(x);
        if (row == null) {
            elements.put(x, new HashMap<Integer, T>());
            row = elements.get(x);
        }
        row.put(y, v);
    }

    public T get(Integer x, Integer y) {
        Map<Integer, T> row = elements.get(x);
        if (row == null)
            return null;
        return row.get(y);
    }

    public SparseMap<T> clone() {
        SparseMap<T> ret = new SparseMap<T>();

        for (Map.Entry<Integer, Map<Integer, T>> row_entry : elements.entrySet()) {
            int x = row_entry.getKey();
            for (Map.Entry<Integer, T> v_entry : row_entry.getValue().entrySet()) {
                int y = v_entry.getKey();
                T v = v_entry.getValue();
                ret.put(x, y, v);
            }
        }

        return ret;
    }

    public void changeAll(T from, T to) {
        List<Pair<Integer, Integer>> list = new LinkedList<Pair<Integer, Integer>>();

        for (Map.Entry<Integer, Map<Integer, T>> row_entry : elements.entrySet()) {
            int x = row_entry.getKey();
            for (Map.Entry<Integer, T> v_entry : row_entry.getValue().entrySet()) {
                int y = v_entry.getKey();
                T v = v_entry.getValue();
                if (v == from) {
                    list.add(new Pair<Integer, Integer>(x, y));
                }
            }
        }

        for (Pair<Integer, Integer> p : list) {
            put(p.first, p.second, to);
        }

    }

    public int size() {
        int cnt = 0;
        for (Map.Entry<Integer, Map<Integer, T>> row_entry : elements.entrySet()) {
            cnt += row_entry.getValue().size();
        }
        return cnt;
    }

    public void save(String file_path) {
        try {
            FileWriter writer = new FileWriter(file_path);
            for (Map.Entry<Integer, Map<Integer, T>> row_entry : elements.entrySet()) {
                int x = row_entry.getKey();
                for (Map.Entry<Integer, T> v_entry : row_entry.getValue().entrySet()) {
                    int y = v_entry.getKey();
                    T v = v_entry.getValue();
                    writer.write("(" + x + "," + y + "," + v + ")\n");
                }
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void main(String argv[]) {
        SparseMap<Integer> sparse_map = new SparseMap<Integer>();
        sparse_map.put(1, 2, 3);
        System.out.println(sparse_map.get(1, 2));
        sparse_map.put(1, 7, 8);
        System.out.println(sparse_map.get(1, 7));
        sparse_map.put(4, 5, 6);
        System.out.println(sparse_map.get(4, 5));
        sparse_map.put(1, 7, 9);
        System.out.println(sparse_map.get(1, 7));
        System.out.println(sparse_map.get(9, 9));
        // output should be 3 8 6 9 null

        SparseMap<Integer> sparse_map_clone = sparse_map.clone();
        sparse_map.put(9, 9, 9);
        System.out.println(sparse_map.get(9, 9));
        sparse_map.put(1, 7, 8);
        System.out.println(sparse_map.get(1, 7));
        System.out.println(sparse_map_clone.get(1, 2));
        System.out.println(sparse_map_clone.get(1, 7));
        System.out.println(sparse_map_clone.get(4, 5));
        System.out.println(sparse_map_clone.get(9, 9));
        // output should be 9 8 3 9 6 null
    }
}
