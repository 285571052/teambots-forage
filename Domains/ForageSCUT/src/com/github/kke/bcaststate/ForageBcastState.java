
/*
 * ForageBroadcast.java
 */
package com.github.kke.bcaststate;

import EDU.gatech.cc.is.util.Vec2;
import EDU.gatech.cc.is.abstractrobot.*;
import EDU.gatech.cc.is.clay.*;
import EDU.gatech.cc.is.communication.*;

import com.github.hy.utils.*;
import com.github.kke.utils.*;

import java.util.*;


public class ForageBcastState extends ControlSystemMFN150 {
	public final static boolean DEBUG = true;
	private NodeVec2 turret_configuration;
	private NodeVec2 steering_configuration;
	private NodeVec2Array targets0_global;
	private i_FSA_ba STATE_MACHINE;


	private v_Cache_v 	cached_closest_global = null;
	private v_Cache_vb cached_closest_bcast_global = null;
	private v_Cache_v cached_history_global = new v_Cache_v(null);

	private NodeInt find_bcast_leader = null;
	private va_FilterOutFollowers gather_followers = null;
	private NodeBoolean has_follower_replys = null;

//	private NodeVec2 debug_cached_closest_target_global = null;
//	private NodeVec2 debug_cached_closest_bcast_global = null;
//	private NodeVec2 debug_cached_inertia_history_global = null;

	/**
	 * Configure the control system using Clay.
	 */
	public void configure() {
		// ======
		// Set some initial hardware configurations.
		// ======
		abstract_robot.setObstacleMaxRange(3.0);
		abstract_robot.setBaseSpeed(0.4 * abstract_robot.MAX_TRANSLATION);

		// ======
		// perceptual schemas
		// ======
		// 自己的坐标
		NodeVec2 PS_GLOBAL_POS = new v_GlobalPosition_r(abstract_robot);
		// 传感器探测得到的障碍物的EGO坐标
		NodeVec2Array PS_OBS = new va_Obstacles_r(abstract_robot);
		// 所有可视目标的EGO坐标
		NodeVec2Array PS_TARGETS_EGO = new va_VisualObjects_r(0, abstract_robot);
		// 所有可视目标的GLOBAL坐标
		NodeVec2Array PS_TARGETS_GLOBAL = new va_Add_vav(PS_TARGETS_EGO, PS_GLOBAL_POS);

		// 所有可视目标的EGO坐标。多余的操作？
		NodeVec2Array PS_TARGETS_EGO_FILT = new va_Subtract_vav(PS_TARGETS_GLOBAL, PS_GLOBAL_POS);
		// 挑离自己最近的一个EGO坐标
		NodeVec2 PS_CLOSEST = new v_Closest_va(PS_TARGETS_EGO_FILT);
		// 最近目标的GLOBAL坐标
		NodeVec2 PS_CLOSEST_GLOBAL = new v_EgoToGlobal_rv(abstract_robot, PS_CLOSEST);
		// 保存距离自己最近的GLOBAL坐标
		cached_closest_global = new v_Cache_v(PS_CLOSEST_GLOBAL);

		// 获得所有广播消息
		NodeMsgArray MSG_RECIEVE = new va_RecieveMessage(abstract_robot);
		// 过滤
		NodeVec2Array PS_BCAST_GLOBAL = new va_FilterOutLastPositions(MSG_RECIEVE, PositionsMessageType.LEADER);
		// 转换为EGO坐标
		NodeVec2Array PS_BCAST_EGO_FILT = new va_Subtract_vav(PS_BCAST_GLOBAL, PS_GLOBAL_POS);
		// 挑离自己最近的
		NodeVec2 PS_CLOSEST_BCAST = new v_Closest_va(PS_BCAST_EGO_FILT);
		// 转换为global坐标
		NodeVec2 PS_CLOSEST_BCAST_GLOBAL = new v_EgoToGlobal_rv(abstract_robot, PS_CLOSEST_BCAST);

		find_bcast_leader = new i_FindLeader_vma(PS_CLOSEST_BCAST_GLOBAL, MSG_RECIEVE);
		gather_followers = new va_FilterOutFollowers(MSG_RECIEVE);
		has_follower_replys = new b_NonZero_va(gather_followers);

		// ======
		// Perceptual Features
		// ======
		// 目标仍然在可视范围内
		NodeBoolean PF_TARGET_VISIBLE = new b_NonZero_v(PS_CLOSEST);
		// 目标逃离可视范围
		NodeBoolean PF_NOT_TARGET_VISIBLE = new b_Not_s(PF_TARGET_VISIBLE);
		// 收到广播消息
		NodeBoolean PF_BCAST_RECEIVED = new b_NonZero_v(PS_CLOSEST_BCAST);

		// 保存距离自己最近的广播GLOBAL目标
		cached_closest_bcast_global = new v_Cache_vb(PS_CLOSEST_BCAST_GLOBAL, PF_BCAST_RECEIVED);

//		NodeBoolean PF_BCAST_NOT_RECEIVED = new b_Not_s(PF_BCAST_RECEIVED);
		// 已经到达广播目标附近
		NodeBoolean PF_NEAR_CLOSEST_BCAST = new b_LessThan_v(
				new v_GlobalToEgo_rv(abstract_robot, cached_closest_bcast_global),
				MultiForageN150.VISION_RANGE
		);


		// 惯性定时器timeout
		NodeBoolean PF_HISTORY_TIMEO = new b_Timer_b(PF_TARGET_VISIBLE, 100, true);


		// ======
		// motor schemas
		// ======
		// 避开障碍物
		NodeVec2 MS_AVOID_OBSTACLES = new v_Avoid_va(1.5, abstract_robot.RADIUS + 0.1, PS_OBS);
		// 向目标移动
		NodeVec2 MS_MOVE_TO_CLOSEST = new v_LinearAttraction_v(0.4, 0.0, PS_CLOSEST);
		// 移动到广播目标需要的向量
		NodeVec2 MS_MOVE_TO_CLOSEST_BCAST = new v_LinearAttraction_v(
				0.4, 0.0, new v_GlobalToEgo_rv(abstract_robot, cached_closest_bcast_global)
		);
		// 移动到历史目标需要的向量		
		NodeVec2 MS_MOVE_TO_HISTORY = new v_LinearAttraction_v(
				0.4, 0.0, new v_GlobalToEgo_rv(abstract_robot, cached_history_global)
		);
		// noise vector
		NodeVec2 MS_NOISE_VECTOR = new v_Noise_(5, seed);
		// 在每个状态的每个动作时, 都增加一定的随机变化量

		// swirl obstacles wrt noise
		NodeVec2 MS_SWIRL_OBSTACLES_NOISE = new v_Swirl_vav(2.0, abstract_robot.RADIUS + 0.1, PS_OBS, MS_NOISE_VECTOR);

		// swirl obstacles wrt target 0
		NodeVec2 MS_SWIRL_OBSTACLES_TARGET0 = new v_Swirl_vav(2.0, abstract_robot.RADIUS + 0.22, PS_OBS, PS_CLOSEST);

		
		// ======
		// AS_WANDER
		// ======
		v_StaticWeightedSum_va AS_WANDER = new v_StaticWeightedSum_va();

		AS_WANDER.weights[0] = 1.0;
		AS_WANDER.embedded[0] = MS_AVOID_OBSTACLES; // 避开障碍

		AS_WANDER.weights[1] = 0.7;
		AS_WANDER.embedded[1] = MS_NOISE_VECTOR; // 随机移动

		AS_WANDER.weights[2] = 1.0;
		AS_WANDER.embedded[2] = MS_SWIRL_OBSTACLES_NOISE; // 绕障碍物转

		// ======
		// AS_ACQUIRE0
		// ======
		v_StaticWeightedSum_va AS_ACQUIRE0 = new v_StaticWeightedSum_va();

		AS_ACQUIRE0.weights[0] = 0.5;
		AS_ACQUIRE0.embedded[0] = MS_AVOID_OBSTACLES; // 避障

		AS_ACQUIRE0.weights[1] = 2.0;
		AS_ACQUIRE0.embedded[1] = MS_MOVE_TO_CLOSEST; // 移向目标

		AS_ACQUIRE0.weights[2] = 0.5;
		AS_ACQUIRE0.embedded[2] = MS_SWIRL_OBSTACLES_TARGET0; // 旋转向目标

		AS_ACQUIRE0.weights[3] = 0.2;
		AS_ACQUIRE0.embedded[3] = MS_NOISE_VECTOR; // 随机移动

		// ======
		// AS_INFORMED
		// ======
		v_StaticWeightedSum_va AS_INFORMED = new v_StaticWeightedSum_va();
		
		AS_INFORMED.weights[0] = 1;
		AS_INFORMED.embedded[0] = MS_AVOID_OBSTACLES; // 避障

		AS_INFORMED.weights[1] = 1.5;
		AS_INFORMED.embedded[1] = MS_MOVE_TO_CLOSEST_BCAST; // 向广播目标前进

		AS_INFORMED.weights[2] = 0.7;
		AS_INFORMED.embedded[2] = MS_NOISE_VECTOR; // 随机移动

//		AS_INFORMED.weights[3] = 1;
//		AS_INFORMED.embedded[3] = MS_SWIRL_OBSTACLES_TARGET0; // 旋转向目标

		// ======
		// AS_INERTIA
		// ======
		v_StaticWeightedSum_va AS_INERTIA = new v_StaticWeightedSum_va();

		AS_INERTIA.weights[0] = 0.6;
		AS_INERTIA.embedded[0] = MS_AVOID_OBSTACLES; // 避障

		AS_INERTIA.weights[1] = 1.5;
		AS_INERTIA.embedded[1] = MS_MOVE_TO_HISTORY; // 继续运动到原来的目标

		AS_INERTIA.weights[2] = 1;
		AS_INERTIA.embedded[2] = MS_SWIRL_OBSTACLES_TARGET0; // 旋转向目标

		AS_INERTIA.weights[3] = 0.5;
		AS_INERTIA.embedded[3] = MS_NOISE_VECTOR; // 随机移动

		// ======
		// STATE_MACHINE
		// ======
		STATE_MACHINE = new i_FSA_ba();

		STATE_MACHINE.state = 0;

		// STATE 0 WANDER
		STATE_MACHINE.triggers[0][0] = PF_TARGET_VISIBLE; // 是否发现目标
		STATE_MACHINE.follow_on[0][0] = 1; // ACQUIRE0
		STATE_MACHINE.triggers[0][1] = PF_BCAST_RECEIVED; // 收到有关目标位置的信息
		STATE_MACHINE.follow_on[0][1] = 2; // INFORMED


		// STATE 1 ACQUIRE0
		STATE_MACHINE.triggers[1][0] = PF_NOT_TARGET_VISIBLE;
		STATE_MACHINE.follow_on[1][0] = 3; // INERTIA


		// STATE 2 INFORMED
		STATE_MACHINE.triggers[2][0] = PF_TARGET_VISIBLE;
		STATE_MACHINE.follow_on[2][0] = 1; // ACQUIRE0
		STATE_MACHINE.triggers[2][1] = PF_NEAR_CLOSEST_BCAST; // 接近广播目标位置
		STATE_MACHINE.follow_on[2][1] = 3; // INERTIA

		
		// STATE 2 INERTIA
		STATE_MACHINE.triggers[3][0] = PF_HISTORY_TIMEO;
		STATE_MACHINE.follow_on[3][0] = 0; // WONDER
		STATE_MACHINE.triggers[3][1] = PF_TARGET_VISIBLE;
		STATE_MACHINE.follow_on[3][1] = 1; // ACQUIRE0
		STATE_MACHINE.triggers[3][2] = PF_BCAST_RECEIVED;
		STATE_MACHINE.follow_on[3][2] = 2; // INFORMED
		
		// ======
		// STEERING
		// ======
		v_Select_vai STEERING = new v_Select_vai((NodeInt) STATE_MACHINE);

		STEERING.embedded[0] = AS_WANDER;
		STEERING.embedded[1] = AS_ACQUIRE0;
		STEERING.embedded[2] = AS_INFORMED;
		STEERING.embedded[3] = AS_INERTIA;

		// ======
		// TURRET
		// ======
		v_Select_vai TURRET = new v_Select_vai((NodeInt) STATE_MACHINE);

		TURRET.embedded[0] = AS_WANDER;
		TURRET.embedded[1] = AS_ACQUIRE0;
		TURRET.embedded[2] = AS_INFORMED;
		TURRET.embedded[3] = AS_INERTIA;

		turret_configuration = TURRET;
		steering_configuration = STEERING;
		targets0_global = PS_TARGETS_GLOBAL;
	}

	/**
	 * Called every timestep to allow the control system to run.
	 */
	public int takeStep() {
		Vec2 result;
		long curr_time = abstract_robot.getTime();

		// STEER
		result = steering_configuration.Value(curr_time);
		abstract_robot.setSteerHeading(curr_time, result.t);
		abstract_robot.setSpeed(curr_time, 1.0);

		// TURRET
		result = turret_configuration.Value(curr_time);
		abstract_robot.setTurretHeading(curr_time, result.t);

		// STATE DISPLAY
		switch (STATE_MACHINE.Value(curr_time)) {
			case 0:{
				abstract_robot.setDisplayString("wander");
				cached_history_global.switchNode(null);
				break;
			}
			case 1: {
				actionOnAcquire(curr_time);
				break;
			}
			case 2: {
				actionOnInformed(curr_time);
				break;
			}
			case 3: {
				abstract_robot.setDisplayString("inertia");

//				Vec2 v = cached_history_global.Value(curr_time);
//				System.out.print(abstract_robot.getID() + " inertia history global(x:" + v.x + " y:" + v.y + ")\n");

				break;
			}
		}

		

		return (CSSTAT_OK);
	}

	public void actionOnInformed(long timestamp) {
		abstract_robot.setDisplayString("informed");

		cached_closest_bcast_global.storeValue(timestamp);
		cached_history_global.switchNode(cached_closest_bcast_global);
		cached_history_global.storeValue(timestamp);

		Vec2 v = cached_closest_bcast_global.Value(timestamp);
		int sender = find_bcast_leader.intValue(timestamp);
		System.out.printf(
				"%d informed bcast global(%f,%f) from %d\n",
				abstract_robot.getID(), v.x, v.y, sender);

		// 把自己的单播给自己选择的leader。
		if (sender != SourcePositionMessage.NO_ID) {

			Message m = new SourcePositionMessage(
					abstract_robot.getPosition(timestamp),
					PositionsMessageType.FOLLOWER,
					abstract_robot.getID()
			);
			try {
				abstract_robot.unicast(sender, m);
			} catch (CommunicationException e) {
				e.printStackTrace();
			}
		}
	}

	public void actionOnAcquire(long timestamp) {
		abstract_robot.setDisplayString("acquire");

		cached_closest_global.storeValue(timestamp);
		cached_history_global.switchNode(cached_closest_global);
		cached_history_global.storeValue(timestamp);


		Vec2 v = cached_closest_global.Value(timestamp);
		System.out.printf(
				"%d acquire target global(%f,%f) bcast\n",
				abstract_robot.getID(), v.x, v.y);

		if (has_follower_replys.Value(timestamp)) {

			for (int id: gather_followers.ids) {
				System.out.printf("\tFollower: %d\n", id);
			}

			Vec2[] followers = gather_followers.Value(timestamp);
			Vec2[] targets = targets0_global.Value(timestamp);

			Vec2[] result = positionAssignment(followers, targets, abstract_robot.getPosition(timestamp), 2);
			int[] ids = gather_followers.ids;
			for (int i = 0; i < followers.length; ++i) {

				System.out.printf("\tcaculated:(%f,%f)\n", result[i].x, result[i].y);

				Message m = new SourcePositionMessage(
						result[i],
						PositionsMessageType.LEADER,
						abstract_robot.getID());

				try {
					abstract_robot.unicast(ids[i], m);
				} catch (CommunicationException e) {
					e.printStackTrace();
				}
			}
		} else {
			Vec2[] va = targets0_global.Value(timestamp);
			// BRODCAST POSITION OF TARTGET0 TO OTHER TEAMMATES
			Message m = new SourcePositionMessage(
					va,
					PositionsMessageType.LEADER,
					abstract_robot.getID());

			abstract_robot.broadcast(m);
		}
	}

	private static Vec2[] positionAssignment(Vec2[] followers, Vec2[] targets, Vec2 from, double minimumRadius) {
	    double sumx = 0, sumy = 0;
	    for (Vec2 v: targets) {
	        sumx += v.x;
	        sumy += v.y;
        }

	    Vec2 c = new Vec2(sumx / targets.length, sumy / targets.length);

	    double r_sq = minimumRadius * minimumRadius;
	    for (Vec2 v: targets) {
	        double tmp = (v.x - c.x) * (v.x - c.x) + (v.y - c.y) *(v.y - c.y);
            r_sq = Math.max(r_sq, tmp);
        }

	    double r = Math.sqrt(r_sq);

		List<Vec2> tmpFollowers = new LinkedList<>();
		List<Vec2> tmpPositions = new LinkedList<>();

		for (int i = 0; i < followers.length; ++i) {
			Vec2 position = new Vec2(from.x, from.y);
			position.sub(c);

	        position.setr(r);
			position.rotate(Math.PI * 2 / (followers.length+1) * (i+1));
	        position.add(c);

			tmpPositions.add(position);
			tmpFollowers.add(followers[i]);
		}


		Map<Vec2, Vec2> followerAssignment = new HashMap<>();
		Map<Vec2, Vec2> positionAssignment = positionAssignment(tmpFollowers, tmpPositions);
		for(Map.Entry<Vec2, Vec2> entry: positionAssignment.entrySet()) {
			followerAssignment.put(entry.getValue(), entry.getKey());
		}

		Vec2[] ret = new Vec2[followers.length];
		for (int i = 0; i < ret.length; ++i) {
			ret[i] = followerAssignment.get(followers[i]);
		}

	    return ret;
    }

    private static Map<Vec2, Vec2> positionAssignment(List<Vec2> followers, List<Vec2> positions) {
        if (followers.size() == 0 && positions.size() == 0)
            return new HashMap<>();

        assert !followers.isEmpty();
        assert !positions.isEmpty();
        assert followers.size() == positions.size();

        Map<Vec2, Vec2> currentAssignment = new HashMap<>();

        List<Vec2> remain_followers = new LinkedList<>();

        for(Vec2 current_follower : followers) {
            double dist = Double.MAX_VALUE;
            Vec2 candidate_position = null;

            for(Vec2 position : positions) {
                // calculate distance
                position.sub(current_follower);
                // always select the closest position against the follower
                if (dist > position.r) {
                    dist = position.r;
                    candidate_position = position;
                }
                position.add(current_follower);
            }
            assert candidate_position != null;
            if (currentAssignment.containsKey(candidate_position)) { // position has been chosen
                Vec2 old_follower = currentAssignment.get(candidate_position);
                // calculate distance
				candidate_position.sub(old_follower);
                // always choose a follower having the longest distance against the position
                if (dist > candidate_position.r) {
                    currentAssignment.put(candidate_position, current_follower);
                    remain_followers.add(old_follower);     // put the old one into remain list
                } else {
                    remain_followers.add(current_follower);
                }
                candidate_position.add(old_follower);
            } else {
                currentAssignment.put(candidate_position, current_follower);
            }
        }

        List<Vec2> remain_positions = new LinkedList<>();
        for(Vec2 position: positions) {
            if (!currentAssignment.containsKey(position)) {
                remain_positions.add(position);
            }
        }

        Map<Vec2, Vec2> subAssignment = positionAssignment(remain_followers, remain_positions);
        currentAssignment.putAll(subAssignment);
        return currentAssignment;
    }

	public static void main(String[] args) {
	    System.out.println("Hello, world!");

		{
			List<Vec2> positions = new LinkedList<>();
			positions.add(new Vec2(-2,0));
			positions.add(new Vec2(0,2));
			positions.add(new Vec2(2, 0));

			List<Vec2> followers = new LinkedList<>();
			followers.add(new Vec2(0, 3));
			followers.add(new Vec2(0, 4));
			followers.add(new Vec2(0,5));

			Map<Vec2, Vec2> m = positionAssignment(followers, positions);

			System.out.println(1);
		}
		{
			Vec2[] followers = new Vec2[3];
			followers[0] = new Vec2(0,7);
			followers[1] = new Vec2(0,8);
			followers[2] = new Vec2(0,9);

			Vec2[] target = new Vec2[1];
			target[0] = new Vec2(0,0);

			Vec2 from = new Vec2(0,4);

			Vec2[] m = positionAssignment(followers, target, from, 2);

			System.out.println(2);
		}
		{
			Vec2[] followers = new Vec2[3];
			followers[0] = new Vec2(-9,0);
			followers[1] = new Vec2(-8,0);
			followers[2] = new Vec2(-7,0);

			Vec2[] target = new Vec2[1];
			target[0] = new Vec2(6,0);

			Vec2 from = new Vec2(1,0);

			Vec2[] m = positionAssignment(followers, target, from, 2);

			System.out.println(3);
		}
    }
	
}
