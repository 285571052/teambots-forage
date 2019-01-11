
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

//class Debug_b_Not_s extends b_Not_s {
//
//	/**
//	 * Instantiate a b_Not_s operator.
//	 *
//	 * @param im1 NodeScalar, the embedded perceptual schema that generates a value
//	 *            to be inverted.
//	 */
//	public Debug_b_Not_s(NodeScalar im1) {
//		super(im1);
//	}
//
//
//	public boolean Value(long timestamp) {
//		Boolean b = super.Value(timestamp);
//		return b;
//	}
//}

public class ForageBcastState extends ControlSystemMFN150 {
	public final static boolean DEBUG = true;
	private NodeVec2 turret_configuration;
	private NodeVec2 steering_configuration;
	private NodeVec2Array targets0_global;
	private i_FSA_ba STATE_MACHINE;


	private v_Cache_v 	cached_closest_global = null;
	private v_Cache_vb cached_closest_bcast_global = null;
	private v_Cache_v cached_history_global = new v_Cache_v(null);

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
		abstract_robot.setBaseSpeed(0.3 * abstract_robot.MAX_TRANSLATION);

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
		// 过滤剩下自己需要的
		NodeVec2Array PS_BCAST_GLOBAL = new va_FilterOutLastPositions(MSG_RECIEVE, PositionsMessageType.HISTORY);
		// 转换为EGO坐标
		NodeVec2Array PS_BCAST_EGO_FILT = new va_Subtract_vav(PS_BCAST_GLOBAL, PS_GLOBAL_POS);
		// 挑离自己最近的
		NodeVec2 PS_CLOSEST_BCAST = new v_Closest_va(PS_BCAST_EGO_FILT);
		// 转换为global坐标
		NodeVec2 PS_CLOSEST_BCAST_GLOBAL = new v_EgoToGlobal_rv(abstract_robot, PS_CLOSEST_BCAST);


		// 从广播中得到响应的其他机器人
		// NodeVec2Array PS_BCAST_TEAMMATES_GLOBAL = new va_FilterOutLastPositions(MSG_RECIEVE, PositionsMessageType.POS_TEAMMATES);
		

		// NodeVec2Array PS_TEAMMATES = new va_Teammates_r(abstract_robot);


		// ======
		// Perceptual Features
		// ======
		// 目标仍然在可视范围内
		NodeBoolean PF_TARGET_VISIBLE = new b_NonZero_v(PS_CLOSEST);
		// 目标逃离可视范围
//		NodeBoolean PF_NOT_TARGET_VISIBLE = new b_Not_s(PF_TARGET_VISIBLE);
		NodeBoolean PF_NOT_TARGET_VISIBLE = new b_Not_s(PF_TARGET_VISIBLE);
		// 收到广播消息
		NodeBoolean PF_BCAST_RECEIVED = new b_NonZero_v(PS_CLOSEST_BCAST);

		// 保存距离自己最近的广播GLOBAL目标
		cached_closest_bcast_global = new v_Cache_vb(PS_CLOSEST_BCAST_GLOBAL, PF_BCAST_RECEIVED);


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

		AS_INFORMED.weights[2] = 1;
		AS_INFORMED.embedded[2] = MS_SWIRL_OBSTACLES_TARGET0; // 旋转向目标

		AS_INFORMED.weights[3] = 0.3;
		AS_INFORMED.embedded[3] = MS_NOISE_VECTOR; // 随机移动

		// ======
		// AS_INERTIA
		// ======
		v_StaticWeightedSum_va AS_INERTIA = new v_StaticWeightedSum_va();

		AS_INERTIA.weights[0] = 0.6;
		AS_INERTIA.embedded[0] = MS_AVOID_OBSTACLES; // 避障

		AS_INERTIA.weights[1] = 2;
		AS_INERTIA.embedded[1] = MS_MOVE_TO_HISTORY; // 继续运动到原来的目标

		AS_INERTIA.weights[2] = 1;
		AS_INERTIA.embedded[2] = MS_SWIRL_OBSTACLES_TARGET0; // 旋转向目标

		AS_INERTIA.weights[3] = 0.2;
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
				abstract_robot.setDisplayString("acquire");

				cached_closest_global.storeValue(curr_time);
				cached_history_global.switchNode(cached_closest_global);
				cached_history_global.storeValue(curr_time);

				Vec2[] va = targets0_global.Value(curr_time);
				// BRODCAST POSITION OF TARTGET0 TO OTHER TEAMMATES
				Message m = new PositionsMessage(va, PositionsMessageType.HISTORY);
				abstract_robot.broadcast(m);

//				Vec2 v = cached_closest_global.Value(curr_time);
//				System.out.print(abstract_robot.getID() + " acquire target global(x:" + v.x + " y:" + v.y + ")\n");

				break;
			}
			case 2: {
				abstract_robot.setDisplayString("informed");

				cached_closest_bcast_global.storeValue(curr_time);
				cached_history_global.switchNode(cached_closest_bcast_global);
				cached_history_global.storeValue(curr_time);

//				Vec2 v = cached_closest_bcast_global.Value(curr_time);
//				System.out.print(abstract_robot.getID() + " informed bcast  global(x:" + v.x + " y:" + v.y + ")\n");

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

	public static void main(String[] args) {
	    System.out.println("Hello, world!");

    }
	
}
