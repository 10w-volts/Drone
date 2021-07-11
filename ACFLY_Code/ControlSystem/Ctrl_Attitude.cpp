#include "ctrl_Attitude.hpp"
#include "ControlSystem.hpp"
#include "ctrl_Main.hpp"
#include "Parameters.hpp"
#include "MeasurementSystem.hpp"
#include "TD4.hpp"
#include "TD3_3D.hpp"
#include "ESO_AngularRate.hpp"
#include "ESO_h.hpp"
#include "Filters_LP.hpp"
#include "drv_PWMOut.hpp"
#include "Receiver.hpp"

#include "StorageSystem.hpp"

/*参数*/
	//控制参数
	struct AttCtrlCfg
	{
		uint64_t UAVType;	//机型类型
		float STThrottle[2];	//起转油门
		float NonlinearFactor[2];	//电机非线性参数
		float FullThrRatio[2];	//满油门比例
		float T[2];	//惯性时间T
		float b[6];	//RPY增益b
		float TD4_P1[6];	//RPY前馈TD4滤波器P1
		float TD4_P2[6];	//RPY前馈TD4滤波器P2
		float TD4_P3[6];	//RPY前馈TD4滤波器P3
		float TD4_P4[6];	//RPY前馈TD4滤波器P4
		float P1[6];	//反馈增益P1
		float P2[6];	//反馈增益P2
		float P3[6];	//反馈增益P3
		float P4[6];	//反馈增益P4
		float beta[2];	//ESO beta
		float maxLean[2];	//最大倾斜角
		float maxRPSp[2];	//最大Pitch Roll速度
		float maxRPAcc[2]; //最大Pitch Roll加速度
		float maxYSp[2];	//最大偏航速度
		float maxYAcc[2];	//最大偏航加速度
	}__PACKED;

	//参数
	static AttCtrlCfg cfg;
/*参数*/
	
/*内部接口*/
	float get_STThrottle()
	{
		return cfg.STThrottle[0];
	}
	float get_maxLean()
	{
		return cfg.maxLean[0];
	}
	float get_maxYawSpeed()
	{
		return cfg.maxYSp[0];
	}
/*内部接口*/
	
/*起飞地点*/
	static bool HomeLatLonAvailable;
	static bool HomeAvailable;
	static vector2<double> HomeLatLon;
	static vector2<double> HomePoint;
	static double HomeLocalZ = 0;
	bool getHomeLocalZ( double* home, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*home = HomeLocalZ;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool getHomePoint( vector2<double>* home, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			bool available = HomeAvailable;
			if(available)
				*home = HomePoint;
			UnlockCtrl();
			if( available )
				return true;
			else
				return false;
		}
		return false;
	}
	bool getHomeLatLon( vector2<double>* home, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			bool available = HomeLatLonAvailable;
			if(available)
				*home = HomeLatLon;
			UnlockCtrl();
			if( available )
				return true;
			else
				return false;
		}
		return false;
	}
/*起飞地点*/
	
/*状态观测器*/
	//姿态ESO
	static ESO_AngularRate ESO[3];
	//高度ESO
	static double throttle_u = 0;
	static ESO_h ESO_height;
	static double hover_throttle = 0;
	static double WindDisturbance_x = 0;
	static double WindDisturbance_y = 0;
	static bool inFlight = false;
	static Filter_Butter4_LP AccZ_filter;
	static Filter_LP_IIR_1 WindDisturbance_filter[2];
	//侧翻检测
	static uint32_t crash_counter = 0;
	static double AC_angle_error = 0;
	static bool crashed = false;
	static inline void update_output_throttle( double throttle, double h )
	{
		Quaternion quat;
		get_Airframe_quat( &quat, 0.1 );
		double output_minimum_throttle = cfg.STThrottle[0];
		double lean_cosin = quat.get_lean_angle_cosin();
		
		//加速度滤波
		vector3<double> AccENU;
		get_AccelerationENU_Ctrl(&AccENU);
		double AccZ = AccZ_filter.run(AccENU.z);
		
		//观测悬停油门
		double r_throttle = throttle - output_minimum_throttle;
		if( r_throttle < 0 )
			r_throttle = 0;
		if( lean_cosin < 0.1f )
				lean_cosin = 0.1f;
		r_throttle *= lean_cosin;		
		throttle_u = r_throttle;
		hover_throttle = ESO_height.get_hover_throttle() + output_minimum_throttle;
		
		//更新飞行状态
		static uint16_t onGround_counter = 0;
		if( inFlight == false )
		{
			onGround_counter = 0;
			if( AccZ > 20 && throttle > output_minimum_throttle + 3 )
				inFlight = true;
		}
		else
		{
			if( ( (hover_throttle<output_minimum_throttle+2) ) && lean_cosin>0 && fabs(AccZ)<15 )
			{
				if( ++onGround_counter >= 1.0 * CtrlRateHz )
					inFlight = false;
			}
			else
				onGround_counter = 0;
		}
		
		//侧翻保护
		if( AC_angle_error>degree2rad(30.0) && fabs(AccZ)<25 && throttle>output_minimum_throttle-0.1 )
		{
			if( ++crash_counter >= CtrlRateHz*2 )
			{
				crash_counter = CtrlRateHz*2;
				crashed = true;
			}
		}
		else
		{
			crashed = false;
			crash_counter = 0;
		}
		
		//观测水平分力
		if( inFlight )
		{
			Quaternion quat;
			get_AirframeY_quat( &quat );
			vector3<double> active_force_xy_vec = quat.rotate_axis_z();
			if( lean_cosin < 0.3f )
				lean_cosin = 0.3f;
			active_force_xy_vec = active_force_xy_vec *( ( AccENU.z + GravityAcc ) / lean_cosin );
			vector3<double> WindDisturbance_xy;
			WindDisturbance_xy.x = AccENU.x - active_force_xy_vec.x;
			WindDisturbance_xy.y = AccENU.y - active_force_xy_vec.y;
			
			WindDisturbance_x = WindDisturbance_filter[0].run(WindDisturbance_xy.x);
			WindDisturbance_y = WindDisturbance_filter[1].run(WindDisturbance_xy.y);
//			double lp_factor = 2 * Pi * (1.0/CtrlRateHz) * 1.0;
//			WindDisturbance_x += lp_factor * ( WindDisturbance_xy.x - WindDisturbance_x );
//			WindDisturbance_y += lp_factor * ( WindDisturbance_xy.y - WindDisturbance_y );
		}
		else
		{
			WindDisturbance_filter[0].reset(0);
			WindDisturbance_filter[1].reset(0);
			WindDisturbance_x = WindDisturbance_y = 0;
		}
		
		//更新Home点位置
		if( inFlight == false )
		{
			vector3<double> position;
			get_Position_Ctrl(&position);
			HomeLocalZ = position.z;
			
			PosSensorHealthInf2 posInf;
			if( get_Health_XY(&posInf) )
			{
				HomeAvailable = true;
				HomePoint.x = posInf.PositionENU.x;
				HomePoint.y = posInf.PositionENU.y;
			}
			else
				HomeAvailable = false;
			
			if( get_OptimalGlobal_XY(&posInf) )
			{
				HomeLatLonAvailable = true;
				map_projection_reproject( &posInf.mp, 
					posInf.PositionENU.x+posInf.HOffset.x, 
					posInf.PositionENU.y+posInf.HOffset.y,
					&HomeLatLon.x, &HomeLatLon.y );
			}			
		}
		else if( get_Position_MSStatus()!= MS_Ready )
			HomeAvailable = false;
	}
	
	static double Roll_u = 0;
	static double Pitch_u = 0;
	static double Yaw_u = 0;
	void update_ESO_1()
	{
		//更新角速度观测器
		vector3<double> angular_rate;
		get_AngularRate_Ctrl(&angular_rate);
		ESO[0].run(angular_rate.x);
		ESO[1].run(angular_rate.y);
		ESO[2].run(angular_rate.z);
		
		vector3<double> acc;
		get_AccelerationENU_Ctrl(&acc);
		ESO_height.run( acc.z );
	}	
	void update_ESO_2()
	{
		ESO[0].update_u(Roll_u);
		ESO[1].update_u(Pitch_u);
		ESO[2].update_u(Yaw_u);
		ESO_height.update_u(throttle_u);
	}
/*状态观测器*/	

/*观测器接口*/
	bool get_hover_throttle( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = hover_throttle;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_throttle_force( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = ESO_height.get_force();
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_throttle_b( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = ESO_height.get_b();
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_ESO_height_T( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = ESO_height.get_T();
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool get_is_inFlight( bool* result, double TIMEOUT )
	{
		*result = inFlight;
		return true;
	}
	bool get_WindDisturbance( vector3<double>* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = vector3<double>( WindDisturbance_x, WindDisturbance_y, 0 );;
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	
	bool get_EsAngularRate( vector3<double>* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = vector3<double>( ESO[0].get_EsAngularRate(), ESO[1].get_EsAngularRate(), ESO[2].get_EsAngularRate() );
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	bool get_EsAngularAcc( vector3<double>* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = vector3<double>( ESO[0].get_EsAngularAcceleration(), ESO[1].get_EsAngularAcceleration(), ESO[2].get_EsAngularAcceleration() );
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	bool get_CrashedState(){ return crashed; }
/*观测器接口*/
	
/*控制接口*/
	//保护方式
	static uint8_t SafeBt = 0;
	
	//期望TD4滤波器
	static TD3_2DSL Target_tracker_RP;
	static TD4_SL Target_trackerY;
	
	//姿态控制模式
	static bool Attitude_Control_Enabled = false;
	static Attitude_ControlMode RollPitch_ControlMode = Attitude_ControlMode_Angle;
	static Attitude_ControlMode Yaw_ControlMode = Attitude_ControlMode_Angle;

	//输出滤波器
	static double outRoll_filted = 0;
	static double outPitch_filted = 0;
	static double outYaw_filted = 0;
	
	static double throttle = 0;
	static double target_Roll;
	static double target_Pitch;
	static double target_Yaw;
	static vector3<double> target_AngularRate;
	
	bool is_Attitude_Control_Enabled( bool* enabled, double TIMEOUT )
	{
		*enabled = Attitude_Control_Enabled;
		return true;
	}
	bool Attitude_Control_Enable( double TIMEOUT )
	{
		if( get_Attitude_MSStatus() != MS_Ready )
			return false;
		
		Quaternion quat;
		if( get_Airframe_quat( &quat, TIMEOUT ) == false )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == true )
			{	//控制器已打开
				UnlockCtrl();
				return false;
			}
			
			//读参数
			if( ReadParamGroup( "AttCtrl", (uint64_t*)&cfg, 0, TIMEOUT ) != PR_OK )
			{
				UnlockCtrl();
				return false;
			}
			uint8_t safe_bt[8];	
			if( ReadParam("MFunc_SafeBt", 0, 0, (uint64_t*)safe_bt, 0 ) != PR_OK )
				SafeBt = 0;
			else
				SafeBt = safe_bt[0];
			
			/*初始化*/				
				//读取电池电压
				float BatVoltage = get_MainBatteryVoltage_filted();
				//读取基准电压
				float STVoltage[2] = {0};
				ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 );
				//计算增益修正系数
				double b_scale = 1.0;
				if( STVoltage[0]>7 && BatVoltage>7 )
						b_scale = BatVoltage / STVoltage[0];
			
				//初始化姿态ESO
				ESO[0].init( cfg.T[0], cfg.b[0]*b_scale, cfg.beta[0], cfg.beta[0], CtrlRateHz*CtrlRateDiv );
				ESO[1].init( cfg.T[0], cfg.b[2]*b_scale, cfg.beta[0], cfg.beta[0], CtrlRateHz*CtrlRateDiv );
				ESO[2].init( 1.0/(CtrlRateHz*CtrlRateDiv), cfg.b[4]*b_scale, cfg.beta[0], cfg.beta[0], CtrlRateHz*CtrlRateDiv );
				
				//初始化高度ESO
				ESO_height.init( cfg.T[0], 4.5, CtrlRateHz*CtrlRateDiv );
			
				//初始化期望TD4滤波器
				Target_tracker_RP.P1=cfg.TD4_P1[0];
				Target_tracker_RP.P2=cfg.TD4_P2[0];
				Target_tracker_RP.P3=cfg.TD4_P3[0];
				Target_tracker_RP.r2=degree2rad(cfg.maxRPSp[0]);
				Target_tracker_RP.r3=degree2rad(cfg.maxRPAcc[0]);
				Target_tracker_RP.r4=degree2rad(100000.0);		
				
				Target_trackerY.P1=cfg.TD4_P1[4];
				Target_trackerY.P2=cfg.TD4_P2[4];
				Target_trackerY.P3=cfg.TD4_P3[4];
				Target_trackerY.P4=cfg.TD4_P4[4];
				Target_trackerY.r2n=Target_trackerY.r2p=degree2rad(cfg.maxYSp[0]);
				Target_trackerY.r3n=Target_trackerY.r3p=degree2rad(cfg.maxYAcc[0]);
			/*初始化*/
			
			//顺序起转电机
			double pwm_out[8] = {0};
			uint8_t main_motors = UAV_MainMotorCount(cfg.UAVType);
			if( main_motors == 0 )
			{
				UnlockCtrl();
				return false;
			}
			set_MainMotorCount(main_motors);
			for( uint8_t i = 0; i < main_motors; ++i )
			{
				pwm_out[i] = cfg.STThrottle[0];						
				MainMotor_PWM_Out( pwm_out );
				os_delay(0.3);
			}
			
			Attitude_Control_Enabled = true;
			target_Yaw = quat.getYaw();
			target_Roll = target_Pitch = 0;
			RollPitch_ControlMode = Attitude_ControlMode_Angle;
			Yaw_ControlMode = Attitude_ControlMode_Angle;
			
			//更新控制时间
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if(!isMSafe)
				last_ZCtrlTime = last_XYCtrlTime = TIME::now();
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_Disable( double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			Altitude_Control_Disable();
			Position_Control_Disable();
			Attitude_Control_Enabled = false;			
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	bool get_Target_Throttle( double* result, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			*result = throttle;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Throttle( double thr, double TIMEOUT )
	{
		if( isnan(thr) || isinf(thr) )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			bool alt_enabled;
			is_Altitude_Control_Enabled(&alt_enabled);
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if( !isMSafe && alt_enabled==false && ForceMSafeCtrl )
			{	//屏蔽用户控制
				last_ZCtrlTime = TIME::now();
				UnlockCtrl();
				return false;
			}
			
			throttle = thr;
			
			//更新控制时间			
			if(!isMSafe && alt_enabled==false)
				last_ZCtrlTime = TIME::now();
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	bool Attitude_Control_get_Target_RollPitch( double* Roll, double* Pitch, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			
			*Roll = target_Roll;
			*Pitch = target_Pitch;
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_RollPitch( double Roll, double Pitch, double TIMEOUT )
	{
		if( isnan(Roll) || isinf(Roll) || isnan(Pitch) || isinf(Pitch) )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			bool pos_enabled;
			is_Position_Control_Enabled(&pos_enabled);
			bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
			if( !isMSafe && pos_enabled==false && ForceMSafeCtrl )
			{	//屏蔽用户控制
				last_ZCtrlTime = TIME::now();
				UnlockCtrl();
				return false;
			}
			
			double angle = safe_sqrt( Roll*Roll + Pitch*Pitch );
			if( angle > degree2rad(cfg.maxLean[0]) )
			{
				double scale = degree2rad(cfg.maxLean[0]) / angle;
				Roll *= scale;
				Pitch *= scale;
			}		
			target_Roll = Roll;
			target_Pitch = Pitch;
			RollPitch_ControlMode = Attitude_ControlMode_Angle;
			
			//更新控制时间
			if(!isMSafe && pos_enabled==false)
				last_XYCtrlTime = TIME::now();
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	
	bool Attitude_Control_get_TargetYaw( double* TargetYaw, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			*TargetYaw = target_Yaw;
			
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	bool Attitude_Control_get_TargetTrackYaw( double* TargetYaw, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			*TargetYaw = Target_trackerY.x1;
			
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	bool Attitude_Control_get_YawTrackErr( double* YawErr, double TIMEOUT )
	{
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			*YawErr = fabs(Target_trackerY.x1 - target_Yaw);
			
			UnlockCtrl();
			return true;
		}
		return false;		
	}
	bool Attitude_Control_set_Target_Yaw( double Yaw, double TIMEOUT )
	{		
		if( isnan(Yaw) || isinf(Yaw) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		
		Quaternion quat, quatY;
		get_Airframe_quat(&quat);
		get_AirframeY_quat(&quatY);
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			if( Yaw_ControlMode != Attitude_ControlMode_Angle )
			{
				Target_trackerY.x1 = quat.getYaw();
				Yaw_ControlMode = Attitude_ControlMode_Angle;
			}
			double yaw_err = Mod( Yaw - quatY.getYaw(), 2*Pi );
			if(yaw_err > Pi)
				yaw_err -= 2*Pi;
			while(yaw_err < -Pi)
				yaw_err += 2*Pi;
			target_Yaw = Target_trackerY.x1 + yaw_err;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_YawRelative( double Yaw, double TIMEOUT )
	{		
		if( isnan(Yaw) || isinf(Yaw) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		
		Quaternion quat;
		get_Airframe_quat(&quat);
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			double currentYaw = quat.getYaw();
			if( Yaw_ControlMode != Attitude_ControlMode_Angle )
			{
				Target_trackerY.x1 = currentYaw;
				Yaw_ControlMode = Attitude_ControlMode_Angle;
			}
			target_Yaw = Target_trackerY.x1 + Yaw;
		
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_Target_YawRate( double YawRate, double TIMEOUT )
	{
		if( isnan(YawRate) || isinf(YawRate) )
			return false;
		
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			target_AngularRate.z = YawRate;
			Yaw_ControlMode = Attitude_ControlMode_AngularRate;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
	bool Attitude_Control_set_YawLock( double TIMEOUT )
	{
		//屏蔽用户控制
		bool isMSafe = (xTaskGetCurrentTaskHandle()==MSafeTaskHandle);
		if( !isMSafe && ForceMSafeCtrl )
			return false;
		if( LockCtrl(TIMEOUT) )
		{
			if( Attitude_Control_Enabled == false )
			{
				UnlockCtrl();
				return false;
			}
			if( Yaw_ControlMode == Attitude_ControlMode_AngularRate )
				Yaw_ControlMode = Attitude_ControlMode_Locking;
			
			UnlockCtrl();
			return true;
		}
		return false;
	}
/*控制接口*/

static void ctrl_Attitude_MultiRotor( uint8_t mt_count, double outRoll, double outPitch, double outYaw, double rp_out[], double yaw_out[] );
	
void ctrl_Attitude()
{	
	double h = 1.0 / CtrlRateHz;
	
	if( Attitude_Control_Enabled == false )
	{	
		Roll_u = Pitch_u = Yaw_u = 0;
		update_output_throttle( 0 , h );
		MainMotor_PullDownAll();
		return;
	}	
	
	Receiver rc;
	getReceiver(&rc);
	
	bool DbgSafe = false;
	if( SafeBt )
	{	//Debug保护
		if( SafeBt==10 )
		{
			if( (rc.available && rc.data[0] < cfg.STThrottle[0] - 0.1) && (rc.data[1] < 10) )
				DbgSafe = true;
		}
		else if( SafeBt>=2 && SafeBt<=4 )
		{
			if( rc.available && rc.available_channels>=4+SafeBt && rc.data[4+SafeBt-1]>90 )
				DbgSafe = true;
		}
	}
	
	//油门低于起转油门拉低所有输出	
	if( throttle < cfg.STThrottle[0] - 0.1 || DbgSafe )
	{
		Roll_u = Pitch_u = Yaw_u = 0;
		update_output_throttle( 0 , h );
		MainMotor_PullDownAll();
		return;
	}
	
	
//	//根据电池电压调整控制对象增益
//	float BatV = getBatteryVoltage();
//	float VST = Cfg_get_BatSTVoltage();
//	if( BatV > 7 && VST > 7 )
//	{
//		float scale = BatV / VST;
//		ESO[0].b = Cfg_get_RPYCtrl_b(0) * scale;
//		ESO[1].b = Cfg_get_RPYCtrl_b(1) * scale;
//		ESO[2].b = Cfg_get_RPYCtrl_b(2) * scale;
//	}
//	else
//	{
//		ESO[0].b = Cfg_get_RPYCtrl_b(0);
//		ESO[1].b = Cfg_get_RPYCtrl_b(1);
//		ESO[2].b = Cfg_get_RPYCtrl_b(2);
//	}
	
	//读取电池电压
	float BatVoltage = get_MainBatteryVoltage_filted();
	//计算增益修正系数
	float STVoltage[2] = {0};
	ReadParam("Bat_STVoltage", 0, 0, (uint64_t*)STVoltage, 0 );
	//计算增益修正系数
	double b_scale = 1.0;
	if( STVoltage[0]>7 && BatVoltage>7 )
		b_scale = BatVoltage / STVoltage[0];
	
	//根据电池电压调整控制对象增益
	ESO[0].b = cfg.b[0] * b_scale;
	ESO[1].b = cfg.b[2] * b_scale;
	ESO[2].b = cfg.b[4] * b_scale;
	
	Quaternion AirframeQuat;
	get_Airframe_quat( &AirframeQuat, 0.1 );
	
	//获取控制参数
	double Ps = cfg.P1[0];
	double PsY = cfg.P1[4];
	vector3<double> P2( cfg.P2[0], cfg.P2[2], cfg.P2[4] );
	vector3<double> P3( cfg.P3[0], cfg.P3[2], cfg.P3[4] );
	
	//目标Roll Pitch四元数
	Quaternion target_quat_PR;			
	//目标角速度
	vector3<double> target_angular_velocity;

	//获取当前四元数的Pitch Roll分量四元数
	double Yaw = AirframeQuat.getYaw();
	double half_sinYaw, half_cosYaw;
	fast_sin_cos( 0.5*Yaw, &half_sinYaw, &half_cosYaw );
	Quaternion YawQuat(
		half_cosYaw ,
		0 ,
		0 ,
		half_sinYaw
	);
	YawQuat.conjugate();
	Quaternion_Ef current_quat_PR = Quaternion_Ef( YawQuat*AirframeQuat );
	
	//计算旋转矩阵
	current_quat_PR.conjugate();				
	double Rotation_Matrix[3][3];	//反向旋转
	current_quat_PR.get_rotation_matrix(Rotation_Matrix);
	current_quat_PR.conjugate();	
	double Rotation_Matrix_P[3][3]; //正向旋转
	current_quat_PR.get_rotation_matrix(Rotation_Matrix_P);
	
	//运行扩张状态观测器得到估计角速度、角加速度
	
	vector3<double> AngularRateCtrl;
	get_AngularRate_Ctrl( &AngularRateCtrl, 0.1 );
	vector3<double> angular_rate_ESO;
	vector3<double> angular_acceleration_ESO;
	//使用ESO估计角速度、角加速度
	angular_rate_ESO.set_vector(
		ESO[0].get_EsAngularRate() ,
		ESO[1].get_EsAngularRate() ,
		ESO[2].get_EsAngularRate()
	);		
	angular_acceleration_ESO.set_vector(
		ESO[0].get_EsAngularAcceleration() ,
		ESO[1].get_EsAngularAcceleration() ,
		ESO[2].get_EsAngularAcceleration()
	);
	
	//计算ENU坐标系下的角速度、角加速度
	vector3<double> angular_rate_ENU;
	angular_rate_ENU.x = Rotation_Matrix_P[0][0]*angular_rate_ESO.x + Rotation_Matrix_P[0][1]*angular_rate_ESO.y + Rotation_Matrix_P[0][2]*angular_rate_ESO.z;
	angular_rate_ENU.y = Rotation_Matrix_P[1][0]*angular_rate_ESO.x + Rotation_Matrix_P[1][1]*angular_rate_ESO.y + Rotation_Matrix_P[1][2]*angular_rate_ESO.z;
	angular_rate_ENU.z = Rotation_Matrix_P[2][0]*angular_rate_ESO.x + Rotation_Matrix_P[2][1]*angular_rate_ESO.y + Rotation_Matrix_P[2][2]*angular_rate_ESO.z;
	vector3<double> angular_acceleration_ENU;
	angular_acceleration_ENU.x = Rotation_Matrix_P[0][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[0][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[0][2]*angular_acceleration_ESO.z;
	angular_acceleration_ENU.y = Rotation_Matrix_P[1][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[1][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[1][2]*angular_acceleration_ESO.z;
	angular_acceleration_ENU.z = Rotation_Matrix_P[2][0]*angular_acceleration_ESO.x + Rotation_Matrix_P[2][1]*angular_acceleration_ESO.y + Rotation_Matrix_P[2][2]*angular_acceleration_ESO.z;
	
	//由Roll Pitch控制模式
	//计算Roll Pitch目标角速度（ENU系）
	vector3<double> target_angular_rate_RP;
	switch( RollPitch_ControlMode )
	{
		default:
		case Attitude_ControlMode_Angle:
		{
			//TD4滤目标角度
			Target_tracker_RP.track3( vector2<double>(target_Roll,target_Pitch), 1.0 / CtrlRateHz );
			
			//使用目标角度构造目标四元数
			//calculate target quat Q1
			//      front
			//       x
			//       ^
			//       |
			// y < --O
			double half_sinR, half_cosR;
			fast_sin_cos( 0.5*Target_tracker_RP.x1.x, &half_sinR, &half_cosR );
			double half_sinP, half_cosP;
			fast_sin_cos( 0.5*Target_tracker_RP.x1.y, &half_sinP, &half_cosP );
			target_quat_PR = Quaternion( 
				half_cosR*half_cosP ,
				half_cosP*half_sinR ,
				half_cosR*half_sinP ,
				-half_sinR*half_sinP
			);
			
			//计算误差四元数Q
			//Q*Q1=Qt  Q1为当前机体四元数，Qt为目标四元数
			//Q=Qt*inv(Q1)
			Quaternion current_quat_conj = current_quat_PR;	current_quat_conj.conjugate();
			vector3<double> PR_rotation = ( target_quat_PR * current_quat_conj ).get_Rotation_vec();
			vector3<double> feed_foward_ratePR = { Target_tracker_RP.x2.x, Target_tracker_RP.x2.y , 0 };
			target_angular_rate_RP = ( PR_rotation * Ps ) + feed_foward_ratePR;
			
			AC_angle_error = safe_sqrt( PR_rotation.get_square() );
			break;
		}
	}
	
	double target_angular_rate_Y;
	switch(Yaw_ControlMode)
	{
		case Attitude_ControlMode_Angle:
		{
			if(inFlight)
			{
				//TD4滤目标角度
				Target_trackerY.r2n = Target_trackerY.r2p = degree2rad(cfg.maxYSp[0]);
				Target_trackerY.track4( target_Yaw , 1.0f / CtrlRateHz );
				
				//角度误差化为-180 - +180
				double angle_error = Target_trackerY.x1 - Yaw;
				while( angle_error < -Pi )
					angle_error+=2*Pi;
				while( angle_error > Pi )
					angle_error-=2*Pi;

				//求目标角速度
				target_angular_rate_Y = angle_error * Ps + Target_trackerY.x2;
				target_angular_rate_Y = constrain( target_angular_rate_Y , 2.5 );
			}
			else
			{				
				Target_trackerY.reset();
				Target_trackerY.x1 = target_Yaw = Yaw;
				target_angular_rate_Y = 0;
			}
			break;
		}
		case Attitude_ControlMode_AngularRate:
		{
			if(inFlight)
			{
				Target_trackerY.r2n=Target_trackerY.r2p=degree2rad(cfg.maxYSp[0]);
				Target_trackerY.track3( target_AngularRate.z , 1.0 / CtrlRateHz );
				target_angular_rate_Y = Target_trackerY.x2;
			}
			else
			{				
				Target_trackerY.reset();
				Target_trackerY.x1 = target_Yaw = Yaw;
				target_angular_rate_Y = 0;
			}
			break;
		}
		case Attitude_ControlMode_Locking:
		{
			if(inFlight)
			{
				Target_trackerY.track3( 0 , 1.0 / CtrlRateHz );
				target_angular_rate_Y = Target_trackerY.x2;
				if( in_symmetry_range( target_angular_rate_Y , 0.001 ) && in_symmetry_range( angular_rate_ENU.z , 0.05 ) )
				{							
					Target_trackerY.x1 = target_Yaw = Yaw;
					Yaw_ControlMode = Attitude_ControlMode_Angle;
				}
			}
			else
			{			
				Target_trackerY.reset();
				Target_trackerY.x1 = target_Yaw = Yaw;
				target_angular_rate_Y = 0;
			}
			break;
		}
	}
	
	//计算前馈量
		double YawAngleP =  ( Target_trackerY.get_tracking_mode() == 4 ) ? ( Ps ) : 0;
		vector3<double> Tv1_ENU = { Ps*( Target_tracker_RP.x2.x - angular_rate_ENU.x ) + Target_tracker_RP.x3.x ,
															Ps*( Target_tracker_RP.x2.y - angular_rate_ENU.y ) + Target_tracker_RP.x3.y ,
															YawAngleP*( Target_trackerY.x2 - angular_rate_ENU.z ) + Target_trackerY.x3 };
		vector3<double> Tv2_ENU = { Ps*( Target_tracker_RP.x3.x - angular_acceleration_ENU.x ) + Target_tracker_RP.T4.x ,
															Ps*( Target_tracker_RP.x3.y - angular_acceleration_ENU.y ) + Target_tracker_RP.T4.y,
															YawAngleP*( Target_trackerY.x3 - angular_acceleration_ENU.z ) + Target_trackerY.x4 };
		
		vector3<double> Tv1;
		Tv1.x = Rotation_Matrix[0][0]*Tv1_ENU.x + Rotation_Matrix[0][1]*Tv1_ENU.y + Rotation_Matrix[0][2]*Tv1_ENU.z;
		Tv1.y = Rotation_Matrix[1][0]*Tv1_ENU.x + Rotation_Matrix[1][1]*Tv1_ENU.y + Rotation_Matrix[1][2]*Tv1_ENU.z;
		Tv1.z = Rotation_Matrix[2][0]*Tv1_ENU.x + Rotation_Matrix[2][1]*Tv1_ENU.y + Rotation_Matrix[2][2]*Tv1_ENU.z;
		vector3<double> Tv2;
		Tv2.x = Rotation_Matrix[0][0]*Tv2_ENU.x + Rotation_Matrix[0][1]*Tv2_ENU.y + Rotation_Matrix[0][2]*Tv2_ENU.z;
		Tv2.y = Rotation_Matrix[1][0]*Tv2_ENU.x + Rotation_Matrix[1][1]*Tv2_ENU.y + Rotation_Matrix[1][2]*Tv2_ENU.z;
		Tv2.z = Rotation_Matrix[2][0]*Tv2_ENU.x + Rotation_Matrix[2][1]*Tv2_ENU.y + Rotation_Matrix[2][2]*Tv2_ENU.z;
		vector3<double> Ta1 = { P2.x*( Tv1.x - angular_acceleration_ESO.x ) + Tv2.x ,
														P2.y*( Tv1.y - angular_acceleration_ESO.y ) + Tv2.y ,
														P2.z*( Tv1.z - angular_acceleration_ESO.z ) + Tv2.z };
	//计算前馈量
													
	//把目标速度从Bodyheading旋转到机体
		vector3<double> target_angular_rate_ENU;
		target_angular_rate_ENU.x = target_angular_rate_RP.x;
		target_angular_rate_ENU.y = target_angular_rate_RP.y;
		target_angular_rate_ENU.z = target_angular_rate_RP.z + target_angular_rate_Y;

		vector3<double> target_angular_rate_body;
		target_angular_rate_body.x = Rotation_Matrix[0][0]*target_angular_rate_ENU.x + Rotation_Matrix[0][1]*target_angular_rate_ENU.y + Rotation_Matrix[0][2]*target_angular_rate_ENU.z;
		target_angular_rate_body.y = Rotation_Matrix[1][0]*target_angular_rate_ENU.x + Rotation_Matrix[1][1]*target_angular_rate_ENU.y + Rotation_Matrix[1][2]*target_angular_rate_ENU.z;
		target_angular_rate_body.z = Rotation_Matrix[2][0]*target_angular_rate_ENU.x + Rotation_Matrix[2][1]*target_angular_rate_ENU.y + Rotation_Matrix[2][2]*target_angular_rate_ENU.z;
	//把目标速度从Bodyheading旋转到机体
													
	//计算目标角加速度
		vector3<double> target_angular_acceleration = target_angular_rate_body - angular_rate_ESO;
		target_angular_acceleration.x *= P2.x;
		target_angular_acceleration.y *= P2.y;
		target_angular_acceleration.z *= P2.z;
		target_angular_acceleration = target_angular_acceleration + Tv1;
	//计算目标角加速度
													
	//计算角加速度误差
	vector3<double> angular_acceleration_error = target_angular_acceleration - angular_acceleration_ESO;
	
	vector3<double> disturbance(
		ESO[0].get_EsDisturbance() ,
		ESO[1].get_EsDisturbance() ,
		ESO[2].get_EsDisturbance()
	);
	static vector3<double> last_disturbance = { 0 , 0 , 0 };		
	vector3<double> disturbance_Derivative = (disturbance - last_disturbance) * CtrlRateHz;
	last_disturbance = disturbance;

	double outRoll;double outPitch;double outYaw;
	if( inFlight )
	{
		outRoll = 	( ESO[0].get_EsMainPower() + ESO[0].T * ( angular_acceleration_error.x * P3.x + Ta1.x /*- disturbance_x*/ ) )/ESO[0].b;
		outPitch =	( ESO[1].get_EsMainPower() + ESO[1].T * ( angular_acceleration_error.y * P3.y + Ta1.y /*- disturbance_y*/ ) )/ESO[1].b;
//		outYaw =		( ESO_AngularRate_get_EsMainPower( &ESO[2] ) + ESO[2].T * ( angular_acceleration_error.z * P.z + Ta1.z /*- disturbance_z*/ ) )/ESO[2].b;
		outYaw = ( target_angular_acceleration.z - disturbance.z ) / ESO[2].b;
	}
	else
	{
		outRoll = 	ESO[0].T * ( angular_acceleration_error.x * P3.x )/ESO[0].b;
		outPitch =	ESO[1].T * ( angular_acceleration_error.y * P3.y )/ESO[1].b;
		//outYaw =		ESO[2].T * ( angular_acceleration_error.z * P.z )/ESO[2].b;
		outYaw = ( target_angular_acceleration.z ) / ESO[2].b;
	}
	
//	outRoll_filted += 80 * h * ( outRoll - outRoll_filted );
//	outPitch_filted += 80 * h * ( outPitch - outPitch_filted );
//	outYaw_filted += 80 * h * ( outYaw - outYaw_filted );
	
	if( inFlight )
	{
		double logbuf[10];
		logbuf[0] = target_angular_rate_body.x;
		logbuf[1] = AngularRateCtrl.x;
		logbuf[2] = angular_rate_ESO.x;
		logbuf[3] = target_angular_acceleration.x;
		logbuf[4] = angular_acceleration_ESO.x;
		logbuf[5] = ESO[0].get_EsMainPower();
		logbuf[6] = outRoll;
		logbuf[7] = disturbance.x;
		logbuf[8] = ESO[0].u;
		SDLog_Msg_DebugVect( "att", logbuf, 9 );
	}

	switch( cfg.UAVType )
	{
		case UAVType_Rotor4_X:
		{
			double rp_out[4];			
			rp_out[0] = -outPitch+outRoll;
			rp_out[1] = +outPitch+outRoll;		
			rp_out[2] = +outPitch-outRoll;
			rp_out[3] = -outPitch-outRoll;
			double yaw_out[4];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			ctrl_Attitude_MultiRotor( 4, outRoll , outPitch , outYaw, rp_out, yaw_out );
			break;		
		}
		
		case UAVType_Rotor6_X:
		{			
			double rp_out[6];
			double RollS = outRoll * 1.1547005383792515290182975610039f;
			double half_outRoll = 0.5f * RollS;			
			rp_out[0] = -outPitch+half_outRoll;
			rp_out[1] = RollS;
			rp_out[2] = +outPitch+half_outRoll;
			rp_out[3] = +outPitch-half_outRoll;
			rp_out[4] = -RollS;
			rp_out[5] = -outPitch-half_outRoll;
			double yaw_out[6];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			yaw_out[4] = -outYaw;
			yaw_out[5] = +outYaw;
			ctrl_Attitude_MultiRotor( 6, outRoll , outPitch , outYaw, rp_out, yaw_out );
			break;		
		}
		
		case UAVType_Rotor8_X:
		{			
			double rp_out[8];		
			rp_out[0] = -outPitch+outRoll;
			rp_out[1] = -outPitch+outRoll;
			rp_out[2] = +outPitch+outRoll;
			rp_out[3] = +outPitch+outRoll;
			rp_out[4] = +outPitch-outRoll;
			rp_out[5] = +outPitch-outRoll;
			rp_out[6] = -outPitch-outRoll;
			rp_out[7] = -outPitch-outRoll;
			double yaw_out[8];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			yaw_out[4] = -outYaw;
			yaw_out[5] = +outYaw;
			yaw_out[6] = -outYaw;
			yaw_out[7] = +outYaw;
			ctrl_Attitude_MultiRotor( 8, outRoll , outPitch , outYaw, rp_out, yaw_out );
			break;		
		}
//		
//		case UAVType_Rotor4_C:
//			ctrl_Attitude_MultiRotor_C4_PWM( outRoll , outPitch , outYaw );
//			break;
//		
//		case UAVType_Rotor42_C:
//			ctrl_Attitude_MultiRotor_C42_PWM( outRoll , outPitch , outYaw );
//			break;
		case UAVType_Rotor6_S1:
		{			
			double rp_out[6];			
			double RollS = outRoll * 1.732;
			double half_outRoll = 0.5f * RollS;
			rp_out[0] = -outPitch+half_outRoll;
			rp_out[1] = RollS;
			rp_out[2] = +outPitch+half_outRoll;
			rp_out[3] = +outPitch-half_outRoll;
			rp_out[4] = -RollS;
			rp_out[5] = -outPitch-half_outRoll;
			double yaw_out[6];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			yaw_out[4] = -outYaw;
			yaw_out[5] = +outYaw;
			ctrl_Attitude_MultiRotor( 6, outRoll , outPitch , outYaw, rp_out, yaw_out );
			break;		
		}
		
		case UAVType_Rotor42_X:
		{
			double rp_out[8];   
			rp_out[0] = -outPitch+outRoll;
			rp_out[1] = +outPitch+outRoll;  
			rp_out[2] = +outPitch-outRoll;
			rp_out[3] = -outPitch-outRoll;
			rp_out[4] = -outPitch+outRoll;
			rp_out[5] = +outPitch+outRoll;  
			rp_out[6] = +outPitch-outRoll;
			rp_out[7] = -outPitch-outRoll;
			double yaw_out[8];
			yaw_out[0] = -outYaw;
			yaw_out[1] = +outYaw;
			yaw_out[2] = -outYaw;
			yaw_out[3] = +outYaw;
			yaw_out[4] = +outYaw;
			yaw_out[5] = -outYaw;
			yaw_out[6] = +outYaw;
			yaw_out[7] = -outYaw;
			ctrl_Attitude_MultiRotor( 8, outRoll , outPitch , outYaw, rp_out, yaw_out );
			break;  
		}		
		default:
			PWM_PullDownAll();
			break;
	}
}
	
//电机非线性输出 线性修正
static inline void throttle_nonlinear_compensation( double out[8] )
{
	double output_minimum_throttle = cfg.STThrottle[0];
	double output_range = 100.0f - output_minimum_throttle;
	double inv_output_range = 1.0 / output_range;
	
	//a：非线性因子(0-1)
	//m：最大油门比例(0.6-1)
	
	//设油门-力曲线方程为：
	//F = kx^2 + (1-a)x ( 0<=x<=m F最大值为1 )
	//x = m时：km^2 + (1-a)m = 1
	//得k = ( 1 - (1-a)m ) / m^2
	//a_1 = a - 1
	//Hk  = 1 / 2k
	//K4  = 4* k
	//解方程组：kx^2 + (1-a)x = out
	//得到的x即为线性化后的输出
	double _lift_max = cfg.FullThrRatio[0];
	double a_1 = cfg.NonlinearFactor[0] - 1;
	double k = ( 1 + a_1*_lift_max ) / (_lift_max*_lift_max);
	double Hk = 1.0f / (2*k);
	double K4 = 4 * k;
		
	for( uint8_t i = 0; i < 8; ++i )
	{
		if( out[i] > output_minimum_throttle - 0.1f )
		{
			out[i] -= output_minimum_throttle;
			out[i] *= inv_output_range;
			if( out[i] < 0 )
				out[i] = 0;
			out[i] = Hk*( a_1 + safe_sqrt( a_1*a_1 + K4*out[i] ) );
			out[i] *= output_range;
			out[i] += output_minimum_throttle;			
		}
		else
			out[i] = 0;
	}
}

static void ctrl_Attitude_MultiRotor( uint8_t mt_count, double outRoll, double outPitch, double outYaw, double rp_out[], double yaw_out[] )
{
	double rotor_output[8];
	double output_minimum_throttle = cfg.STThrottle[0];	
	
	if( throttle < output_minimum_throttle - 0.1f )
	{
		MainMotor_PullDownAll();
		update_output_throttle( 0 , 1.0/CtrlRateHz );
		return;
	}		
	
	double output_throttle = throttle;
	double output_midpoint = ( 100.0f - output_minimum_throttle ) / 2;
	
	/*pitch roll 输出限幅*/
		//如果需要的pitch roll输出超出当前油门能提供的输出范围
		//调整油门获得尽量满足pitch roll输出
		double output_max = fabs(rp_out[0]);
		for( uint8_t i = 1 ; i < mt_count ; ++i )
		{
			double abs_out = fabs(rp_out[i]);
			if( abs_out > output_max ) 
				output_max = abs_out;
		}
		
		double max_allow_output = 100.0f - output_throttle;
		double min_allow_output = output_minimum_throttle - output_throttle;			
		double allow_ouput_range;
		if( max_allow_output < -min_allow_output )
		{	//降低油门确保姿态输出
			allow_ouput_range = max_allow_output;
			if( output_max > allow_ouput_range )
			{	//需要降低油门
				if( output_max > output_midpoint )
				{	//输出超过最大输出范围
					//将油门调整为50%确保可以进行最大输出
					output_throttle = output_midpoint + output_minimum_throttle;
					allow_ouput_range = output_midpoint;
				}
				else
				{	//降低油门到所需值
					output_throttle = 100.0f - output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		else
		{	//抬高油门保证姿态输出
			allow_ouput_range = -min_allow_output;
			if( output_max > allow_ouput_range )
			{	//需要抬高油门
				
				//求最高允许的油门值（不能大于悬停油门）
				double hover_throttle_force = hover_throttle - output_minimum_throttle;
				double max_allowed_output_range = hover_throttle_force*0.85;
				if( output_midpoint < max_allowed_output_range )
					max_allowed_output_range = output_midpoint;				
				if( max_allowed_output_range < output_throttle - output_minimum_throttle )
					max_allowed_output_range = output_throttle - output_minimum_throttle;
				double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
				
				if( output_max > max_allowed_output_range )
				{	//输出范围大于最大允许范围
					//抬高油门至最大允许范围
					output_throttle = max_allowed_throttle;
					allow_ouput_range = max_allowed_output_range;
				}
				else
				{	//抬高油门到所需值
					output_throttle = output_minimum_throttle + output_max;
					allow_ouput_range = output_max;
				}
			}
		}
		
		//输出限幅修正
		if( output_max > allow_ouput_range )
		{	//需要修正输出
			double scale  = allow_ouput_range / output_max;
			for( uint8_t i = 0 ; i < mt_count ; ++i )
				rotor_output[i] = rp_out[i] * scale;
			Roll_u = outRoll * scale;
			Pitch_u = outPitch * scale;
		}		
		else
		{
			for( uint8_t i = 0 ; i < mt_count ; ++i )
				rotor_output[i] = rp_out[i];
			Roll_u = outRoll;
			Pitch_u = outPitch;
		}			
	/*pitch roll 输出限幅*/
	
	/*yaw output 输出限幅*/
		//抬升油门保住偏航输出
		
		//计算偏航输出到上下界距离
		double yaw_out_up=100, yaw_out_dn=100;
		for( uint8_t i = 0 ; i < mt_count ; ++i )
		{
			double current_rotor_output = output_throttle + rotor_output[i];
			max_allow_output = 100.0f - current_rotor_output;
			min_allow_output = current_rotor_output - output_minimum_throttle;
						
			if( yaw_out[i] > 0 )
			{	//更新距离上界距离
				double out_up = max_allow_output - yaw_out[i];
				if( out_up < yaw_out_up )
					yaw_out_up = out_up;
			}
			else
			{	//更新距离下界距离
				double out_dn = min_allow_output - -yaw_out[i];
				if( out_dn < yaw_out_dn )
					yaw_out_dn = out_dn;
			}
		}
		
		if( yaw_out_dn < 0 )
		{	//偏航输出超出下界
			//抬升油门保住偏航输出
			
			//求最高允许的油门值（不能大于悬停油门）
			double hover_throttle_force = hover_throttle - output_minimum_throttle;
			double max_allowed_output_range = hover_throttle_force*0.95;
			if( output_midpoint < max_allowed_output_range )
				max_allowed_output_range = output_midpoint;				
			if( max_allowed_output_range < output_throttle - output_minimum_throttle )
				max_allowed_output_range = output_throttle - output_minimum_throttle;
			double max_allowed_throttle = max_allowed_output_range + output_minimum_throttle;
			
			//期望抬升的油门量
			double req_up = -yaw_out_dn;
			//抬升油门量不能使输出超出上界
			if( req_up > yaw_out_up )
				req_up = yaw_out_up;
			if( req_up < 0 )
				req_up = 0;
			
			//抬升油门
			output_throttle += req_up;
			if( output_throttle > max_allowed_throttle )
				output_throttle = max_allowed_throttle;
		}

		/*yaw输出限幅计算*/
			double yaw_scale = 1.0;		
			for( uint8_t i = 0 ; i < mt_count ; ++i )
			{
				double current_rotor_output = output_throttle + rotor_output[i];
				max_allow_output = 100.0f - current_rotor_output;
				min_allow_output = output_minimum_throttle - current_rotor_output;
				if( yaw_out[i] > max_allow_output + 0.01f )
				{
					double new_yaw_scale = max_allow_output / yaw_out[i];
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
				else if( yaw_out[i] < min_allow_output - 0.01f )
				{
					double new_yaw_scale = min_allow_output / yaw_out[i];
					if( new_yaw_scale < yaw_scale ) yaw_scale = new_yaw_scale;
				}
			}						
		/*yaw输出限幅计算*/
		
		//lower yaw output to ensure attitude control and alt control
		if( yaw_scale < 0 )
			yaw_scale = 0;
		outYaw *= yaw_scale;
		Yaw_u = outYaw;
	/*yaw output 输出限幅*/
		
	//更新油门油门观测
	update_output_throttle( output_throttle , 1.0/CtrlRateHz );
	//补偿非线性输出
	for( uint8_t i = 0 ; i < mt_count ; ++i )
		rotor_output[i] += output_throttle + yaw_scale*yaw_out[i];
	throttle_nonlinear_compensation( rotor_output );
	MainMotor_PWM_Out( rotor_output );
}

void init_Ctrl_Attitude()
{
	AccZ_filter.set_cutoff_frequency( CtrlRateHz , 1.5 );
	WindDisturbance_filter[0].set_cutoff_frequency( CtrlRateHz , 0.1 );
	WindDisturbance_filter[1].set_cutoff_frequency( CtrlRateHz , 0.1 );
	
	//注册参数
	cfg.UAVType = UAVType_Rotor4_X;
	cfg.STThrottle[0] = 10;
	cfg.NonlinearFactor[0] = 0.45;
	cfg.FullThrRatio[0] = 0.95;
	cfg.T[0] = 0.1;
	cfg.b[0] = 5.5;	cfg.b[2] = 5.5;	cfg.b[4] = 1.0;
	cfg.TD4_P1[0] = 15;	cfg.TD4_P1[2] = 15;	cfg.TD4_P1[4] = 2;
	cfg.TD4_P2[0] = 15;	cfg.TD4_P2[2] = 15;	cfg.TD4_P2[4] = 5;
	cfg.TD4_P3[0] = 25;	cfg.TD4_P3[2] = 25;	cfg.TD4_P3[4] = 25;
	cfg.TD4_P4[0] = 25;	cfg.TD4_P4[2] = 25;	cfg.TD4_P4[4] = 25;
	cfg.P1[0] = 7;	cfg.P1[2] = 7;	cfg.P1[4] = 2;
	cfg.P2[0] = 10;	cfg.P2[2] = 10;	cfg.P2[4] = 5;
	cfg.P3[0] = 50;	cfg.P3[2] = 50;	cfg.P3[4] = 25;
	cfg.P4[0] = 15;	cfg.P4[2] = 15;	cfg.P4[4] = 15;
	cfg.beta[0] = 12;
	cfg.maxLean[0] = 35;
	cfg.maxRPSp[0] = 350;
	cfg.maxRPAcc[0] = 7000;
	cfg.maxYSp[0] = 80;
	cfg.maxYAcc[0] = 1000;
	MAV_PARAM_TYPE param_types[] = {
		MAV_PARAM_TYPE_UINT8 ,	//UAV Type
		MAV_PARAM_TYPE_REAL32 ,	//起转油门
		MAV_PARAM_TYPE_REAL32 ,	//非线性参数
		MAV_PARAM_TYPE_REAL32 ,	//满油门比例
		MAV_PARAM_TYPE_REAL32 ,	//T
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//b[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P1[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P2[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P3[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//TD4_P4[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P1[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P2[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P3[3]
		MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,MAV_PARAM_TYPE_REAL32 ,	//P4[3]
		MAV_PARAM_TYPE_REAL32 ,	//beta
		MAV_PARAM_TYPE_REAL32 ,	//最大倾斜角
		MAV_PARAM_TYPE_REAL32 ,	//maxRPSp
		MAV_PARAM_TYPE_REAL32 ,	//maxRPAcc
		MAV_PARAM_TYPE_REAL32 ,	//maxYSp
		MAV_PARAM_TYPE_REAL32 ,	//maxYAcc
	};
	SName param_names[] = {
		"AC_UAVType" ,	//UAV Type
		"AC_STThr" ,	//起转油门
		"AC_NonlinF" ,	//非线性参数
		"AC_FullThrR" ,	//满油门比例
		"AC_T" ,	//T
		"AC_Roll_b" ,"AC_Pitch_b" ,"AC_Yaw_b" ,	//b[3]
		"AC_Roll_TD4P1" ,"AC_Pitch_TD4P1" ,"AC_Yaw_TD4P1" ,	//TD4_P1[3]
		"AC_Roll_TD4P2" ,"AC_Pitch_TD4P2" ,"AC_Yaw_TD4P2" ,	//TD4_P2[3]
		"AC_Roll_TD4P3" ,"AC_Pitch_TD4P3" ,"AC_Yaw_TD4P3" ,	//TD4_P3[3]
		"AC_Roll_TD4P4" ,"AC_Pitch_TD4P4" ,"AC_Yaw_TD4P4" ,	//TD4_P4[3]
		"AC_Roll_P1" ,"AC_Pitch_P1" ,"AC_Yaw_P1" ,	//P1[3]
		"AC_Roll_P2" ,"AC_Pitch_P2" ,"AC_Yaw_P2" ,	//P2[3]
		"AC_Roll_P3" ,"AC_Pitch_P3" ,"AC_Yaw_P3" ,	//P3[3]
		"AC_Roll_P4" ,"AC_Pitch_P4" ,"AC_Yaw_P4" ,	//P4[3]
		"AC_Beta" ,	//beta
		"AC_maxLean" ,	//最大倾斜角
		"AC_maxRPSp" ,	//最大Pitch Roll速度
		"AC_maxRPAcc" ,	//最大Pitch Roll加速度
		"AC_maxYSp" ,	//最大偏航速度
		"AC_maxYAcc" ,	//最大偏航加速度
	};
	ParamGroupRegister( "AttCtrl", 3, 38, param_types, param_names, (uint64_t*)&cfg );
}