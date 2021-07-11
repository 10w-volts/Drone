#include "Sensors.hpp"
#include "SensorsBackend.hpp"
#include "Basic.hpp"
#include "Parameters.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "StorageSystem.hpp"
#include "ControlSystem.hpp"
#include "MeasurementSystem.hpp"

/*IMU*/

	const uint8_t External_Magnetometer_Index = 0;
	const uint8_t Internal_Magnetometer_Index = 2;

	static SemaphoreHandle_t AccelerometersMutex = xSemaphoreCreateMutex();
	static SemaphoreHandle_t GyroscopesMutex = xSemaphoreCreateMutex();
	static SemaphoreHandle_t MagnetometersMutex = xSemaphoreCreateMutex();
	static IMU_Sensor* Accelerometers[IMU_Sensors_Count] = {0};
	static IMU_Sensor* Gyroscopes[IMU_Sensors_Count] = {0};
	static IMU_Sensor* Magnetometers[IMU_Sensors_Count] = {0};
	
	
	
	
	/*IMU传感器注册函数*/
		bool IMUAccelerometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] != 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				Accelerometers[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Accelerometers[index];
				sensor->name = name;
				sensor->last_update_time = TIME::now();
				sensor->sensitivity = sensitivity;
				sensor->data.zero();
				sensor->data_raw.zero();
				
				//注册参数
				MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
				IMUConfig initial_cfg;
				initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
				initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
				initial_cfg.STTemperature = 0;
				initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
				ParamGroupRegister( name+"_Acc", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool IMUAccelerometerUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				delete Accelerometers[index];
				Accelerometers[index] = 0;
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		
		bool IMUGyroscopeRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] != 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				Gyroscopes[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Gyroscopes[index];
				sensor->name = name;
				sensor->last_update_time = TIME::now();
				sensor->sensitivity = sensitivity;
				sensor->data.zero();
				sensor->data_raw.zero();
				
				//注册参数
				MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
				IMUConfig initial_cfg;
				initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
				initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
				initial_cfg.STTemperature = 0;
				initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
				ParamGroupRegister( name+"_Gyro", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool IMUGyroscopeUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				delete Gyroscopes[index];
				Gyroscopes[index] = 0;
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		
		bool IMUMagnetometerRegister( uint8_t index, SName name, double sensitivity, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] != 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				Magnetometers[index] = new IMU_Sensor;
				IMU_Sensor* sensor = Magnetometers[index];
				sensor->name = name;
				sensor->last_update_time = TIME::now();
				sensor->sensitivity = sensitivity;
				sensor->data.zero();
				sensor->data_raw.zero();
				
				//注册参数
				MAV_PARAM_TYPE types[] = { MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_UINT64 };
				IMUConfig initial_cfg;
				initial_cfg.offset[0] = initial_cfg.offset[1] = initial_cfg.offset[2] = 0;
				initial_cfg.scale[0] = initial_cfg.scale[1] = initial_cfg.scale[2] = 1;
				initial_cfg.STTemperature = 0;
				initial_cfg.TemperatureCoefficient[0] = initial_cfg.TemperatureCoefficient[1] = initial_cfg.TemperatureCoefficient[2] = 0;
				ParamGroupRegister( name+"_Mag", 3, IMUConfigLength, types, 0, (uint64_t*)&initial_cfg );
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
		bool IMUMagnetometerUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				delete Magnetometers[index];
				Magnetometers[index] = 0;
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
	/*IMU传感器注册函数*/
		
	/*IMU传感器更新函数*/
		bool IMUAccelerometerUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
					
			vector3<int32_t> offset(0,0,0);			
			vector3<double> scale(1,1,1);			
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Accelerometers[index];
				//读取参数				
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Acc", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				sensor->data.x = (sensor->data_raw.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_raw.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_raw.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool IMUAccelerometerUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
					
			vector3<int32_t> offset(0,0,0);			
			vector3<double> scale(1,1,1);			
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Accelerometers[index];
				//读取参数				
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Acc", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = true;
				sensor->data_error = data_error;
			 	sensor->temperature = temperature;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				sensor->data.x = (sensor->data_rawTC.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_rawTC.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_rawTC.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		
		bool IMUGyroscopeUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Gyroscopes[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Gyro", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				sensor->data.x = (sensor->data_raw.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_raw.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_raw.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool IMUGyroscopeUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Gyroscopes[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Gyro", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = true;
				sensor->temperature = temperature;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				sensor->data.x = (sensor->data_rawTC.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_rawTC.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_rawTC.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		
		bool IMUMagnetometerUpdate( uint8_t index, vector3<int32_t> data, bool data_error, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Magnetometers[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Mag", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = false;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				sensor->data_rawTC.set_vector( data.x, data.y, data.z );
				sensor->data.x = (sensor->data_raw.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_raw.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_raw.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
		bool IMUMagnetometerUpdateTC( uint8_t index, vector3<int32_t> data, bool data_error, double temperature, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			vector3<double> offset(0,0,0);
			vector3<double> scale(1,1,1);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}
				
				IMU_Sensor* sensor = Magnetometers[index];
				//读取参数
				IMUConfig cfg;	bool is_new;
				if( ReadParamGroup( sensor->name + "_Mag", (uint64_t*)&cfg, &is_new ) == PR_OK )
				{
					if( !is_new )
					{
						offset.set_vector( cfg.offset[0], cfg.offset[1], cfg.offset[2] );
						scale.set_vector( cfg.scale[0], cfg.scale[1], cfg.scale[2] );
						sensor->calibrated = true;
					}
					else
						sensor->calibrated = false;
				}
				
				//写入更新时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//写入传感器数据
				sensor->have_temperature = true;
				sensor->temperature = temperature;
				sensor->data_error = data_error;
				sensor->data_raw = data;
				double temperature_err = temperature - cfg.STTemperature;
				sensor->data_rawTC.set_vector(
					sensor->data_raw.x - temperature_err*cfg.TemperatureCoefficient[0],
					sensor->data_raw.y - temperature_err*cfg.TemperatureCoefficient[1],
					sensor->data_raw.z - temperature_err*cfg.TemperatureCoefficient[2] );
				sensor->data.x = (sensor->data_rawTC.x - offset.x) * sensor->sensitivity * scale.x;
				sensor->data.y = (sensor->data_rawTC.y - offset.y) * sensor->sensitivity * scale.y;
				sensor->data.z = (sensor->data_rawTC.z - offset.z) * sensor->sensitivity * scale.z;
				
				xSemaphoreGive(MagnetometersMutex);				
				return true;
			}
			return false;
		}
	/*IMU传感器更新函数*/
		
	/*IMU传感器读取函数*/
		bool GetAccelerometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(AccelerometersMutex,TIMEOUT_Ticks) )
			{
				if( Accelerometers[index] == 0 )
				{
					xSemaphoreGive(AccelerometersMutex);
					return false;
				}				
				*sensor = *Accelerometers[index];
				
				xSemaphoreGive(AccelerometersMutex);
				return true;
			}
			return false;
		}
		bool GetGyroscope( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(GyroscopesMutex,TIMEOUT_Ticks) )
			{
				if( Gyroscopes[index] == 0 )
				{
					xSemaphoreGive(GyroscopesMutex);
					return false;
				}				
				*sensor = *Gyroscopes[index];
				
				xSemaphoreGive(GyroscopesMutex);
				return true;
			}
			return false;
		}
		bool GetMagnetometer( uint8_t index, IMU_Sensor* sensor, double TIMEOUT )
		{
			if( index >= IMU_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(MagnetometersMutex,TIMEOUT_Ticks) )
			{
				if( Magnetometers[index] == 0 )
				{
					xSemaphoreGive(MagnetometersMutex);
					return false;
				}				
				*sensor = *Magnetometers[index];
				
				xSemaphoreGive(MagnetometersMutex);
				return true;
			}
			return false;
		}
	/*IMU传感器读取函数*/
		
/*IMU*/
		
/*位置传感器*/
		
	const uint8_t default_ultrasonic_sensor_index = 1;
	const uint8_t default_optical_flow_index = 8;
	const uint8_t external_baro_sensor_index = 3;
	const uint8_t internal_baro_sensor_index = 4;
	const uint8_t default_rtk_sensor_index = 6;
	const uint8_t default_gps_sensor_index = 7;
		
	static SemaphoreHandle_t Position_Sensors_Mutex[Position_Sensors_Count] = {0};
	static Position_Sensor* Position_Sensors[Position_Sensors_Count];
		
	/*位置传感器注册函数*/
		bool PositionSensorRegister( 
			uint8_t index ,\
			Position_Sensor_Type sensor_type ,\
			Position_Sensor_DataType sensor_data_type ,\
			Position_Sensor_frame sensor_vel_frame ,\
			double delay ,\
			double xy_trustD , \
			double z_trustD , \
			const double* addition_inf,	\
			double TIMEOUT \
		)
		{
			if( index >= Position_Sensors_Count )
				return false;			
							
			if( delay < 0 )
				return false;
			if( xy_trustD < 0 )
				return false;
			if( z_trustD < 0 )
				return false;
			
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{
				if( Position_Sensors[index] != 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				Position_Sensors[index] = new Position_Sensor;
				Position_Sensor* sensor = Position_Sensors[index];
				sensor->available = false;
				sensor->available_status_update_time = TIME::now();
				sensor->last_update_time = TIME::now();
				sensor->sensor_type = sensor_type;
				sensor->sensor_DataType = sensor_data_type;
				sensor->velocity_data_frame = sensor_vel_frame;
				sensor->delay = delay;
				sensor->xy_trustD = xy_trustD;
				sensor->z_trustD = z_trustD;
				sensor->sample_time = -1;
				sensor->mp.initialized = false;				
				sensor->data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->addition_inf, addition_inf, 8*sizeof(double) );
				else
					memset( sensor->addition_inf, 0, 8*sizeof(double) );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUnRegister( uint8_t index, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				delete sensor;
				Position_Sensors[index] = 0;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
	/*位置传感器注册函数*/
	
	//更改位置传感器DataType
	bool PositionSensorChangeDataType( uint8_t index, Position_Sensor_DataType datatype, double TIMEOUT )
	{
		if( index >= Position_Sensors_Count )
			return false;
		
		TickType_t TIMEOUT_Ticks;
		if( TIMEOUT >= 0 )
			TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
		else
			TIMEOUT_Ticks = portMAX_DELAY;
		if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
		{	//锁定传感器
			if( Position_Sensors[index] == 0 )
			{
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return false;
			}
			
			Position_Sensor* sensor = Position_Sensors[index];
			sensor->sensor_DataType = datatype;
			
			xSemaphoreGive(Position_Sensors_Mutex[index]);
			return true;
		}	//解锁传感器
		return false;
	}
		
	/*位置传感器更新函数*/
		
		//失能位置传感器
		bool PositionSensorSetInavailable( uint8_t index, const double* addition_inf, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			Quaternion quat;
			get_AirframeY_quat(&quat);
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				//更新可用状态
				if( sensor->available != false )
					sensor->available_status_update_time = TIME::now();
				sensor->available = false;
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();
				//复位姿态
				sensor->data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);				
				return true;
			}	//解锁传感器
			return false;
		}
	
		bool PositionSensorUpdatePositionGlobal( uint8_t index, vector3<double> position_Global, bool available, double delay, double xy_trustD, double z_trustD, const double* addition_inf, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			//读取偏移校准
			char ofs_ind[2] = {0};
			ofs_ind[0] = index + '0';
			SName ofs_name = SName("POfs_S")+SName(ofs_ind);
			float ofs_value_x[2]={0};					
			float ofs_value_y[2]={0};	
			float ofs_value_z[2]={0};	
			ReadParam( ofs_name+SName("_x"), 0, 0, (uint64_t*)ofs_value_x, 0 );
			ReadParam( ofs_name+SName("_y"), 0, 0, (uint64_t*)ofs_value_y, 0 );
			ReadParam( ofs_name+SName("_z"), 0, 0, (uint64_t*)ofs_value_z, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			
			//获取历史姿态
			if( delay >= 0 )
			{
				if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
				{	//锁定传感器获取延时时间
					Position_Sensor* sensor = Position_Sensors[index];
					delay = sensor->delay;
					
					xSemaphoreGive(Position_Sensors_Mutex[index]);
				}
				else
					return false;
			}
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, delay );		
			
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_s_xy:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_z:
						if( __ARM_isnan( position_Global.z ) || __ARM_isinf( position_Global.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_xyz:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || __ARM_isnan( position_Global.z ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || __ARM_isinf( position_Global.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}								
				
				//更新可用状态
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();				
				//更新延时时间
				if( delay >= 0 )
					sensor->delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//更新数据
				sensor->position_Global = position_Global;				
				
				//经纬坐标平面投影
				if( available )
				{					
					if( sensor->mp.initialized == false )
					{
						map_projection_init( &sensor->mp , position_Global.x , position_Global.y );
					}
					double pos_x , pos_y;
					map_projection_project( &sensor->mp , position_Global.x , position_Global.y , &pos_x , &pos_y );
					
					vector3<double> offset_comp = quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
					sensor->position.x = pos_x - offset_comp.x;
					sensor->position.y = pos_y - offset_comp.y;
					sensor->position.z = position_Global.z - offset_comp.z;
				}
					
				//记录姿态
				sensor->data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdatePosition( uint8_t index, vector3<double> position, bool available, double delay, double xy_trustD, double z_trustD, const double* addition_inf, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			//读取偏移校准
			char ofs_ind[2] = {0};
			ofs_ind[0] = index + '0';
			SName ofs_name = SName("POfs_S")+SName(ofs_ind);
			float ofs_value_x[2]={0};					
			float ofs_value_y[2]={0};	
			float ofs_value_z[2]={0};	
			ReadParam( ofs_name+SName("_x"), 0, 0, (uint64_t*)ofs_value_x, 0 );
			ReadParam( ofs_name+SName("_y"), 0, 0, (uint64_t*)ofs_value_y, 0 );
			ReadParam( ofs_name+SName("_z"), 0, 0, (uint64_t*)ofs_value_z, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			
			//获取历史姿态
			if( delay >= 0 )
			{
				if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
				{	//锁定传感器获取延时时间
					Position_Sensor* sensor = Position_Sensors[index];
					delay = sensor->delay;
					
					xSemaphoreGive(Position_Sensors_Mutex[index]);
				}
				else
					return false;
			}
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, delay );
			
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_s_xy:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_z:
						if( __ARM_isnan( position.z ) || __ARM_isinf( position.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_s_xyz:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || __ARM_isnan( position.z ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || __ARM_isinf( position.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();	
				//延时大于0更新延时
				if( delay > 0 )
					sensor->delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;				
				
				//更新数据
				vector3<double> offset_comp = quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				sensor->position = position - offset_comp;					
				
				//记录姿态
				sensor->data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdatePositionGlobalVel( uint8_t index, vector3<double> position_Global, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, const double* addition_inf, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			//读取偏移校准
			char ofs_ind[2] = {0};
			ofs_ind[0] = index + '0';
			SName ofs_name = SName("POfs_S")+SName(ofs_ind);
			float ofs_value_x[2]={0};					
			float ofs_value_y[2]={0};	
			float ofs_value_z[2]={0};	
			ReadParam( ofs_name+SName("_x"), 0, 0, (uint64_t*)ofs_value_x, 0 );
			ReadParam( ofs_name+SName("_y"), 0, 0, (uint64_t*)ofs_value_y, 0 );
			ReadParam( ofs_name+SName("_z"), 0, 0, (uint64_t*)ofs_value_z, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			
			//获取历史姿态
			if( delay >= 0 )
			{
				if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
				{	//锁定传感器获取延时时间
					Position_Sensor* sensor = Position_Sensors[index];
					delay = sensor->delay;
					
					xSemaphoreGive(Position_Sensors_Mutex[index]);
				}
				else
					return false;
			}
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, delay );
			
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_sv_xy:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_z:
						if( __ARM_isnan( position_Global.z ) || __ARM_isinf( position_Global.z ) || \
								__ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_xyz:
						if( __ARM_isnan( position_Global.x ) || __ARM_isnan( position_Global.y ) || __ARM_isnan( position_Global.z ) || \
								__ARM_isinf( position_Global.x ) || __ARM_isinf( position_Global.y ) || __ARM_isinf( position_Global.z ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();				
				//更新延时时间
				if( delay > 0 )
					sensor->delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//更新数据
				sensor->position_Global = position_Global;
				vector3<double> offset_comp = quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				vector3<double> d_offset;
				if( sensor->velocity_data_frame == Position_Sensor_frame_ENU )
					d_offset = offset_comp - sensor->data_quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );	
				else
				{
					d_offset = quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) )
										-sensor->data_quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				}
				
				double sample_freq = 0;
				if( sensor->sample_time > 0.0001 )
					sample_freq = 1.0 / sensor->sample_time;
				sensor->velocity = vel - d_offset*sample_freq;
				//经纬坐标平面投影				
				if( available )
				{
					if( sensor->mp.initialized == false )
					{
						map_projection_init( &sensor->mp , position_Global.x , position_Global.y );
					}
					double pos_x , pos_y;
					map_projection_project( &sensor->mp , position_Global.x , position_Global.y , &pos_x , &pos_y );
					
					sensor->position.x = pos_x - offset_comp.x;
					sensor->position.y = pos_y - offset_comp.y;
					sensor->position.z = position_Global.z - offset_comp.z;
				}
					
				//记录姿态
				sensor->data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdatePositionVel( uint8_t index, vector3<double> position, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, const double* addition_inf, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			//读取偏移校准
			char ofs_ind[2] = {0};
			ofs_ind[0] = index + '0';
			SName ofs_name = SName("POfs_S")+SName(ofs_ind);
			float ofs_value_x[2]={0};					
			float ofs_value_y[2]={0};	
			float ofs_value_z[2]={0};	
			ReadParam( ofs_name+SName("_x"), 0, 0, (uint64_t*)ofs_value_x, 0 );
			ReadParam( ofs_name+SName("_y"), 0, 0, (uint64_t*)ofs_value_y, 0 );
			ReadParam( ofs_name+SName("_z"), 0, 0, (uint64_t*)ofs_value_z, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			
			//获取历史姿态
			if( delay >= 0 )
			{
				if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
				{	//锁定传感器获取延时时间
					Position_Sensor* sensor = Position_Sensors[index];
					delay = sensor->delay;
					
					xSemaphoreGive(Position_Sensors_Mutex[index]);
				}
				else
					return false;
			}
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, delay );
			
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_sv_xy:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_z:
						if( __ARM_isnan( position.z ) || __ARM_isinf( position.z ) || \
								__ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_sv_xyz:
						if( __ARM_isnan( position.x ) || __ARM_isnan( position.y ) || __ARM_isnan( position.z ) || \
								__ARM_isinf( position.x ) || __ARM_isinf( position.y ) || __ARM_isinf( position.z ) || \
								__ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();							
				//延时大于0更新延时
				if( delay > 0 )
					sensor->delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//更新数据
				vector3<double> offset_comp = quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				vector3<double> d_offset;
				if( sensor->velocity_data_frame == Position_Sensor_frame_ENU )
					d_offset = offset_comp - sensor->data_quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );	
				else
				{
					d_offset = quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) )
										-sensor->data_quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				}
				double sample_freq = 0;
				if( sensor->sample_time > 0.0001 )
					sample_freq = 1.0 / sensor->sample_time;
				sensor->position = position - offset_comp;				
				sensor->velocity = vel - d_offset*sample_freq;
				
				//记录姿态
				sensor->data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
		bool PositionSensorUpdateVel( uint8_t index, vector3<double> vel, bool available, double delay, double xy_trustD, double z_trustD, const double* addition_inf, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			bool inFlight;
			get_is_inFlight(&inFlight);
			uint64_t log = 0;
			ReadParam( "SDLog_PosSensor", 0, 0, (uint64_t*)&log, 0 );
			
			//读取偏移校准
			char ofs_ind[2] = {0};
			ofs_ind[0] = index + '0';
			SName ofs_name = SName("POfs_S")+SName(ofs_ind);
			float ofs_value_x[2]={0};					
			float ofs_value_y[2]={0};	
			float ofs_value_z[2]={0};	
			ReadParam( ofs_name+SName("_x"), 0, 0, (uint64_t*)ofs_value_x, 0 );
			ReadParam( ofs_name+SName("_y"), 0, 0, (uint64_t*)ofs_value_y, 0 );
			ReadParam( ofs_name+SName("_z"), 0, 0, (uint64_t*)ofs_value_z, 0 );
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			
			//获取历史姿态
			if( delay >= 0 )
			{
				if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
				{	//锁定传感器获取延时时间
					Position_Sensor* sensor = Position_Sensors[index];
					delay = sensor->delay;
					
					xSemaphoreGive(Position_Sensors_Mutex[index]);
				}
				else
					return false;
			}
			//获取延时补偿姿态
			Quaternion quat;
			get_history_AirframeQuatY( &quat, delay );
			
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				
				//判断传感器类型、数据是否正确
				bool data_effective;
				switch( sensor->sensor_DataType )
				{
					case Position_Sensor_DataType_v_xy:
						if( __ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_v_z:
						if( __ARM_isnan( vel.z ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					case Position_Sensor_DataType_v_xyz:
						if( __ARM_isnan( vel.x ) || __ARM_isnan( vel.y ) || __ARM_isnan( vel.z ) || \
								__ARM_isinf( vel.x ) || __ARM_isinf( vel.y ) || __ARM_isinf( vel.z ) )
							data_effective = false;
						else
							data_effective = true;
						break;
						
					default:
						data_effective = false;
						break;
				}
				if( !data_effective )
				{	//数据出错退出
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				//更新可用状态
				if( sensor->available != available )
					sensor->available_status_update_time = TIME::now();
				sensor->available = available;
				
				//更新采样时间
				sensor->sample_time = sensor->last_update_time.get_pass_time_st();							
				//延时大于0更新延时
				if( delay > 0 )
					sensor->delay = delay;
				//更新信任度
				if( xy_trustD >= 0 )
					sensor->xy_trustD = xy_trustD;
				if( z_trustD >= 0 )
					sensor->z_trustD = z_trustD;
				
				//更新数据
				vector3<double> offset_comp = quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				vector3<double> d_offset;
				if( sensor->velocity_data_frame == Position_Sensor_frame_ENU )
					d_offset = offset_comp - sensor->data_quat.rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );	
				else
				{
					d_offset = quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) )
										-sensor->data_quat.get_RP_quat().rotate( vector3<double>(ofs_value_x[0],ofs_value_y[0],ofs_value_z[0]) );
				}
				double sample_freq = 0;
				if( sensor->sample_time > 0.0001 )
					sample_freq = 1.0 / sensor->sample_time;		
				sensor->velocity = vel - d_offset*sample_freq;
				
				//记录姿态
				sensor->data_quat = quat;
				
				//记录附加信息
				if(addition_inf)
					memcpy( sensor->addition_inf, addition_inf, 8*sizeof(double) );
				
				//记录位置数据
				if(inFlight && log)
					SDLog_Msg_PosSensor( index, *sensor );
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
	/*位置传感器更新函数*/
		
	/*位置传感器读取函数*/
		bool GetPositionSensor( uint8_t index, Position_Sensor* result_sensor, double TIMEOUT )
		{
			if( index >= Position_Sensors_Count )
				return false;
			
			TickType_t TIMEOUT_Ticks;
			if( TIMEOUT >= 0 )
				TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
			else
				TIMEOUT_Ticks = portMAX_DELAY;
			if( xSemaphoreTake(Position_Sensors_Mutex[index],TIMEOUT_Ticks) )
			{	//锁定传感器
				if( Position_Sensors[index] == 0 )
				{
					xSemaphoreGive(Position_Sensors_Mutex[index]);
					return false;
				}
				
				Position_Sensor* sensor = Position_Sensors[index];
				*result_sensor = *sensor;
				
				xSemaphoreGive(Position_Sensors_Mutex[index]);
				return true;
			}	//解锁传感器
			return false;
		}
	/*位置传感器读取函数*/
	
/*位置传感器*/
		
void init_Sensors()
{
	//初始化传感器互斥锁
	for( int i = 0; i < Position_Sensors_Count; ++i )
		Position_Sensors_Mutex[i] = xSemaphoreCreateMutex();
	
	/*注册传感器位置偏移参数*/
		SensorPosOffset initial_posoffset;
		//飞控偏移
		initial_posoffset.Fc_x[0] = 0;
		initial_posoffset.Fc_y[0] = 0;
		initial_posoffset.Fc_z[0] = 0;
		//传感器0偏移
		initial_posoffset.S0_x[0] = 0;
		initial_posoffset.S0_y[0] = 0;
		initial_posoffset.S0_z[0] = 0;
		//传感器1偏移
		initial_posoffset.S1_x[0] = 0;
		initial_posoffset.S1_y[0] = 0;
		initial_posoffset.S1_z[0] = 0;
		//传感器2偏移
		initial_posoffset.S2_x[0] = 0;
		initial_posoffset.S2_y[0] = 0;
		initial_posoffset.S2_z[0] = 0;
		//传感器3偏移
		initial_posoffset.S3_x[0] = 0;
		initial_posoffset.S3_y[0] = 0;
		initial_posoffset.S3_z[0] = 0;
		//传感器4偏移
		initial_posoffset.S4_x[0] = 0;
		initial_posoffset.S4_y[0] = 0;
		initial_posoffset.S4_z[0] = 0;
		//传感器5偏移
		initial_posoffset.S5_x[0] = 0;
		initial_posoffset.S5_y[0] = 0;
		initial_posoffset.S5_z[0] = 0;
		//传感器6偏移
		initial_posoffset.S6_x[0] = 0;
		initial_posoffset.S6_y[0] = 0;
		initial_posoffset.S6_z[0] = 0;
		//传感器7偏移
		initial_posoffset.S7_x[0] = 0;
		initial_posoffset.S7_y[0] = 0;
		initial_posoffset.S7_z[0] = 0;
		//传感器8偏移
		initial_posoffset.S8_x[0] = 0;
		initial_posoffset.S8_y[0] = 0;
		initial_posoffset.S8_z[0] = 0;
		//传感器9偏移
		initial_posoffset.S9_x[0] = 0;
		initial_posoffset.S9_y[0] = 0;
		initial_posoffset.S9_z[0] = 0;
		//传感器10偏移
		initial_posoffset.S10_x[0] = 0;
		initial_posoffset.S10_y[0] = 0;
		initial_posoffset.S10_z[0] = 0;
		//传感器11偏移
		initial_posoffset.S11_x[0] = 0;
		initial_posoffset.S11_y[0] = 0;
		initial_posoffset.S11_z[0] = 0;
		//传感器12偏移
		initial_posoffset.S12_x[0] = 0;
		initial_posoffset.S12_y[0] = 0;
		initial_posoffset.S12_z[0] = 0;
		//传感器13偏移
		initial_posoffset.S13_x[0] = 0;
		initial_posoffset.S13_y[0] = 0;
		initial_posoffset.S13_z[0] = 0;
		//传感器14偏移
		initial_posoffset.S14_x[0] = 0;
		initial_posoffset.S14_y[0] = 0;
		initial_posoffset.S14_z[0] = 0;
		//传感器15偏移
		initial_posoffset.S15_x[0] = 0;
		initial_posoffset.S15_y[0] = 0;
		initial_posoffset.S15_z[0] = 0;
	
		MAV_PARAM_TYPE param_types[] = {
			//飞控偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器0偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器1偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器2偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器3偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器4偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器5偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器6偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器7偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器8偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器9偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器10偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器11偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器12偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器13偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器14偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
			//传感器15偏移
			MAV_PARAM_TYPE_REAL32 ,	//x
			MAV_PARAM_TYPE_REAL32 ,	//y
			MAV_PARAM_TYPE_REAL32 ,	//z
		};
		
		//飞控偏移		
		SName param_names[] = {
			//飞控偏移
			"POfs_Fc_x" ,	//x
			"POfs_Fc_y" ,	//y
			"POfs_Fc_z" ,	//z
			//传感器0偏移
			"POfs_S0_x" ,	//x
			"POfs_S0_y" ,	//y
			"POfs_S0_z" ,	//z
			//传感器1偏移
			"POfs_S1_x" ,	//x
			"POfs_S1_y" ,	//y
			"POfs_S1_z" ,	//z
			//传感器2偏移
			"POfs_S2_x" ,	//x
			"POfs_S2_y" ,	//y
			"POfs_S2_z" ,	//z
			//传感器3偏移
			"POfs_S3_x" ,	//x
			"POfs_S3_y" ,	//y
			"POfs_S3_z" ,	//z
			//传感器4偏移
			"POfs_S4_x" ,	//x
			"POfs_S4_y" ,	//y
			"POfs_S4_z" ,	//z
			//传感器5偏移
			"POfs_S5_x" ,	//x
			"POfs_S5_y" ,	//y
			"POfs_S5_z" ,	//z
			//传感器6偏移
			"POfs_S6_x" ,	//x
			"POfs_S6_y" ,	//y
			"POfs_S6_z" ,	//z
			//传感器7偏移
			"POfs_S7_x" ,	//x
			"POfs_S7_y" ,	//y
			"POfs_S7_z" ,	//z
			//传感器8偏移
			"POfs_S8_x" ,	//x
			"POfs_S8_y" ,	//y
			"POfs_S8_z" ,	//z
			//传感器9偏移
			"POfs_S9_x" ,	//x
			"POfs_S9_y" ,	//y
			"POfs_S9_z" ,	//z
			//传感器10偏移
			"POfs_S10_x" ,	//x
			"POfs_S10_y" ,	//y
			"POfs_S10_z" ,	//z
			//传感器11偏移
			"POfs_S11_x" ,	//x
			"POfs_S11_y" ,	//y
			"POfs_S11_z" ,	//z
			//传感器12偏移
			"POfs_S12_x" ,	//x
			"POfs_S12_y" ,	//y
			"POfs_S12_z" ,	//z
			//传感器13偏移
			"POfs_S13_x" ,	//x
			"POfs_S13_y" ,	//y
			"POfs_S13_z" ,	//z
			//传感器14偏移
			"POfs_S14_x" ,	//x
			"POfs_S14_y" ,	//y
			"POfs_S14_z" ,	//z
			//传感器15偏移
			"POfs_S15_x" ,	//x
			"POfs_S15_y" ,	//y
			"POfs_S15_z" ,	//z
		};
		ParamGroupRegister( "PosOffset", 1, 51, param_types, param_names, (uint64_t*)&initial_posoffset );
}