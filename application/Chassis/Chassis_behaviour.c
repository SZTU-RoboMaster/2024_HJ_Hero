//
// Created by Gularx on 2024/3/7.
//

#include "Chassis.h"
extern chassis_t chassis;
extern gimbal_t gimbal;

// 机械信息
static fp32 rotate_ratio_f = ((WHEELBASE+WHEELTRACK) / 2.0f - GIMBAL_OFFSET);// / RADIAN_COEF;
static fp32 rotate_ratio_b = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_OFFSET);// / RADIAN_COEF;
static fp32 wheel_rpm_ratio = 60.0f / (PERIMETER * M3508_DECELE_RATIO);


/**
  * @brief          底盘失能实现
  * @param[in]      pvParameters
  * 将四个轮子的电流信号给定为0
  * @retval         none
  */
void chassis_relax_handle() {
    CAN_cmd_motor(CAN_1,CAN_MOTOR_0x200_ID,0,0,0,0);
}


/**
  * @brief          底盘跟随云台模式的代码实现
  * @param[in]      pvParameters
  * 获取云台偏航角的相对角度，并转换为弧度 计算云台偏航角 保存底盘速度
  * 保存的底盘速度在云台坐标系下进行分解，得到在该坐标系下的底盘速度
  * 控制底盘的角速度，使其与云台的偏航角保持稳定，使用PID控制器计算底盘的角速度
  * @retval         none
  */
void chassis_follow_gimbal_handle(){
    //获取云台偏航角的相对角度，并转换为弧度
    fp32 yaw_relative_radian = gimbal.yaw.relative_angle_get * ANGLE_TO_RAD;
    fp32 sin_yaw,cos_yaw;

    // 计算云台偏航角
    sin_yaw = -(fp32)sin(yaw_relative_radian);
    cos_yaw =  (fp32)cos(yaw_relative_radian);

    // 保存底盘速度
    fp32 vx_temp = chassis.vx;
    fp32 vy_temp = chassis.vy;

    // 保存的底盘速度在云台坐标系下进行分解，得到在该坐标系下的底盘速度
    chassis.vx = (cos_yaw * vx_temp - sin_yaw * vy_temp);
    chassis.vy = (sin_yaw * vx_temp + cos_yaw * vy_temp);

    //控制底盘的角速度，使其与云台的偏航角保持稳定，使用PID控制器计算底盘的角速度
    chassis.vw = pid_calc(&chassis.chassis_vw_pid, gimbal.yaw.relative_angle_get, 0);

    if(abs(chassis.vw)<0.1){
        chassis.vw=0;
    }
    // 确保变量val的值在指定的范围内，即不小于最小值min且不大于最大值max
    //VAL_LIMIT(chassis.vw,-MAX_CHASSIS_VW_SPEED,MAX_CHASSIS_VW_SPEED);
}


/**
  * @brief          麦轮解算
  * @param[in]      pvParameters
  * 计算麦轮的速度
  * @retval         none
  */
fp32 vx, vy, vw;
void chassis_meknum_wheel_cal() {
    fp32 max = 0;
    int16_t wheel_rpm[4];

    vx=chassis.vx;
    vy=chassis.vy;
    vw=chassis.vw;
    // 根据给定的底盘运动参数，计算四个轮子的转速
    wheel_rpm[0] = ( vy - vx + vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[1] = ( vy + vx + vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = (-vy - vx + vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (-vy + vx + vw * rotate_ratio_b) * wheel_rpm_ratio;

    // find max item
    for (uint8_t i = 0; i < 4; i++) {
        if (abs(wheel_rpm[i]) > max) max=abs(wheel_rpm[i]);
    }

    // equal proportion
    // 如果超过最大转速，对所有轮子转速进行缩放，使其不超过最大转速
    if (max > M3508_MAX_RPM) {
        float rate = M3508_MAX_RPM / max;
        for (uint8_t i = 0; i < 4; i++) wheel_rpm[i] *= rate;
    }

    // 设置给定的转速
    chassis.motor_chassis[RF].rpm_set=wheel_rpm[0];
    chassis.motor_chassis[LF].rpm_set=wheel_rpm[1];
    chassis.motor_chassis[RB].rpm_set=wheel_rpm[2];
    chassis.motor_chassis[LB].rpm_set=wheel_rpm[3];
}


/**
  * @brief          麦轮电流给定
  * @param[in]      pvParameters
  * pid调整给定的麦轮电流
  * @retval         none
  */
void chassis_wheel_loop_cal(){
    chassis.motor_chassis[LF].give_current= pid_calc(&chassis.motor_chassis[LF].speed_p,
                                                     chassis.motor_chassis[LF].motor_measure->speed_rpm,
                                                     chassis.motor_chassis[LF].rpm_set);

    chassis.motor_chassis[RF].give_current= pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                     chassis.motor_chassis[RF].motor_measure->speed_rpm,
                                                     chassis.motor_chassis[RF].rpm_set);

    chassis.motor_chassis[RB].give_current= pid_calc(&chassis.motor_chassis[RB].speed_p,
                                                     chassis.motor_chassis[RB].motor_measure->speed_rpm,
                                                     chassis.motor_chassis[RB].rpm_set);

    chassis.motor_chassis[LB].give_current= pid_calc(&chassis.motor_chassis[LB].speed_p,
                                                     chassis.motor_chassis[LB].motor_measure->speed_rpm,
                                                     chassis.motor_chassis[LB].rpm_set);
}


/**
  * @brief          小陀螺
  * @param[in]      pvParameters
  * 计算云台坐标系下的底盘速度，给定固定的vw最大速度(150)
  * @retval         none
  */
void chassis_spin_handle(){
    //获取云台偏航角的相对角度，并转换为弧度
    fp32 yaw_relative_radian = -gimbal.yaw.relative_angle_get*ANGLE_TO_RAD;
    fp32 sin_yaw, cos_yaw;
    sin_yaw = -(fp32)sin(yaw_relative_radian);
    cos_yaw = (fp32)cos(yaw_relative_radian);

    // 保存底盘速度
    fp32 vx_temp = chassis.vx;
    fp32 vy_temp = chassis.vy;

    // 保存的底盘速度在云台坐标系下进行分解，得到在该坐标系下的底盘速度
    chassis.vx = (cos_yaw * vx_temp + sin_yaw * vy_temp);
    chassis.vy = -(sin_yaw * vx_temp - cos_yaw * vy_temp);
    // 给定vw转速
    chassis.vw = 420;//
}


void chassis_device_offline_handle() {
    if(detect_list[DETECT_REMOTE].status == OFFLINE&&detect_list[DETECT_VIDEO_TRANSIMITTER].status==OFFLINE)
        chassis.mode = CHASSIS_RELAX;//防止出现底盘疯转
}

//计算函数f(iset)=K*Ωset+M的系数K，M
void calc_power_limit(pid_type_def *pid,int i)
{
    //计算积分累计
    pid->sum_err += (pid->err[NOW]+pid->err[LAST])/2;
    //根据pid计算函数推出的
    chassis.chassis_power_limit.K[i]=pid->p*CHASSIS_CURRENT_CONVERT;
    chassis.chassis_power_limit.M[i]=-(pid->p*chassis.motor_chassis[i].motor_measure->speed_rpm-pid->iout)*CHASSIS_CURRENT_CONVERT;
    pid->sum_err = fp32_constrain(pid->sum_err,-CHASSIS_3508_PID_MAX_IOUT,-CHASSIS_3508_PID_MAX_IOUT);
}

void chassis_power_limit() {
    chassis.chassis_power_limit.power_set=Referee.GameRobotStat.chassis_power_limit-2;
    chassis.chassis_power_limit.k_c=1;
    //防止裁判系统串口离线时，Referee.GameRobotStat.chassis_power_limit=0，出现底盘被限0w的情况
    if(Referee.GameRobotStat.chassis_power_limit<=0){
        chassis.chassis_power_limit.power_set=70;
    }
    //更新电容状态
    cap_info_update();
    for (int i = 0; i < 4; ++i) {
        //计算K,M
        calc_power_limit(&chassis.motor_chassis[i].speed_p,i);
    }
    float a=0,b=0,c=0;
    for (int i = 0; i < 4; ++i) {
        //a,b,c的计算过程，具体参看华中科技大学功率控制模型
        a += CHASSIS_POWER_R0 * pow(chassis.motor_chassis[i].rpm_set,2) * pow(chassis.chassis_power_limit.K[i],2);
        b += chassis.motor_chassis[i].rpm_set * (2 * CHASSIS_POWER_R0 * chassis.chassis_power_limit.K[i] * chassis.chassis_power_limit.M[i] + CHASSIS_POWER_K0 * chassis.chassis_power_limit.K[i] * chassis.motor_chassis[i].motor_measure->speed_rpm);
        c += CHASSIS_POWER_R0 * pow(chassis.chassis_power_limit.M[i],2) + CHASSIS_POWER_K0 * chassis.chassis_power_limit.M[i] * chassis.motor_chassis[i].motor_measure->speed_rpm;
    }
    c += CHASSIS_POWER_P0;

    //算出下一周期的功率
    chassis.chassis_power_limit.predict_send_power = fp32_constrain(a+b+c,-1000,1000);
    c-=chassis.chassis_power_limit.power_set;
    bool_t dangerous=false;//缓冲能量过低触发危险信号
    //缓冲能量小于10J，危险信号开启
    if(Referee.PowerHeatData.chassis_power_buffer<50){
        dangerous=true;
        //系数c越大，限制的功率值越低，比如最大功率80w，限制的功率降至60w，那底盘功率被限制后最大仅可达到60w上下，几乎不会超过80w
        c+=(50-chassis.chassis_power_limit.power_buff)*1;
    }
    //当预测功率大于最大功率时或者缓冲能量过低时，启动功率限制，求出刚好令预测功率<=最大功率的系数K_c
    if(chassis.chassis_power_limit.predict_send_power > chassis.chassis_power_limit.power_set||dangerous)
    {
        if(a<=0){
            return;
        }
        if(b*b < 4*c*a)
        {
            chassis.chassis_power_limit.k_c = fp32_constrain(-b/(2*a),0.0f,1.0f);
        }
        else
        {
            float sqrt_result;
            arm_sqrt_f32(b*b - 4*c*a ,&sqrt_result);
            chassis.chassis_power_limit.k_c = fp32_constrain((-b + sqrt_result)/2/a,0.0f,1.0f);
        }
        //当开电容并且缓冲能量充足时，函数直接返回，不执行后续的功率控制
        //电容返回的数据具体参考电容手册
        if(!dangerous&&detect_list[DETECT_CAP].status==ONLINE){
            return;
        }
        for (int i = 0; i < 4; ++i) {
            //对rpm_set进行比例缩减，达到功率控制效果
            chassis.motor_chassis[i].rpm_set *= chassis.chassis_power_limit.k_c;
        }

    }
}


/**
  * @brief          底盘移动小一点
  * @param[in]      pvParameters
  * 底盘不跟着云台动  侧着打  在坡上打前哨战用
  * @retval         none
  */
void chassis_independent_handle(){

    chassis.vw=0;
    chassis.vx=(fp32)(chassis.vx*0.5);
    chassis.vy=(fp32)(chassis.vy*0.5);
}
