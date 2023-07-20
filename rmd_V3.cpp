#include "rmd_V3.h"
using namespace mbed;

RMDmotor::RMDmotor() : addr(0x141), can(CAN_RX, CAN_TX)
{    txmsg.id = addr;   }

RMDmotor::RMDmotor(uint8_t id) : addr(140 | id), can(CAN_RX, CAN_TX)
{    txmsg.id = addr;   }

void RMDmotor::init()
{
    while (!can.frequency(1000000))
        Serial.println("Bps setting....");
    while (!can.mode(CAN::Normal))
        Serial.println("Mode setting...");
    // can.filter(CAN_RMD_ID, CAN_MASK, CANStandard, 0);
    // can.filter(CAN_MOTION_ID, CAN_MASK, CANStandard, 1);
    can.filter(addr ^ 0x300, CAN_MASK, CANStandard, (addr&0xf)*2);
    can.filter(addr ^ 0x440, CAN_MASK, CANStandard, (addr&0xf)*2-1);
}

uint8_t RMDmotor::sendCMD(byte req[])
{
    txmsg.id=addr;
    memcpy(txmsg.data, req, sizeof(req));
    uint8_t result = can.write(txmsg);
    return result;
}
uint8_t RMDmotor::sendCMD(byte req[], byte res[])
{
    txmsg.id=addr;
    memcpy(txmsg.data, req, sizeof(req));
    uint8_t result = can.write(txmsg);
    if(result==0) return CAN_FAILED; 
    while(!can.read(rxmsg));
    memcpy(res, rxmsg.data, sizeof(rxmsg.data));
    return CAN_OK;
}

bool RMDmotor::offMotor()
{
    byte res[8] = {0, };
    uint8_t can_res = sendCMD(frame_off, res);
    if (can_res == CAN_OK)
    {
        return memcmp(res, frame_off, 8) == 0;
    }
    return false;
}

bool RMDmotor::pauseMotor()
{
    byte res[8] = {0, };
    uint8_t can_res = sendCMD(frame_pause, res);
    if (can_res == CAN_OK)
    {
        return memcmp(res, frame_pause, 8) == 0;
    }
    return false;
}

bool RMDmotor::writePIDRAM()
{
    byte res[8] = {0, };
    uint8_t can_res = sendCMD(frame_write_RAMPID, res);
    if (can_res == CAN_OK)
    {
        return memcmp(res, frame_write_RAMPID, 8) == 0;
    }
    return false;
}

bool RMDmotor::writePIDRAM(byte set[])
{
    byte res[8] = {0, };
    memcpy(frame_write_RAMPID + 8 - sizeof(set), set, sizeof(set));
    uint8_t can_res = sendCMD(frame_write_RAMPID, res);
    if (can_res == CAN_OK)
    {
        return memcmp(res, frame_write_RAMPID, 8) == 0;
    }
    return false;
}

bool RMDmotor::restart()
{
    return CAN_OK == sendCMD(frame_system_reset);
}

/* Torque unit is 0.01A/LSB */
bool RMDmotor::torqueControl1(const int16_t &torque)
{
    *((int16_t *)(frame_torque_control + 4)) = torque;
    return CAN_OK == sendCMD(frame_torque_control, (byte *)&current_state2);
}
bool RMDmotor::torqueControl2(const uint8_t &direction, const int16_t &torque)
{
    if (direction == CMD_MOTOR_DIR_CCW)
        return torqueControl1(torque * (-1));
    else
        return torqueControl1(torque);
}

/* Speed unit is 0.01dps/LSB */
bool RMDmotor::speedControl1(const int32_t &speed)
{
    *((int32_t *)(frame_speed_control + 4)) = speed;
    return CAN_OK == sendCMD(frame_speed_control, (byte *)&current_state2);
}
bool RMDmotor::speedControl2(const uint8_t &direction, const int32_t &speed)
{
    if (direction == CMD_MOTOR_DIR_CCW)
        return speedControl1(speed * (-1));
    else
        return speedControl1(speed);
}

/* Max Speed unit is 0.01dps/LSB, Position unit is 0.01deg/LSB */
bool RMDmotor::absposControl(const uint16_t &speed_limit, const int32_t &pos)
{
    *((uint16_t *)(frame_abs_pos_control + 2)) = speed_limit;
    *((int32_t *)(frame_abs_pos_control + 4)) = pos;
    return CAN_OK == sendCMD(frame_abs_pos_control, (byte*)&current_state2);
}
bool RMDmotor::singleposControl(const uint8_t &direct, const uint16_t &speed_limit, const uint16_t &pos)
{
    *((uint8_t *)(frame_single_pos_control + 1)) = direct;
    *((uint16_t *)(frame_single_pos_control + 2)) = speed_limit;
    *((uint16_t *)(frame_single_pos_control + 4)) = pos;
    return CAN_OK == sendCMD(frame_single_pos_control, (byte *)&current_state2);
}
bool RMDmotor::increposControl(const uint16_t &speed_limit, const int32_t &pos)
{
    *((uint16_t *)(frame_incre_pos_control + 2)) = speed_limit;
    *((int32_t *)(frame_incre_pos_control + 4)) = pos;
    return CAN_OK == sendCMD(frame_incre_pos_control, (byte *)&current_state2);
}

/* idx = 0 is clear multi-turn value, 1 is CAN filter */
bool RMDmotor::functioncontrol(const uint8_t& idx, const uint32_t& value){
    byte res[8] = {0, };
    *((uint8_t*)(frame_function_control+1)) = idx;
    *((uint32_t*)(frame_function_control+4))= value;
    uint8_t result = sendCMD(frame_function_control, res);
    if (result == CAN_OK)
    {
        return memcmp(res, frame_function_control, 8) == 0;
    }
    return false;
} 



/* States unit : p_des is rad, v_des is rad/s */
uint8_t RMDmotor::RAW_motionControl(const uint16_t &p_des, const uint16_t &v_des, const uint16_t &Kp, const uint16_t &Kd, const uint16_t &t_ff, byte res[])
{
    byte frame_motion_control[8] = {0, };
    *((uint16_t *)(frame_motion_control)) = (p_des << 8 | p_des >> 8);      // rad(-12.5~12.5)
    *((uint16_t *)(frame_motion_control + 2)) = (v_des << 12 | v_des >> 4); // rad/s (-45~45)
    *((uint16_t *)(frame_motion_control + 3)) |= (Kp << 8 | Kp >> 8);       // (0~500)
    *((uint16_t *)(frame_motion_control + 5)) = (Kd << 12 | Kd >> 4);       // (0~5)
    *((uint16_t *)(frame_motion_control + 6)) |= (t_ff << 8 | t_ff >> 8);   // (-24~24)
    txmsg.id = addr^0x540;
    memcpy(txmsg.data, frame_motion_control, sizeof(frame_motion_control));
    uint8_t result = can.write(txmsg);
    if(result==0) return CAN_FAILED;
    while (!can.read(rxmsg));
    memcpy(res, rxmsg.data, sizeof(rxmsg.data));
    return result;
    
}
bool RMDmotor::motioncontrol(const double &angle, const double &speed, const double &Kp, const double &Kd, const double &tor)
{
    uint16_t pos=angle*(M_PI/180*65535./25)+65535./2;
    pos = (pos > 65535) ? 65535 : (pos < 0) ? 0 : pos;
    uint16_t vel = speed*(M_PI/180*4095./90)+4095./2;
    vel = (vel > 4095) ? 4095 : (vel < 0) ? 0 : vel;
    uint16_t kp = Kp;
    uint16_t kd = Kd;
    uint16_t t_ff = (tor + 24) / 48 * 4095;
    return CAN_OK == RAW_motionControl(pos, vel, kp, kd, t_ff, (byte *)&pre_state);
}

/* need to reset to be effective */
bool RMDmotor::zero_offset()
{
    uint8_t result = sendCMD(frame_zero_offset);
    return CAN_OK == result;
}

/* unit is 0.01deg/LSB */
bool RMDmotor::updateMultAngle()
{
    bool result = CAN_OK == sendCMD(frame_read_multi_angle, (byte *)frame_res_multi_angle);
    multi_angle = *((int32_t *)(frame_res_multi_angle + 4));
    return result;
}
bool RMDmotor::updateSingleAngle()
{
    bool result = CAN_OK == sendCMD(frame_read_single_angle, (byte *)frame_res_single_angle);
    single_angle = *((uint16_t *)(frame_res_single_angle + 6));
    return result;
}

/* need to call this function before get motion_states */
void RMDmotor::calculate_motionstate()
{
    uint16_t pre_pos = (pre_state.angle << 8) | (pre_state.angle >> 8);
    uint16_t pre_vel = (pre_state.vel_tor << 4 & 0xff0) | (pre_state.vel_tor >> 12 & 0xf);
    uint16_t pre_tor = (pre_state.vel_tor & 0xf00) | (pre_state.vel_tor >> 16 & 0xff);

    motion_state.deg = ((pre_pos / 65535.) * 25. - 12.5) * 180 / M_PI;
    motion_state.vel = ((pre_vel / 4095.) * 90. - 45) * 180 / M_PI;
    motion_state.tor = (pre_tor / 4095. * 48.) - 24;
}

bool RMDmotor::updateCurrentState2()
{
    return CAN_OK == sendCMD(frame_read_status2, (byte *)&current_state2);
}

bool RMDmotor::updateCurrentState1()
{
    return CAN_OK == sendCMD(frame_read_status1, (byte *)&current_state1);
}

bool RMDmotor::checkError(){
    updateCurrentState1();
    uint16_t error = *((uint16_t*)(current_state1 + 6));
    return error>0 ? true : false;

}

void RMDmotor::showCurrentState1()
{
    if (updateCurrentState1()){
        Serial.print("cmd: ");
        Serial.println(current_state1[0], HEX);
        Serial.print("current temeperature: ");
        Serial.println(*((int8_t *)(current_state1 + 1)));
        Serial.print("current voltage: ");
        Serial.println(*((uint16_t *)(current_state1 + 4)));
        Serial.print("currunt error state: ");
        Serial.println(*((uint16_t *)(current_state1 + 6)), HEX);
        Serial.print("brake release: ");
        Serial.println(current_state1[3], HEX);
    }
}

void RMDmotor::showMotionState()
{
    calculate_motionstate();
    Serial.print("deg : ");
    Serial.println(motion_state.deg);
    Serial.print("vel: ");
    Serial.println(motion_state.vel);
    Serial.print("tor : ");
    Serial.println(motion_state.tor);
}

void RMDmotor::showCurrentState2()
{
    if (updateCurrentState2())
    {
        Serial.print("current degree: ");
        Serial.println(current_state2.degree);
        Serial.print("current iq: ");
        Serial.println(current_state2.iq);
        Serial.print("current speed: ");
        Serial.println(current_state2.speed);
        Serial.print("current temperature: ");
        Serial.println(current_state2.temperature);
    }
}