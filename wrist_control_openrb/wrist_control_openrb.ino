#include <Dynamixel2Arduino.h>
#include <math.h>

//OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial

#define NDOF 2
#define DX_BAUDRATE 115200

#define RAD_TO_CNT 651.8986469044033

//This namespace is required to use Control table item names
using namespace ControlTableItem;

const int DXL_DIR_PIN = -1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = { 1, 2 };

// Goal Position = 116 (4 byte)
const uint16_t SW_START_ADDR = 116;
const uint16_t SW_ADDR_LEN = 4;

typedef struct sw_data {
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;


sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];


// Function prototypes
void calc_inverse_kinematics(float n[3], float angles[][2]);
float saturate(float in, float range[2]);
float deg2rad(float deg);
double f_map(double x, double in_min, double in_max, double out_min, double out_max);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(DX_BAUDRATE);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT; i++) {

    // Get DYNAMIXEL information
    dxl.ping(DXL_ID_LIST[i]);

    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
    dxl.torqueOn(DXL_ID_LIST[i]);

    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_LIST[i], 500);

    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t *)&sw_data[i].goal_position;
    sw_infos.xel_count++;

    sw_data[i].goal_position = 2048;
  }
  sw_infos.is_info_changed = true;
  delay(1000);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
float n_array[4][3] = {
  {0.30901738, -0.29389264, 0.90450836},
  {-0.30901661, -0.29389272, 0.90450860},
  {-0.30901661, 0.29389261, 0.90450863},
  {0.30901738, 0.29389254, 0.90450839}
};

int cnt = 0;
int idx = 0;

void loop() {
  // put your main code here, to run repeatedly:
  float motor_angles[DXL_ID_CNT] = {0, 0};
  calc_inverse_kinematics(n_array[idx], &motor_angles);
  
  
  for (int i = 0; i < DXL_ID_CNT; i++) {
    int32_t tmp = 2048 + (int32_t)(RAD_TO_CNT * motor_angles[i]);

    //DEBUG_SERIAL.println(tmp);

    //sw_data[i].goal_position = tmp;
  }
  //DEBUG_SERIAL.println();


  sw_infos.is_info_changed = true;
  if (dxl.syncWrite(&sw_infos) != true) {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }

  // update index
  if(!(++cnt % 200)){
    idx = (++idx) % 4; 
    DEBUG_SERIAL.print("IK results: ");
    DEBUG_SERIAL.println((int32_t)(RAD_TO_CNT * motor_angles[0]));
    DEBUG_SERIAL.println((int32_t)(RAD_TO_CNT * motor_angles[1]));

    DEBUG_SERIAL.println("idx: ");
    DEBUG_SERIAL.println(idx);
    DEBUG_SERIAL.println();
  }
  delay(10);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

float angle_limit[2][2] = {{-deg2rad(35), deg2rad(35)},{-deg2rad(40), deg2rad(40)}};

void calc_inverse_kinematics(float n[3], float angles[][2]){
  // Normalize n
  float norm = sqrt(n[0]* n[0] + n[1]* n[1] + n[2]* n[2]);
  float n_norm[3];

  for(int i=0;i<3;i++){
    n_norm[i] = n[i] / norm;
  }

  // Calculate kinematics
  float ang_x, ang_y, theta1, theta2;
  float gear_ratio = 15 / 9.0;

  ang_y = asin(n_norm[0]);
  ang_x = asin(-n_norm[1]/cos(ang_y));

  theta1 = ang_x * gear_ratio;
  theta2 = ang_y * gear_ratio;

  (*angles)[0] = saturate(theta1, angle_limit[0]);
  (*angles)[1] = saturate(theta2, angle_limit[1]);

  return;
}

float saturate(float in, float range[2]){
  float out;

  if(in < range[0]){
    out = range[0];
  }
  else if(in > range[1]){
    out = range[1];
  }
  else out = in;

  return out;
}

float deg2rad(float deg){
  return deg * PI / 180.0;
}

double f_map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
