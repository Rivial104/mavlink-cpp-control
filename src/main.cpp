#include <Arduino.h>
#include <common/mavlink.h>

// Obsluga mavlink
void MavRequestData();
void sendHeartbeat();
void handleMessage(mavlink_message_t* msg);
void receiveMavlink();

// Podstawowe sterowanie FC
void mavSetMode(int flightMode);
void armMotors(bool arm);
void setAltitude(float altitude);
void setMotorSpeed(int motor_id, int speed);
void setSpeed(float vx, float vy, float vz);
// void setYaw(float yaw_angle);

// Loty automatyczne i testy
void automat();
void testAutomat();
void ascendToAltitude(float target_altitude, float climb_rate);

// Reszta funkcji
void clearSerialBuffer();
void readFromRaspi();
void convertLocalToGPS(double lat, double lon, double x, double y, double& new_lat, double& new_lon); 


// Deklaracja pinów portów szeregowych
HardwareSerial mavlinkSerial(PA3, PA2); // RX TX
HardwareSerial Serial3(PB7, PB6); // RX TX

unsigned long int previousMillis = 0L;
unsigned long int INTERVAL = 1000L;
unsigned long modeChangeMillis = 0L;
const unsigned long MODE_CHANGE_INTERVAL = 10000L; // 10 seconds

// Konfiguracja MAVLINK
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
uint8_t system_mode = MAV_MODE_PREFLIGHT;
uint32_t custom_mode = 0;
uint8_t system_state = MAV_STATE_STANDBY;
uint8_t sysid = 255;
uint8_t compid = 2;
uint8_t type = MAV_TYPE_QUADROTOR;
uint8_t target_system = 1;
uint8_t target_component = 0;

// Reszta zmiennych konfiguracyjnych oraz stałych
String receivedData = "";
float alt = 0.0; 
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;
const double EARTH_RADIUS = 6378137.0; 
double current_lat = 0.0; 
double current_lon = 0.0; 

// Definicje masek
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_ROLL_RATE 0x01
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_PITCH_RATE 0x02
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_YAW_RATE 0x04
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_THRUST 0x08

void setup() {
  Serial.begin(57600);
  mavlinkSerial.begin(57600);
  Serial3.begin(57600, SERIAL_8N1);

  pinMode(PC13, OUTPUT);
  clearSerialBuffer();
  delay(5000);
}

void loop() {

  mavSetMode(0);
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= INTERVAL) {
    previousMillis = currentMillis;


    
    // ascendToAltitude(6.0, 1.0); // Wznoś się na wysokość 10 metrów z prędkością 1 

    // automat();
    testAutomat();

      MavRequestData();
      receiveMavlink();
  }
}


//######################################################################//
//                PODSTAWOWE FUNKCJE DO STEROWANIA DRONEM
//######################################################################//

// Funkcja wysyłająca polecenie do ustawiania predkosci drona
void setSpeed(float vx, float vy, float vz) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Wypełnienie wiadomości MAVLink
  mavlink_msg_set_position_target_local_ned_pack(
      sysid,
      compid,
      &msg,
      0, // czas w milisekundach od rozpoczęcia misji
      target_system,
      target_component,
      MAV_FRAME_LOCAL_NED,
      0b0000111111000111, // Maska ignorująca wszystkie inne ustawienia oprócz prędkości
      0, 0, 0,            // x, y, z (nie używane)
      vx, vy, -vz,  // prędkości w kierunku x, y, z
      0, 0, 0,            // przyspieszenia w kierunku x, y, z (nie używane)
      0, 0                // yaw, yaw_rate (nie używane)
  );

  // Serializacja wiadomości do bufora
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Wysłanie wiadomości przez port szeregowy
  mavlinkSerial.write(buf, len);
}

void mavSetMode(int flightMode) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_set_mode_pack(sysid, compid, &msg, 1, 209, flightMode);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
  Serial.println("Tryb lotu zmieniony");
}

void armMotors(bool arm) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    sysid, compid, &msg, target_system, target_component, MAV_CMD_COMPONENT_ARM_DISARM, 0, arm, 0, 0, 0, 0, 0, 0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
  Serial.println("Motors armed");
}

void setAltitude(float altitude) {
  mavlink_message_t msg;
  mavlink_msg_set_position_target_global_int_pack(sysid, compid, &msg, millis(), target_system, target_component, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                   0b0000111111111000, 0, 0, altitude , 0, 0, 0, 0, 0, 0, 0, 0);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
}

void setMotorSpeed(int motor_id, int speed) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Assuming motor_id starts from 1 for the first motor
    mavlink_msg_rc_channels_override_pack(sysid, compid, &msg, target_system, target_component,
                                        motor_id == 1 ? speed : UINT16_MAX,
                                        motor_id == 2 ? speed : UINT16_MAX,
                                        motor_id == 3 ? speed : UINT16_MAX,
                                        motor_id == 4 ? speed : UINT16_MAX,
                                        UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, // Remaining channels set to neutral (UINT16_MAX)
                                        UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
  Serial.print("Set motor ");
  Serial.print(motor_id);
  Serial.print(" speed to ");
  Serial.println(speed);
}

void flyToGPS(double lat, double lon, float alt, float stayTime) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Ustawienie waypoint
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_system, target_component,
                                MAV_CMD_NAV_WAYPOINT, stayTime, 0, 0, 0, 0, lat, lon, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);

  Serial.print("Flying to Latitude: ");
  Serial.print(lat, 8);
  Serial.print(" Longitude: ");
  Serial.print(lon, 8);
  Serial.print(" Altitude: ");
  Serial.println(alt, 2);
}

void setYaw(float yaw_angle) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Przetworzenie kąta na radiany
  float yaw = radians(yaw_angle);

  // Ustawienia bitowe dla maski ignorującej inne ustawienia
  uint16_t type_mask = MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_ROLL_RATE |
                       MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_PITCH_RATE |
                       MAVLINK_MSG_SET_ATTITUDE_TARGET_IGNORE_THRUST;

  // Kąty w radianach
  float q[4];
  mavlink_euler_to_quaternion(0, 0, yaw, q);

  // Wysyłanie wiadomości MAVLink SET_ATTITUDE_TARGET
  mavlink_msg_set_attitude_target_pack(sysid, compid, &msg, millis() / 1000, target_system, target_component, type_mask, q, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
  Serial.print("Yaw angle set to: ");
  Serial.println(yaw_angle);
}

void mavlink_euler_to_quaternion(float roll, float pitch, float yaw, float quaternion[4]) {
  float cosRoll = cos(roll * 0.5);
  float sinRoll = sin(roll * 0.5);
  float cosPitch = cos(pitch * 0.5);
  float sinPitch = sin(pitch * 0.5);
  float cosYaw = cos(yaw * 0.5);
  float sinYaw = sin(yaw * 0.5);

  quaternion[0] = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  quaternion[1] = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  quaternion[2] = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  quaternion[3] = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
}

//######################################################################//
//                          AUTOMATYCZNY LOT I TESTY
//######################################################################//

void testAutomat() {
  // Ustaw tryb Loiter (np. mode 5)
  mavSetMode(4);
  delay(1000);

  // Uzbrój drona
  armMotors(true);
  delay(1000);

  // // Przyspiesz silniki do wartości 1400 PWM (zakładając zakres 1000-2000)
  // for (int i = 1; i <= 4; ++i) {
  //   setMotorSpeed(i, 1200); // 1400 to przykład 20% zakresu PWM
  // }
  // delay(1000);

  // for (int i = 1; i <= 4; ++i) {
  //   setMotorSpeed(i, 1300); // 1400 to przykład 20% zakresu PWM
  // }
  // delay(1000);

  // for (int i = 1; i <= 4; ++i) {
  //   setMotorSpeed(i, 1400); // 1400 to przykład 20% zakresu PWM
  // }
  // delay(1000);

  // Ustaw wysokość na 10 metrów
  setAltitude(10.0);
  delay(5000); // Poczekaj na osiągnięcie wysokości

  // ascendToAltitude(10.0,2.0);
  // delay(4000);

  // // Ustaw wysokość na 15 metrów
  // setAltitude(15.0);
  // delay(5000); // Poczekaj na osiągnięcie wysokości

  // Włącz tryb RTL lub Land (np. mode 6)
  mavSetMode(6);
  delay(15000);

  // Rozbrój drona po lądowaniu
  armMotors(false);
}

void automat() {
  mavSetMode(4); // Mav mode guided disarmed
  delay(1000);

  armMotors(1);
  delay(1000);

  // Set motor speeds to 20% (assuming PWM range is 1000-2000)
  for (int i = 1; i <= 3; ++i) {
    setMotorSpeed(i, 1400); // 1200 is 20% of the 1000-2000 range
  }
  delay(2000);

  // Land the drone
  mavSetMode(6); // Mode 9 is typically RTL (Return to Launch) or you can use 6 for LAND
  delay(2000);

  // Disarm the drone after landing
  armMotors(0);
  while (true); // Stop execution
}

void ascendToAltitude(float target_altitude, float climb_rate) {
  while (alt < target_altitude) {
    setSpeed(0.0,0.0,1.0);
    delay(1000); // Czekaj chwilę przed kolejną aktualizacją
  }
  // Po osiągnięciu docelowej wysokości, ustaw tryb utrzymywania wysokości
  setAltitude(target_altitude);
}


//######################################################################//
//                       OBSŁUGA WIADOMOŚCI MAVLINK                     
//######################################################################//

void MavRequestData() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Żądanie wiadomości o wysokości (ID 74)
    mavlink_msg_request_data_stream_pack(sysid, compid, &msg, target_system, target_component, MAV_DATA_STREAM_POSITION, 5, 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlinkSerial.write(buf, len);
    delay(100); // krótka przerwa między żądaniami

    // Żądanie wiadomości o kątach położenia przestrzennego (ID 30, 33)
    mavlink_msg_request_data_stream_pack(sysid, compid, &msg, target_system, target_component, MAV_DATA_STREAM_EXTRA1, 5, 1);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlinkSerial.write(buf, len);
    delay(100);
}

void receiveMavlink() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (mavlinkSerial.available() > 0) {
    uint8_t c = mavlinkSerial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Sprawdź, czy wiadomość ma ID 74, 30, lub 33
      if (msg.msgid == 74 || msg.msgid == 30 || msg.msgid == 33) {
        Serial.print("Received message with ID: ");
        Serial.println(msg.msgid);
        handleMessage(&msg);
      }
    }
  }
}

void handleMessage(mavlink_message_t* msg) {
  switch (msg->msgid) {
    case 74: { // MAVLINK_MSG_ID_ALTITUDE
        mavlink_altitude_t altitude;
        mavlink_msg_altitude_decode(msg, &altitude);
        float alt = altitude.altitude_local; // Wartość w metrach
        Serial.print("Current Altitude: ");
        Serial.println(alt, 4);
      }
      break;

    case 30: { // MAVLINK_MSG_ID_ATTITUDE
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(msg, &attitude);
        float roll = attitude.roll * 10.0; // Skalowanie o 10
        float pitch = attitude.pitch * 10.0; // Skalowanie o 10
        float yaw = attitude.yaw * 10.0; // Skalowanie o 10
        
        // Formatowanie i drukowanie danych
        Serial.print("Roll: ");
        Serial.print(roll, 2);
        Serial.println("°");

        Serial.print("Pitch: ");
        Serial.print(pitch, 2);
        Serial.println("°");

        Serial.print("Yaw: ");
        Serial.print(yaw, 2);
        Serial.println("°");
      }
      break;

    case 33: { // MAVLINK_MSG_ID_GLOBAL_POSITION_INT
        mavlink_global_position_int_t global_position;
        mavlink_msg_global_position_int_decode(msg, &global_position);
        // Przekonwertowane na readable
        // float lat = global_position.lat/10000000; // Wartość w milimetrach
        // float lon = global_position.lon/10000000;  // Wartość w milimetrach
        float lat = global_position.lat; // Wartość w milimetrach
        float lon = global_position.lon;  // Wartość w milimetrach
        Serial.print("Latitude: ");
        Serial.println(lat);
        Serial.print("Longitude: ");
        Serial.println(lon);
      }
      break;
  }
}

void sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  mavlinkSerial.write(buf, len);
  Serial.println("Heartbeat sent");
}

//######################################################################//
//                     OBSŁUGA KOMUNIKACJI Z RASPI                    
//######################################################################//

void readFromRaspi() {
  if (Serial3.available() > 0) {
    String command = Serial3.readStringUntil('@');
    String commandType = command.substring(0, 3);
    String parameters = command.substring(4);

    switch (commandType[0]) {
      case 'D': // DST
        {
          float x, y, z;
          sscanf(parameters.c_str(), "%f, %f, %f", &x, &y, &z);
          double new_lat, new_lon;
          convertLocalToGPS(current_lat, current_lon, x, y, new_lat, new_lon);
          setAltitude(z); // Ustaw wysokość
          // Można dodać więcej kodu do nawigacji GPS
          // np. flyToGPS(new_lat, new_lon, z);
        }
        break;
      
      case 'V': // VEL
        {
          float vx, vy, vz;
          sscanf(parameters.c_str(), "%f, %f, %f", &vx, &vy, &vz);
          setSpeed(vx, vy, vz);
        }
        break;
      
      case 'R': // ROT
        {
          float yaw_angle;
          sscanf(parameters.c_str(), "%f", &yaw_angle);
          setYaw(yaw_angle);
        }
        break;
      
      case 'L': // LND
        {
          mavSetMode(6); // Change flightmode to LAND
        }
        break;
      
      case 'S': // STR
        {
          armMotors(true);
        }
        break;

      default:
        Serial3.print("UNK@");
        break;
    }

    // Sending ACK to confirm command reception
    Serial3.print("ACK@");
  }
}

//######################################################################//
//                             RESZTA FUNKCJI
//######################################################################//

void clearSerialBuffer() {
  while (mavlinkSerial.available() > 0) {
    mavlinkSerial.read();
  }
}

void readFromRaspi(){

  if (Serial3.available()) {
          String dataFromSerial3 = Serial3.readStringUntil('\n');
          Serial.print("Odebrano na Serial3: ");
          Serial.println(dataFromSerial3);

          if (dataFromSerial3 == "arm") {
                armMotors(1);
          }

          if (dataFromSerial3 == "darm") {
                armMotors(0);
          }

          if (dataFromSerial3 == "flt4") {
                mavSetMode(4);
          }

          if (dataFromSerial3 == "flt0") {
                mavSetMode(0);
          }

          if (dataFromSerial3 == "land") {
                mavSetMode(6);
          }

          // Wysłanie danych z powrotem na Serial2
          Serial3.println(dataFromSerial3);
      }
}

void convertLocalToGPS(double lat, double lon, double x, double y, double& new_lat, double& new_lon) {
  // Konwersja lokalnych współrzędnych na zmiany szerokości i długości geograficznej
  double dLat = y / EARTH_RADIUS;
  double dLon = x / (EARTH_RADIUS * cos(M_PI * lat / 180.0));

  // Nowa szerokość i długość geograficzna
  new_lat = lat + (dLat * 180.0 / M_PI);
  new_lon = lon + (dLon * 180.0 / M_PI);
}