#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_ DEFINITIONS_H_

#include <cstdint>

#define LOG_FILE_NAME "OFLogFile.json"
#define SENSOR_SERVER_PORT  8080                                         //original 5555
#define SENSOR_SERVER_IP_ADDRESS "127.0.0.1"

#define OBJECTS_RECEIVED_MSG  "ObjectsReceived"
#define OBJECTS_RECEIVED_MSG_SIZE 15

// Measurement noise is constant and provided by sensor supplier

static constexpr float Rx = 0.3f;
static constexpr float Ry = 0.3f;
static constexpr float Rvx = 0.1f;
static constexpr float Rvy = 0.1f;

// Those parameters define the measurement variance
// To covariance is supposed to be zero
static constexpr float Rxx = Rx * Rx;
static constexpr float Ryy = Ry * Ry;
static constexpr float Rvxvx = Rvx * Rvx;
static constexpr float Rvyvy = Rvy * Rvy;

// Process noise
static constexpr float Qx = 1.0f;
static constexpr float Qy = 1.0f;
static constexpr float Qvx = 0.3f;
static constexpr float Qvy = 0.3f;

// Variance of the process noise
static constexpr float Qxx = Qx * Qx;
static constexpr float Qyy = Qy * Qy;
static constexpr float Qvxvx = Qvx * Qvx;
static constexpr float Qvyvy = Qvy * Qvy;

static constexpr int MAX_NUM_OF_OBJECTS = 10;
static constexpr int MAX_NUM_OF_SENSOR_OBJECTS = 10;

struct SensorObject {
  float x;  //[m]
  float y;  //[m]
  float vx; //[m/s]
  float vy; //[m/s]

  SensorObject() : x(0.f), y(0.f), vx(0.f), vy(0.f) {}
  SensorObject(const float x, const float y, const float vx, const float vy)
      : x(x), y(y), vx(vx), vy(vy) {}
};

struct SensorObjectList {
  SensorObjectList() : timestamp(0), numOfValidObjects(0) {}

  SensorObject objectList[MAX_NUM_OF_SENSOR_OBJECTS];
  uint8_t numOfValidObjects;
  uint64_t timestamp; // Microseconds
};

struct Object {
  uint16_t objectId;
  float x;     //[m] x position
  float y;     //[m] y position
  float vx;    // [m/s] velocity in x direction
  float vy;    // [m/s] velocity in y direction
  float Pxx;   // variance of x position
  float Pyy;   // variance of y position
  float Pvxvx; // variance of x velocity
  float Pvyvy; // variance of y velocity
  float Pxvx;  // covariance of x position and velocity
  float Pyvy;  // covariance of y position and velocity

  Object()
      : x(0.f), y(0.f), vx(0.f), vy(0.f), Pxx(0.f), Pyy(0.f), Pvxvx(0.f),
        Pvyvy(0.f), Pxvx(0.f), Pyvy(0.f), objectId(0) {}
};

struct ObjectList {
  Object objects[MAX_NUM_OF_OBJECTS];
  uint8_t numOfValidObjects;
  uint64_t timestamp;
  ObjectList() : numOfValidObjects(0), timestamp(0) {}
};

#endif
