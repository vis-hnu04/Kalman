#include "Definitions.h"
#include <iostream>
#include <math.h>
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define WINDOWS_LEAN_AND_MEAN
#include <winsock2.h>
#define M_PI 3.1415f
#define socklen_t int
#else
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

static const float VELOCITY = 15.0f; // m/s
static const float RADIUS1 = 50.0f;  // m
static const float RADIUS2 = 50.0f;  // m

SensorObject so1(0.0, RADIUS1, -VELOCITY, 0.0);

SensorObject so2(0.0, RADIUS2, VELOCITY, 0.0);

std::normal_distribution<float> normDistX(0, Rxx);
std::normal_distribution<float> normDistY(0, Ryy);
std::normal_distribution<float> normDistVx(0, Rvxvx);
std::normal_distribution<float> normDistVy(0, Rvyvy);

std::random_device rd;
std::mt19937 gen(rd());

void moveObjectOnCircle(SensorObject &so, const bool turnLeft,
                        const float radius,
                        const uint64_t deltaTimeMicroSeconds) {
  float distMoved = VELOCITY * (float)deltaTimeMicroSeconds /
                    1000000.f; // Approx. length of circular arc
  float angle_deg = distMoved / radius;
  float angle_rad = angle_deg / (2 * M_PI);

  float objectHeading = atan2(so.vy, so.vx);

  float objectHeadingNew{0};

  if (turnLeft)
    objectHeadingNew = objectHeading + angle_rad;
  else
    objectHeadingNew = objectHeading - angle_rad;

  so.vx = cos(objectHeadingNew) * VELOCITY;
  so.vy = sin(objectHeadingNew) * VELOCITY;

  so.x = cos(objectHeadingNew) * radius;
  so.y = sin(objectHeadingNew) * radius;

  so.x += normDistX(gen);
  so.y += normDistY(gen);
  so.vx += normDistVx(gen);
  so.vy += normDistVy(gen);
  std::cout << so.x << " " << so.y << " " << so.vx << " " << so.vy << std::endl;
}

void doExit(int code) {
#ifdef _WIN32
  WSACleanup();
#endif
  exit(code);
}

int main(int argc, char const *argv[]) {
  int server_fd{0};
  int client{0};
  struct sockaddr_in address;
  int opt{1};
  int addrlen{sizeof(address)};

#ifdef _WIN32
  WSADATA wsaData;
  if (WSAStartup(MAKEWORD(2, 2), &wsaData) != NO_ERROR) {
    perror("WSAStartup");
    doExit(EXIT_FAILURE);
  }
#endif

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port 8080
  int sockfd = setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR,
                          (const char *)&opt, sizeof(opt));
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt,
                 sizeof(opt))) {
    perror("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(SENSOR_SERVER_PORT);

  // Forcefully attaching socket to the port 8080
  if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  if (listen(server_fd, 3) < 0) {
    perror("listen");
    doExit(EXIT_FAILURE);
  }

  std::cout << "Listening for incoming connections..." << std::endl;

  char objectsReceivedMsg[OBJECTS_RECEIVED_MSG_SIZE];

  if ((client = accept(server_fd, (struct sockaddr *)&address,
                       (socklen_t *)&addrlen)) > 0) {
    std::cout << "Client connected!" << std::endl;

    ////////////////////////////////////////

    uint64_t timestamp = 0;

    for (int i = 0; i < 100; ++i) {
      moveObjectOnCircle(so1, false, RADIUS1, 400000);
      moveObjectOnCircle(so2, true, RADIUS2, 400000);

      SensorObjectList sol;
      sol.numOfValidObjects = 2;
      timestamp += 400000;
      sol.timestamp = timestamp;
      sol.objectList[0] = so1;
      sol.objectList[1] = so2;

     
      send(client, (char *)&sol, sizeof(SensorObjectList), 0);
      std::cout<<timestamp<<std::endl;
      std::cout<<i<<std::endl;
      recv(client, objectsReceivedMsg, OBJECTS_RECEIVED_MSG_SIZE, 0);
    }

    perror("accept");
    doExit(EXIT_FAILURE);
  }

  doExit(0);
}
