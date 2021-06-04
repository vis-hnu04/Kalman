#include "JSONFileLogger.h"

JSONFileLogger::JSONFileLogger(char *fileName)
    : _firstArray(true), _firstArrayElement(true), _fusionCycleCounter(0),
      _logger(fileName, std::ofstream::out) {}

void JSONFileLogger::init() {
  _firstArray = true;
  _firstArrayElement = true;
  _fusionCycleCounter = 0;
  _logger << "{\n";
}

void JSONFileLogger::close() {
  _logger << "}\n";
  _firstArray = true;
  _logger.close();
}

void JSONFileLogger::openNewFusionCycleArray() {
  if (!_firstArray) {
    _logger << ",\n";
  } else {
    _firstArray = false;
  }

  _logger << "\"" <<"  "<< _fusionCycleCounter++ << "\":"
          << "{\n";
}

void JSONFileLogger::closeFusionCycleArray() {
  _logger << "}\n";
  _firstArrayElement = true;
}

void JSONFileLogger::addSensorObjectList(const SensorObjectList &list) {
  if (!list.numOfValidObjects) {
    return;
  }

  if (!_firstArrayElement) {
    _logger << ",\n";

  } else {
    _firstArrayElement = false;
  }

  _logger << "\"SensorObjectList\": [\n";

  for (uint8_t i{0}; i < list.numOfValidObjects; ++i) {
    _logger << "{\n";
    _logger << "\"x\": " << list.objectList[i].x << ",\n";
    _logger << "\"y\": " << list.objectList[i].y << ",\n";
    _logger << "\"vx\": " << list.objectList[i].vx << ",\n";
    _logger << "\"vy\": " << list.objectList[i].vy << "\n";

    if (i < list.numOfValidObjects - 1) {
      _logger << "},\n";
    } else {
      _logger << "}\n";
    }
  }

  _logger << "]\n";
}

void JSONFileLogger::addObjectList(const ObjectList &list, char *name) {
  if (!list.numOfValidObjects) {
    return;
  }

  if (!_firstArrayElement) {
    _logger << ",\n";

  } else {
    _firstArrayElement = false;
  }

  _logger << " \"" << name << "\": [\n";

  for (uint8_t i{0}; i < list.numOfValidObjects; ++i) {
    _logger << "{\n";
    _logger << "\"id\": " << list.objects[i].objectId << ",\n";
    _logger << "\"x\": " << list.objects[i].x << ",\n";
    _logger << "\"y\": " << list.objects[i].y << ",\n";
    _logger << "\"vx\": " << list.objects[i].vx << ",\n";
    _logger << "\"vy\": " << list.objects[i].vy << ",\n";
    _logger << "\"Pxx\": " << list.objects[i].Pxx << ",\n";
    _logger << "\"Pyy\": " << list.objects[i].Pyy << ",\n";
    _logger << "\"Pvxvx\": " << list.objects[i].Pvxvx << ",\n";
    _logger << "\"Pvyvy\": " << list.objects[i].Pvyvy << ",\n";
    _logger << "\"Pxvx\": " << list.objects[i].Pxvx << ",\n";
    _logger << "\"Pyvy\": " << list.objects[i].Pyvy << "\n";

    if (i < list.numOfValidObjects - 1) {
      _logger << "},\n";
    } else {
      _logger << "}\n";
    }
  }

  _logger << "]\n";
}
