#ifndef JSON_FILE_LOGGER_H_
#define JSON_FILE_LOGGER_H_ JSON_FILE_LOGGER_H_
#include "Definitions.h"
#include <fstream>
#include <vector>

class JSONFileLogger {
public:
  JSONFileLogger(char *fileName);

  /*
   *Write 'name:[' to the file
   */
  void openNewFusionCycleArray();
  /*
   *Close the current array by adding ']' to the file.
   */
  void closeFusionCycleArray();
  /*
   *Initialization of the json struct
   *by adding a '{' to the file.
   */
  void init();

  void addObjectList(const ObjectList &list, char *name);

  void addSensorObjectList(const SensorObjectList &list);

  /*
   *Use this function to add a single json element, e.g., timestamp data.
   */
  template <typename T> void addElement(const T elem, char *name);

  /*You can use this method to add association information. Here,
   *the first element of the pair represents the index of the object
   *available in your object track database while the second element
   *refers to the index of the sensor object.
   */
  template <typename ObjectIntType, typename SensorObjectIntType>
  void addAssociationIndices(
      const std::vector<std::pair<ObjectIntType, SensorObjectIntType>>
          associationPairs);

  /*
   *This function finishes creating the JSON struct by adding
   *the last '}' which corresponds to the bracket of the init
   *and closes the file.
   */
  void close();

protected:
  std::ofstream _logger;
  // If false, add a ',' before inserting a new array
  bool _firstArray;
  // If false, add a ',' before inserting a new element.
  bool _firstArrayElement;

  uint32_t _fusionCycleCounter;
};

template <typename T>
void JSONFileLogger::addElement(const T elem, char *name) {
  if (!_firstArrayElement) {
    _logger << ","
            << "\n";
  } else {
    _firstArrayElement = false;
  }

  _logger << "\"" << name << "\":" << elem << "\n";
}

template <typename ObjectIntType, typename SensorObjectIntType>
void JSONFileLogger::addAssociationIndices(
    const std::vector<std::pair<ObjectIntType, SensorObjectIntType>>
        associationPairs) {

  if (!_firstArrayElement) {
    _logger << ","
            << "\n";
  } else {
    _firstArrayElement = false;
  }

  _logger << " \"Associations\": [\n";

  for (int i{0}; i < associationPairs.size(); ++i) {
    _logger << "{\n";
    _logger << "\"ObjectIndex\""
            << ":" << associationPairs[i].first << ",\n";
    _logger << "\"SensorObjectIndex\""
            << ":" << associationPairs[i].second << "\n";

    if (i < associationPairs.size() - 1) {
      _logger << "},\n";
    } else {
      _logger << "}\n";
    }
  }

  _logger << "]\n";
}

#endif
