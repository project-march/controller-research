#include "odrive_interface/odrive.hpp"

#define ODRIVE_OK 0;
#define ODRIVE_ERROR 1;

using namespace std;

Odrive::Odrive(const std::string& serial_number, const std::string& axis_number)
{
  this->serial_number = serial_number;
  this->axis_number = axis_number;

  // Open USB connection
  if (this->odrive_endpoint_.open_connection(serial_number))
  {
    ROS_ERROR("Odrive %s error opening USB connection", serial_number.c_str());
  }

  // Read JSON from target
  if (getJson())
  {
    ROS_ERROR("Odrive %s error getting JSON", serial_number.c_str());
  }
}

Odrive::~Odrive()
{
  this->odrive_endpoint_.remove();
}

int Odrive::getJson()
{
  commBuffer rx;
  commBuffer tx;

  int len;
  int address = 0;

  string json;

  do
  {
    this->odrive_endpoint_.endpointRequest(0, rx, len, tx, true, 512, true, address);
    address = address + len;
    json.append((const char*)&rx[0], (size_t)len);
  } while (len > 0);

  Json::Reader reader;
  bool res = reader.parse(json, this->odrive_json_);

  if (!res)
  {
    return ODRIVE_ERROR;
  }
  return ODRIVE_OK;
}

int main()
{
  ROS_INFO("Starting");
}
