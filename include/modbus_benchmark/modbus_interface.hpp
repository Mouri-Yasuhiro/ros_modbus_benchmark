#ifndef MODBUS_BENCHMARK_MODBUS_INTERFACE_HPP
#define MODBUS_BENCHMARK_MODBUS_INTERFACE_HPP
#pragma once

#include <ros/ros.h>
#include <thread>
#include <vector>
#include <modbus/modbus.h>

enum BenchmarkType{
  WRITE_BIT,
  WRITE_REGISTER,
  READ_BIT,
  READ_REGISTER,
  READ_INPUT_BIT,
  READ_INPUT_REGISTER,
  End
};

class ModbusInterface{
public:
  ModbusInterface(std::string ip_adrs, int port, int slave_id, int loop_limit, int start_register, int num_bit_register, int num_holding_register);
  ~ModbusInterface();

  void runBenchmark(BenchmarkType type);
  void runBenchmarkUseMultiThread(BenchmarkType type);

private:
  void tryConnect(std::string ip_adrs, int port, int slave_id);

  ros::NodeHandle nh_;
  modbus_t *md_;

  int loop_limit_;
  int start_register_;
  int num_bit_register_;
  int num_holding_register_;
};

#endif