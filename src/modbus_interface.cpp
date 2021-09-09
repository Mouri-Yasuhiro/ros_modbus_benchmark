#include <modbus_benchmark/modbus_interface.hpp>

ModbusInterface::ModbusInterface(std::string ip_adrs, int port, int slave_id, int loop_limit, int start_register, int num_bit_register, int num_holding_register)
  :loop_limit_(loop_limit), start_register_(start_register), num_bit_register_(num_bit_register), num_holding_register_(num_holding_register)
{
  nh_ = ros::NodeHandle();
  tryConnect(ip_adrs, port, slave_id);
  if(num_bit_register_ > MODBUS_MAX_READ_BITS)
    num_bit_register_ = MODBUS_MAX_READ_BITS;
  if(num_holding_register_ > MODBUS_MAX_READ_REGISTERS)
    num_holding_register_ = MODBUS_MAX_READ_REGISTERS;

  ROS_INFO("%s, %d, %d, %d, %d, %d", ip_adrs.c_str(), port, loop_limit, start_register, num_bit_register, num_holding_register);
}
ModbusInterface::~ModbusInterface(){
  modbus_close(md_);
  modbus_free(md_);
}

void ModbusInterface::tryConnect(std::string ip_adrs, int port, int slave_id){
  md_ = modbus_new_tcp(ip_adrs.c_str(),port);
  modbus_set_slave(md_, slave_id);
  modbus_connect(md_);
}

void ModbusInterface::runBenchmark(BenchmarkType type){
  
  uint8_t data_8[MODBUS_MAX_READ_BITS];
  uint16_t data_16[MODBUS_MAX_READ_REGISTERS];
  ros::Time start_time;
  switch (type)
  {
  case WRITE_BIT:
    ROS_INFO("WITE_BIT");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      modbus_write_bit(md_, start_register_+i, true);
    }
    break;
  case WRITE_REGISTER:
    ROS_INFO("WITE_REGISTER");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      modbus_write_register(md_, start_register_+i, USHRT_MAX);
    }
    break;
  case READ_BIT: 
    ROS_INFO("READ_BIT");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      modbus_read_bits(md_, start_register_+i, num_bit_register_, data_8);
    }
    break;
  case READ_REGISTER:
    ROS_INFO("READ_REGISTER");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      modbus_read_registers(md_, start_register_+i, num_holding_register_, data_16);
    }
    break;
  case READ_INPUT_BIT:
    ROS_INFO("READ_INPUT_BIT");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      modbus_read_input_bits(md_, start_register_+i, num_bit_register_, data_8);
    }
    break;
  case READ_INPUT_REGISTER:
    ROS_INFO("READ_INPUT_REGISTER");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      modbus_read_input_registers(md_, start_register_+i, num_holding_register_, data_16);
    }
    break;
  default:
    break;
  }

  ros::Time end_time = ros::Time::now();
  ROS_INFO("time, %f, loop, %d, ave, %f", (end_time - start_time).toSec(), loop_limit_, (end_time - start_time).toSec()/(double)loop_limit_);
}

void ModbusInterface::runBenchmarkUseMultiThread(BenchmarkType type){
  
  uint8_t data_8;
  uint16_t data_16;
  ros::Time start_time;

  std::vector<std::thread> threads;

  switch (type)
  {
  case WRITE_BIT:
    ROS_INFO("WITE_BIT");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      threads.emplace_back(std::thread([this, i]{modbus_write_bit(md_, start_register_+i, true);}));
    }
    for(int i=0;i<loop_limit_;i++){
      threads.at(i).join();
    }
    break;
  case WRITE_REGISTER:
    ROS_INFO("WITE_REGISTER");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      threads.emplace_back(std::thread([this, i]{modbus_write_register(md_, start_register_+i, USHRT_MAX);}));
    }
    for(int i=0;i<loop_limit_;i++){
      threads.at(i).join();
    }
    break;
  case READ_BIT: 
    ROS_INFO("READ_BIT");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      threads.emplace_back(std::thread([this, i, &data_8]{modbus_read_bits(md_, start_register_+i, 1, &data_8);}));
    }
    for(int i=0;i<loop_limit_;i++){
      threads.at(i).join();
    }
    break;
  case READ_REGISTER:
    ROS_INFO("READ_REGISTER");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      threads.emplace_back(std::thread([this, i, &data_16]{modbus_read_registers(md_, start_register_+i, 1, &data_16);}));
    }
    for(int i=0;i<loop_limit_;i++){
      threads.at(i).join();
    }
    break;
  case READ_INPUT_BIT:
    ROS_INFO("READ_INPUT_BIT");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      threads.emplace_back(std::thread([this, i, &data_8]{modbus_read_input_bits(md_, start_register_+i, 1, &data_8);}));
    }
    for(int i=0;i<loop_limit_;i++){
      threads.at(i).join();
    }
    break;
  case READ_INPUT_REGISTER:
    ROS_INFO("READ_INPUT_REGISTER");
    start_time = ros::Time::now();
    for(int i=0;i<loop_limit_;i++){
      threads.emplace_back(std::thread([this, i, &data_16]{modbus_read_input_registers(md_, start_register_+i, 1, &data_16);}));
    }
    for(int i=0;i<loop_limit_;i++){
      threads.at(i).join();
    }
    break;
  default:
    break;
  }

  ros::Time end_time = ros::Time::now();
  ROS_INFO("time, %f, loop, %d, ave, %f", (end_time - start_time).toSec(), loop_limit_, (end_time - start_time).toSec()/(double)loop_limit_);
}