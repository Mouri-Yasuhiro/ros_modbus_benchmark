#include <modbus_benchmark/modbus_interface.hpp>

int main(int argc, char* argv[]){
  ros::init(argc, argv, "modbus_benchmark");
  ros::NodeHandle nh_private = ros::NodeHandle("~");

  std::string ip_adrs="127.0.0.1";
  int port = 502;
  int slave_id = 1;
  int loop_limit = 100;
  int start_register = 4000;
  int num_bit_register = 1;
  int num_holding_register = 1;


  nh_private.getParam("ip_adrs",ip_adrs);
  nh_private.getParam("port",port);
  nh_private.getParam("loop_limit",loop_limit);
  nh_private.getParam("start_register",start_register);
  nh_private.getParam("num_bit_register",num_bit_register);
  nh_private.getParam("num_holding_register",num_holding_register);

  ModbusInterface mod_if(ip_adrs, port, slave_id, loop_limit, start_register, num_bit_register, num_holding_register);

  std::vector<std::thread> threads;
  for(int i=0;i<(int)End;++i){
    mod_if.runBenchmark((BenchmarkType)i);
  }
  return 0;
}