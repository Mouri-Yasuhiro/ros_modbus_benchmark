# modbus_benchmark

## Build
 `catkin_make -C ~/catkin_ws --pkg modbus_test`

## Run
 default ( ip: 127.0.0.1 port:502 )  
  `roslaunch modbus_test modbus_bench.launch`
 
 costom  
  `roslaunch modbus_test modbus_bench.launch ip_adrs:=127.0.0.1 port:=502`  

## Params
|Name|Dafault|Discription|
|:--|--:|:--|
|ip_adrs|127.0.0.1|Modbus server IP address|
|port|502|Modbus server port|
|loop_limit|100|Number of times to repeat reading or writing for average calculation|
|start_register|4000|The first address to read / write, loop_limit, is incremented by 1.|
|num_bit_register|1|Number to be read at the same time when reading bits(<1000)|
|num_holding_rebister|1|Number to be read at the same time when reading holding registers (<125)|
