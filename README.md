# modbus_benchmark (ros node)
 - ModbusTCPを使用したModbusサーバとの通信にかかるオーバーヘッドを計測するためのノードです
 - start_register から開始し、loop_limitまで1つずつアドレスを移動しながら loop_limit 回計測をします
 - 書込は1アドレスごとに行いますが、読込は num_bit_register, num_holding_registerに値を設定することで同時読込の時間を計測することができます(上限あり)

## Environment 
 - ROS1 (kinetic / melodic / noetic)
 - ubuntu (16.04 / 18.04 / 20.04)
## Install Libraly
  `sudo apt install -y libmodbus-dev`  
  [Documents](https://libmodbus.org/documentation/)
## Build
 `catkin_make -C ~/catkin_ws --pkg modbus_benchmark`


## Run
 default ( ip: 127.0.0.1 port:502 )  
  `roslaunch modbus_benchmark modbus_bench.launch`
 
 costom  
  `roslaunch modbus_benchmark modbus_bench.launch ip_adrs:=127.0.0.1 port:=502`  

## Params
|Name|Dafault|Discription|
|:--|--:|:--|
|ip_adrs|127.0.0.1|Modbus server IP address|
|port|502|Modbus server port|
|loop_limit|100|Number of times to repeat reading or writing for average calculation|
|start_register|4000|The first address to read / write, loop_limit, is incremented by 1.<br>When using the actual modbus server, please stop whether the written address is safe.
|num_bit_register|1|Number to be read at the same time when reading bits(<1000)|
|num_holding_rebister|1|Number to be read at the same time when reading holding registers (<125)|
