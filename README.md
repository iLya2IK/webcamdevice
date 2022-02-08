# HTTP2 Web Camera Client Device
The client is designed to work in conjunction with the [REST Web Camera server](https://github.com/iLya2IK/wcwebcamserver).

Electronic schematic for current project 

![alt text](https://github.com/iLya2IK/webcamdevice/blob/main/webcamdevicesample.png?raw=true)

# Sub-protocol description
Data exchange between devices is carried out according to the HTTP/2 protocol using the POST method. The contents of requests and responses are JSON objects. The description for JSON requests/respones inside sub-protocol you can found [here](https://github.com/iLya2IK/wcwebcamserver/wiki).

# Copyrights and contributions
* [ESP-Camera - Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD](https://github.com/espressif/esp32-camera)
* SCCB (I2C like) driver - Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
