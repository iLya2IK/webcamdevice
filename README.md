# HTTP2 Web Camera Client Device
The client is designed to work in conjunction with the [REST Web Camera server](https://github.com/iLya2IK/wcwebcamserver).

Electronic schematic for current project

![alt text](https://github.com/iLya2IK/webcamdevice/blob/main/webcamdevicesample.png?raw=true)

# Sub-protocol description
Data exchange between devices is carried out according to the HTTP/2 protocol using the POST method. The contents of requests and responses are JSON objects. The description for JSON requests/respones inside sub-protocol you can found [here](https://github.com/iLya2IK/wcwebcamserver/wiki).

## Messages supported by this module

What is message and how to send/get messages you can read [here](https://github.com/iLya2IK/wcwebcamserver/wiki).

### Common params

* _**mid**_ - integer - a unique identifier for your message. The device sends this _**mid**_ back to indicate which message was answered. You can ignore this parameter if you do not need to control the message flow.

### To do snapshot with camera

Request

```json
{"msg":"dosnap","params":{"mid":22}}
```

Response

```json
{"msg":"dosnap","params":{"mid":22,"result":"OK"}}
```

### To get adc voltage value from IO15 (mV)

Request

```json
{"msg":"getadcval","params":{"mid":12}}
```

Response

```json
{"msg":"adcval","params":{"mid":12,"adcval":1000}}
```

### To switch logical level for a pin (IO2|IO14)

Request

```json
{"msg":"output","params":{"mid":0,"pin":2,"level":1}}
```

Response

```json
{"msg":"output","params":{"mid":0,"result":"OK|BAD"}}
```

### Device button event (IO12|IO13)

Message from device

```json
{"msg":"btnevent","params":{"btn":"btn1|btn2"}}
```

# Copyrights and contributions
* [ESP-Camera - Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD](https://github.com/espressif/esp32-camera)
* SCCB (I2C like) driver - Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
