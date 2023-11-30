set(COMPONENT_SRCS "webcamdevice.c"                   
                   "button.c"                    
                   "cam_hal.c"
                   "esp_camera.c"                   
                   "ll_cam.c"
                   "ov2640.c"
                   "sccb.c"
                   "sensor.c"                   
                   "xclk.c")
                   
set(COMPONENT_ADD_INCLUDEDIRS ".;./include")

register_component()
