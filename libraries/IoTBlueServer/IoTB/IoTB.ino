#include <IoTBlueServer.h>

IoTBlueServerClass IoTBlueServer;

void setup()
{   
    IoTBlueServer.IoTBSbegin(); 
}

void loop() {
    IoTBlueServer.IoTBSmain(); 
}