#include <IoTWiFiClient.h>

// Logging mode
#define  LOGSERIAL1
//#define LOGSDCARD  // log to SD Card
//#define LOGTRACE   // Enable trace
#include <log.h>
//File logFile;
                    // The loging class
const char* host = "192.168.0.18";        // Server IP
const int   port = 8080;                  // Server Port

WiFiClient tcpClient;
IOTSerialClass IOTSerial;

void IoTWiFiClientClass::ESPblink(int n)
{
  // blink the resquested led n times
  for (int i=0;i<n;i++){
        digitalWrite(PIN_LED, HIGH);  // turn on led
        delay(500);
        digitalWrite(PIN_LED, LOW);  // turn off led
        delay(500);  
  }          
}

int IoTWiFiClientClass::IoTWCbegin()
{
    char ssid[31];
    char password[64];
    int ret = SUCCESS;


    ret = IOTSerial.IOTSbegin(0); // Init Serial port
    if (ret != 0) {
       return ret;
    }
    Serial.pins(15,13);  // arduino is connected to these pins to avoid sending ROM logging at startup

    PRINTbegin  // start logging 
    PRINTs(" ")
    PRINTs("Starting")
    pinMode(PIN_LED, OUTPUT); 
    digitalWrite(PIN_LED, HIGH); // Led on  

    // We start by connecting to a WiFi network
    ret =  get_credentials(ssid, password);
    if (ret == 0) {
       PRINT("ssid: ",ssid)
       PRINT("password: ",password)  
    }
    else  {  
       PRINT("Error get credentials: ",ret)
       return ret;
    }

    PRINT("Connecting to ",ssid)
             
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      PRINTa(".")
    }
 
    PRINTs("")
    PRINTs("WiFi connected")  
    PRINT("IP address: ",WiFi.localIP())
    
    // MAC address 
    byte mac[6]; 
    WiFi.macAddress(mac);
    PRINT6("MAC: ",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5])
       
    digitalWrite(PIN_LED, LOW); // Led off
    PRINTs("Init OK")
    return SUCCESS;                   
}


void IoTWiFiClientClass::IoTWCend()
{
    IOTSerial.IOTSend(0);
    PRINTs("Go to sleep......")
    PRINTs(" ")
    delay(1000); // time to send the print
    ESP.deepSleep(0,WAKE_RF_DEFAULT); // deep sleep for ever until reset
}


int IoTWiFiClientClass::IoTWCSendInfos (char *infos)
{
  PRINTs("Begin IoTWCSendInfos")
  String url = "/robotInfos.php/";
  
  PRINT("Connecting to ",host)
  
  if (!tcpClient.connect(host, port)) {
     PRINTs("connection failed")
     return (-1);
  }
  PRINTs("is connected to server")

  PRINT("infos: ",infos)
  digitalWrite(PIN_LED, HIGH);    // Led on 
  tcpClient.print(String("POST ") + url + " HTTP/1.1\r\n" +
                         "Host: " + host + "\r\n" +
                         "Connection: close\r\n" +
                         "Content-Type: application/x-www-form-urlencoded\r\n" +
                         "Content-length: " + strlen(infos) + "\r\n\r\n"
  );
  
  tcpClient.println(infos);
  tcpClient.println(); 
  tcpClient.stop();
  digitalWrite(PIN_LED, LOW); // Led off    
  PRINTs("End IoTWCSendInfos")
  return SUCCESS;              
} 


int IoTWiFiClientClass::IoTWCSendPicture (int n, uint16_t size)
{
  char filename[12+1];
  char start_request[256];
  int ret = SUCCESS;

  PRINTs("Begin IoTWCSendPicture")
  PRINT("number: ",n)
  PRINT("size: ",size)
  
  // Prepare HTTP POST
  sprintf(filename, "PICT%d.jpg", n);
  sprintf(start_request, "\n--AaB03x\nContent-Disposition: form-data; name=\"picture\"; filename=%s\nContent-Type: image/jpeg\nContent-Transfer-Encoding: binary\n\n", filename);
  char end_request[]    = "\n--AaB03x--\n";
  
  String url = "/robotPicture.php/";
  uint16_t len = size + strlen(start_request) + strlen(end_request);
     
  if (!tcpClient.connect(host, port)) {
     PRINTs("connection failed")
     return (-1);
  }
  
  tcpClient.print(String("POST ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Content-Type: multipart/form-data; boundary=AaB03x\r\n" +
               "Content-Length: " + len + "\r\n\r\n"
               );
               
  tcpClient.print( start_request);
       
  digitalWrite(PIN_LED, HIGH);    // Led on 
  unsigned long previousTime;
  previousTime = millis();
  int incomingByte = 0;
  uint32_t count = 0;          
  while ((incomingByte >= 0) &&(count < size) && ( millis()< previousTime+ (30*1000))) {
      count++;  
      incomingByte = IOTSerial.IOTSRawread(0);
      if (incomingByte >= 0) tcpClient.write(incomingByte);
  }// while count
 
  tcpClient.print(end_request);
  tcpClient.println();
  tcpClient.stop();
  digitalWrite(PIN_LED, LOW); // Led off  
   
  PRINTs("Call IOTSsend with RESP_OK")    
  ret = IOTSerial.IOTSsend(0, RESP_OK);  // CANNOT use TX with Sparkfun Thing but CAN with Adafruit HUZZAH 
  PRINTs("End IoTWCSendPicture")
  return SUCCESS;     
}

int IoTWiFiClientClass::IoTWCReceive (char *infos, uint8_t *type, int *n, uint16_t *size)
{
    uint8_t msg[MSG_SIZE_MAX];
    uint8_t msg_len = 0;
    uint8_t tag [MAX_TAGS];
    uint16_t value [MAX_TAGS];
    uint8_t nbtags; 
    char buf[1024]= "";
    char szresp[10]= ""; 
    int ret = SUCCESS;
     
    PRINTs("Begin IoTWCReceive")
    digitalWrite(PIN_LED, HIGH);    // Led on 
    ret = IOTSerial.IOTSread(0, msg, &msg_len, 10000UL);  // timeout 10s
    PRINT("Call IOTSread, ret: ",ret);
    PRINT("msg_len: ",msg_len)
    digitalWrite(PIN_LED, LOW);    // Led off 
    
    if (ret != SUCCESS) {
       PRINTs("error IOTSread")
       return ret;  
    }
  
    IOTSerial.IOTSgetTags(msg, tag, value, &nbtags); // parse the response  
    PRINT("Call IOTSgetTags, nbtags: ",nbtags)
    
    if (nbtags < 1)          return -1; 
    
    PRINTx("tag[0]: ",tag[0])  
    if (tag[0]!= TAG_CMDID)   return -2;
        
    PRINTx("tag[1]: ",tag[1]);
    if (tag[1]== TAG_PICTURE)
    {
     *type = PICTURE;
     *n = (int)value[2];
     *size = value[3];
         
     PRINTs("Call IOTSsend with RESP_OK")    
     ret = IOTSerial.IOTSsend(0, RESP_OK);  // CANNOT use TX with Sparkfun Thing but CAN with Adafruit HUZZAH  
    }
    else if (tag[1]== TAG_INFOS)
    {  
       *type = INFOS;
       *n = 0;
       *size = 0;
       for(int j=0; j<value[1]; j++)  
       {
                    strcat(buf,szField[j]);
                    strcat(buf,"=");
                    sprintf(szresp, "%10d", (int)value[j+2]); // 2 tags to exclure before parameter
                    strcat(buf,szresp);
                    strcat(buf,"&");
       }
       sprintf(infos, buf);
    }
    else if (tag[1]== TAG_SLEEP)
    {
       *type = SLEEP;
       *n = 0;
       *size = 0;
    }
    else
    {    
       PRINTs("error IOTSread, TAG unknown")
       return -4;
    }
             
    
    PRINTs("End IoTWCReceive")
    return SUCCESS;  
}