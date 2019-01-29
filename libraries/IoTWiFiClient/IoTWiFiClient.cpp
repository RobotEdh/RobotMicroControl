#include <IoTWiFiClient.h>

const char* host = "192.168.0.18";        // Server IP
const int   port = 8080;                  // Server Port

WiFiClient tcpClient;
IOTSerialClass IOTSerial;

void IoTWiFiClientClass::ESPblink(void)
{
  // blink the resquested led during 6s
  for (int i=0;i<5;i++){
        digitalWrite(5, HIGH);  // turn on led
        delay(500);
        digitalWrite(5, LOW);  // turn off led
        delay(500);  
  }          
}

int IoTWiFiClientClass::IoTWCbegin()
{
    char ssid[31];
    char password[64];
    int ret = SUCCESS;

    //Serial.println("Begin IoTWCbegin");
        
    pinMode(5, OUTPUT); 
    digitalWrite(5, HIGH); // Led on  

    // We start by connecting to a WiFi network
    ret =  get_credentials(ssid, password);
    if (ret == 0) {
       //Serial.print ("ssid: ");Serial.println (ssid);
       //Serial.print ("password: ");Serial.println (password);   
    }
    else  {  
       //Serial.print ("Error get credentials: ");Serial.println (ret);
       return ret;
    }

    //Serial.print("Connecting to ");
    //Serial.println(ssid);
             
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      //Serial.print(".");
    }
 
    //Serial.println("");
    //Serial.println("WiFi connected");  
    //Serial.println("IP address: ");
    //Serial.println(WiFi.localIP());
    
    // MAC address 
    byte mac[6]; 
    WiFi.macAddress(mac);
    /*Serial.print("MAC: ");
    Serial.print(mac[0],HEX);
    Serial.print(":");
    Serial.print(mac[1],HEX);
    Serial.print(":");
    Serial.print(mac[2],HEX);
    Serial.print(":");
    Serial.print(mac[3],HEX);
    Serial.print(":");
    Serial.print(mac[4],HEX);
    Serial.print(":");
    Serial.println(mac[5],HEX);
    */
    ret = IOTSerial.IOTSbegin(0); // Serial port
    if (ret != 0) {
       //Serial.print ("Error IOTSbegin: ");Serial.println (ret);
       return ret;
    }
    
    digitalWrite(5, LOW); // Led off
    
    return SUCCESS;                   
}


int IoTWiFiClientClass::IoTWCSendInfos (char *infos)
{
  //Serial.println("Begin IoTWCSendInfos");
  String url = "/robotInfos.php/";
  
  //Serial.print("connecting to ");
  //Serial.println(host); 
  
  if (!tcpClient.connect(host, port)) {
     //Serial.println("connection failed");
     return (-1);
  }
  //Serial.println("is connected to server");

  //Serial.println(infos);
  digitalWrite(5, HIGH);    // Led on 
  tcpClient.print(String("POST ") + url + " HTTP/1.1\r\n" +
                         "Host: " + host + "\r\n" +
                         "Connection: close\r\n" +
                         "Content-Type: application/x-www-form-urlencoded\r\n" +
                         "Content-length: " + strlen(infos) + "\r\n\r\n"
  );
  
  tcpClient.println(infos);
  tcpClient.println(); 
  tcpClient.stop();
  digitalWrite(5, LOW); // Led off    

  return SUCCESS;              
} 


int IoTWiFiClientClass::IoTWCSendPicture (int n, uint16_t size)
{
  char filename[12+1];
  char start_request[256];

  // Open the file
  sprintf(filename, "PICT%d.jpg", n);

  // Prepare HTTP POST
  sprintf(filename, "PICT%d.jpg", n);
  sprintf(start_request, "\n--AaB03x\nContent-Disposition: form-data; name=\"picture\"; filename=%s\nContent-Type: image/jpeg\nContent-Transfer-Encoding: binary\n\n", filename);
  char end_request[]    = "\n--AaB03x--\n";
  
  String url = "/robotPicture.php/";
  uint16_t len = size + strlen(start_request) + strlen(end_request);
     
  if (!tcpClient.connect(host, port)) {
     //Serial.println("connection failed");
     return (-1);
  }
  
  tcpClient.print(String("POST ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Content-Type: multipart/form-data; boundary=AaB03x\r\n" +
               "Content-Length: " + len + "\r\n\r\n"
               );
               
  tcpClient.print( start_request);
       
  digitalWrite(5, HIGH);    // Led on 
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
  digitalWrite(5, LOW); // Led off  
  
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

    //Serial.println("Begin IoTWCReceive");
    digitalWrite(5, HIGH);    // Led on 
    ret = IOTSerial.IOTSread(0, msg, &msg_len, 10000UL);  // timeout 10s
    //Serial.print("Call IOTSread, ret: "); Serial.print(ret); Serial.print(", msg_len: "); Serial.println((int)msg_len);
    digitalWrite(5, LOW);    // Led off 
    
    if (ret != SUCCESS) {
       //Serial.println("error IOTSread");
       return ret;  
    }
  
    IOTSerial.IOTSgetTags(msg, tag, value, &nbtags); // parse the response  
    //Serial.print("Call IOTSgetTags, nbtags: "); Serial.println((int)nbtags);
    
    if (nbtags < 1)          return -1; 
    
    //Serial.print("tag[0]: "); Serial.println((int)tag[0]);  
    if (tag[0]!= TAG_CMDID)   return -2;
        
    //Serial.print("tag[1]: "); Serial.println((int)tag[1]);
    
    if (tag[1]== TAG_PICTURE)
    {
     *type = PICTURE;
     *n = (int)value[2];
     *size = value[3];
         
     //Serial.println("Call IOTSsend with RESP_OK");    
     ret = IOTSerial.IOTSsend(0, RESP_OK);
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
    else
    {    
       //Serial.println("error IOTSread, TAG unknown");
       return -4;
    }
             
    
    //Serial.println("End IoTWCReceive");
    return SUCCESS;  
}
