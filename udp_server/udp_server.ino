#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define in1 D1
#define in2 D2
#define in3 D3
#define in4 D5
#define ena D6
#define enb D7

const unsigned int SERVER_PORT  = 9999;
const char *SERVER_NAME         = "NODEMCU";
const char *SERVER_PASWD        = "";
const int SERVER_CHANNEL        = 4;

IPAddress AP_SERVER(192, 168, 4, 1);

typedef struct _udp_packet {
  int pwm_l;
  int pwm_r;
  bool forward;
  bool stop_car;
} UDP_PACKET;

WiFiUDP Udp;
UDP_PACKET packet;

bool stop_car = false;
bool forward = true;
int pwm_l = 0;
int pwm_r = 0;

void setup() {
  Serial.begin(115200);
  Serial.println();

  WiFi.disconnect();
  WiFi.enableAP(false);
  WiFi.softAP(SERVER_NAME, SERVER_PASWD, SERVER_CHANNEL);
  delay(1000);

  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("Starting connection to server...");
  Udp.begin(SERVER_PORT);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
//    Serial.print("Received packet of size ");
//    Serial.println(packetSize);
//    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
//    Serial.print(remoteIp);
//    Serial.print(", port ");
//    Serial.println(Udp.remotePort());

    int len = Udp.read((byte *)&packet, sizeof(UDP_PACKET));

//    Serial.print("Contents: ");
//    Serial.print("id = ");
//    Serial.print(packet.id);
//    Serial.print(", local = ");
//    Serial.print(packet.local);
//    Serial.print(", distance = ");
//    Serial.print(packet.distance);
//    Serial.print(", enabled = ");
//    Serial.println(packet.enabled);
      Serial.print("Left pwm: ");
      Serial.print(packet.pwm_l);
      Serial.print("Right pwm: ");
      Serial.print(packet.pwm_r);
      Serial.print("Forward: ");
      Serial.print(packet.forward);
      Serial.print("Stop Car: ");
      Serial.print(packet.stop_car);
      Serial.println("");
      pwm_l = packet.pwm_l;
      pwm_r = packet.pwm_r;
      forward = packet.forward;
      stop_car = packet.stop_car;
      if(!stop_car){
        if(forward){
          digitalWrite(in1, 1);
          digitalWrite(in2, 0);
          digitalWrite(in3, 0);
          digitalWrite(in4, 1);
        }
        else{
          digitalWrite(in1, 0);
          digitalWrite(in2, 1);
          digitalWrite(in3, 1);
          digitalWrite(in4, 0);
        }
        analogWrite(ena, pwm_l);
        analogWrite(enb, pwm_r);
      }
  }
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(SERVER_NAME);

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
