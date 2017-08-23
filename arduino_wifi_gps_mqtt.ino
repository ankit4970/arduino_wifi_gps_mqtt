#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspUdp.h>
#include "SoftwareSerial.h"
#include <PubSubClient.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <Adafruit_FONA.h>


#define FONA_RX             11      // RX wire is plugged into pin 11 on the Arduino
#define FONA_TX             10      // TX wire is plugged into pin 10 on the Arduino  
#define FONA_RST            4       // RST wire is plugged into pin 4 on the Arduino 
#define ONE_WIRE_BUS        5       // Data wire is plugged into pin 2 on the Arduino

//IPAddress server(10, 0, 0, 22);
const char* broker = "iot.eclipse.org";
const char* topic = "cmpe297/sensor";
char ssid[] = "Lenovo";                // your network SSID (name)
char pass[] = "12345678";             // your network password
int status = WL_IDLE_STATUS;            // the Wifi radio's status

// DS18B20 1 wire temperature sensor
OneWire oneWire(ONE_WIRE_BUS);  
DallasTemperature sensors(&oneWire);

// Adafruit FONA SIM808 GPS
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// ESP8266 WiFi
WiFiEspClient espClient;
PubSubClient client(espClient);
SoftwareSerial soft(2,3);           // RX, TX
char temp[50];

float latitude=37.336082, longitude= -121.882572, speed_kph, heading, speed_mph, altitude,temperature=25.36;
/*******************************************************************************************************    
*   readTemperature 
*******************************************************************************************************/
void readTemperature()
{
    sensors.requestTemperatures();                      // Send the command to get temperature readings 
    //Serial.println("DONE"); 
    Serial.print("Temperature is: ");
    temperature = sensors.getTempCByIndex(0);           // 0 refers to the first IC on the wire
    Serial.print(temperature);
    Serial.println();  
    delay(100); 
}

/*******************************************************************************************************    
*   readGPS 
*******************************************************************************************************/
bool readGPS()
{
    // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
    boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude); 
    if (gps_success) 
    { 
        Serial.print("GPS lat:");
        Serial.println(latitude, 6);
        Serial.print("GPS long:");
        Serial.println(longitude, 6);
        Serial.print("GPS speed KPH:");
        Serial.println(speed_kph);
        Serial.print("GPS speed MPH:");
        speed_mph = speed_kph * 0.621371192;
        Serial.println(speed_mph);
        Serial.print("GPS heading:");
        Serial.println(heading);
        Serial.print("GPS altitude:");
        Serial.println(altitude);
    } 
    else 
    {
        Serial.println("Waiting for FONA GPS 3D fix...");
        return false;
    }

     return true; 
}

/*******************************************************************************************************    
*   sendSensorData 
*******************************************************************************************************/
bool sendSensorData()
{
    // Use mqtt to send data to a subscribed topic
    String lat_str,lon_str,long_str,temp_str,datetime_str;
    lat_str = String(latitude,6);
    lon_str = String(longitude,7);
    temp_str = String(temperature,2);
    long_str = lat_str+","+lon_str+","+temp_str;
    long_str.toCharArray(temp, long_str.length() + 1);
   
    return (client.publish(topic, temp));     
}

/*******************************************************************************************************    
*   setup 
*******************************************************************************************************/
void setup() 
{
    // initialize serial for debugging
    Serial.begin(9600);
#if 1
    // Initialize GPS Module
    fonaSerial->begin(9600);
    if (! fona.begin(*fonaSerial)) 
    {
        Serial.println(F("Couldn't find FONA"));
        while(1);
    }
    
    Serial.println(F("FONA is OK"));
    
    Serial.println(F("Enabling GPS..."));
    fona.enableGPS(true);
#endif
    
    // initialize serial for ESP module
    Serial.println(F("Initializingling ESP8266"));
    soft.begin(9600);

    Serial.println(F("Enabling DS18B20"));
    sensors.begin();

    // initialize ESP module
    WiFi.init(&soft);
    
    // check for the presence of the shield
    if (WiFi.status() == WL_NO_SHIELD) 
    {
        Serial.println("WiFi shield not present");
        // don't continue
        while (true);
    }
    
    // attempt to connect to WiFi network
    while ( status != WL_CONNECTED) 
    {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        // Connect to WPA/WPA2 network
        status = WiFi.begin(ssid, pass);
    }

    // you're connected now, so print out the data
    Serial.println("You're connected to the Wifi network");

    //connect to MQTT server
    client.setServer(broker, 1883);
    client.setCallback(callback);

    if(client.connect("sensorClient"))
    {
        Serial.println("Connection Successful");
    }
    else
    {
        Serial.print("failed, rc=");
        Serial.print(client.state());
    }

}

/*******************************************************************************************************    
*   setup 
*******************************************************************************************************/
void callback(char* topic, byte* payload, unsigned int length) 
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i=0;i<length;i++) 
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

/*******************************************************************************************************    
*   setup 
*******************************************************************************************************/
void loop() 
{
    // Read temperature from DS18B20
    readTemperature();

    // Read GPS coordinates from SIM808
    //readGPS();
    
    if (!client.connected()) 
    {
        reconnect();
    }
    client.loop();

    // Send sensor data(temperature+GPS) to server using mqtt publish
    sendSensorData();
}

/*******************************************************************************************************    
*   setup 
*******************************************************************************************************/
void reconnect() 
{
    // Loop until we're reconnected
    while (!client.connected()) 
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect, just a name to identify the client
        if (client.connect("sensorClient")) 
        {
            Serial.println("connected");
            // Once connected, publish an announcement...
            //client.publish(topic,"hello world");
        } 
        else 
        {
            //Serial.print("failed, rc=");
            //Serial.print(client.state());
            //Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}
