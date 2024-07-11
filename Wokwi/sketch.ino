#include <DHT.h> // Include the DHT library for interfacing with the DHT sensor
#include <WiFi.h> // Include the WiFi library for connecting to a WiFi network
#include <PubSubClient.h> // Include the PubSubClient library for MQTT communication
#include <TinyGPS++.h> // Include the TinyGPS++ library for parsing GPS data
#include <Wire.h> // Include the Wire library for I2C communication
#include <MPU6050.h> // Include the MPU6050 library for interfacing with the MPU6050 sensor

// Pin connected to the DHT sensor
#define DHTPIN 15 // Define the pin connected to the DHT sensor - ADC (Analog-to-Digital Converter) 
#define DHTTYPE DHT22 // Define the type of DHT sensor

// WiFi credentials
const char* ssid = "Wokwi-GUEST"; // Define the WiFi SSID
const char* password = ""; // Define the WiFi password

// Ubidots credentials
const char* mqtt_server = "industrial.api.ubidots.com"; // Define the Ubidots MQTT server address
const char* mqtt_client_name = "ESP32Client"; // Define the MQTT client name
const char* token = "BBUS-2FkzRDZlFJN3RHkJ0VFzH9XqcMEkFH"; // Define the Ubidots token
const char* device_label = "smart-shoes"; // Define the device label
const char* variable_label_temp = "temperature"; // Define the temperature variable label
const char* variable_label_humi = "humidity"; // Define the humidity variable label
const char* variable_label_pressure = "pressure"; // Define the pressure variable label
const char* variable_label_location = "location"; // Define the pressure variable label
const char* variable_label_speed = "speed"; // Define the speed variable label
const char* variable_label_heart_rate = "heart_rate"; // Define the heart rate variable label
const char* variable_label_gait = "status"; // Define the gait variable label
const char* variable_label_battery = "battery"; // Define the battery variable label
const char* variable_label_date = "date"; // Define the date variable label
const char* variable_label_time = "time"; // Define the time variable label
const char* variable_label_altitude = "altitude"; // Define the altitude variable label
const char* variable_label_accel_x = "acceleration_x"; // Define the accelerometer X-axis variable label
const char* variable_label_accel_y = "acceleration_y"; // Define the accelerometer Y-axis variable label
const char* variable_label_accel_z = "acceleration_z"; // Define the accelerometer Z-axis variable label
const char* variable_label_gyro_x = "gyroscope_x"; // Define the gyroscope X-axis variable label
const char* variable_label_gyro_y = "gyroscope_y"; // Define the gyroscope Y-axis variable label
const char* variable_label_gyro_z = "gyroscope_z"; // Define the gyroscope Z-axis variable label

// DHT sensor instance
DHT dht(DHTPIN, DHTTYPE); // Create an instance of the DHT sensor

// WiFi and MQTT clients
WiFiClient espClient; // Create a WiFi client
PubSubClient client(espClient); // Create a MQTT client using the WiFi client

// Pressure simulation variables
int potPin = 34; // Define the pin connected to the potentiometer
int pressureValue; // Declare a variable to store pressure value
int potValue; // Declare a variable to store potentiometer value

enum GaitType { STOPPED, WALKING, RUNNING }; // Enumerate the gait types
GaitType currentGait = STOPPED; // Initialize the current gait type to stopped
String gaitString; // Declare a variable to store the gait string
int gaitNumber;

unsigned long gaitStartTime; // Declare a variable to store the start time of the gait
unsigned long currentTime; // Declare a variable to store the current time
unsigned long walkDuration = 900; // Define the duration of walking - seconds 
unsigned long runDuration = 300; // Define the duration of running - seconds

// GPS variables
TinyGPSPlus gps; // Create an instance of the TinyGPS++ library
HardwareSerial serialGPS(1); // Create a hardware serial instance for GPS communication
double latitude = 0.0; // Declare a variable to store latitude
double longitude = 0.0; // Declare a variable to store longitude
double speed = 0.0; // Declare a variable to store speed
double prev_latitude = 0.0; // For distance calculation
double prev_longitude = 0.0; // For distance calculation

// MPU6050 instance
MPU6050 mpu; // Create an instance of the MPU6050 sensor

// LED pins for indicating status
const int walkingLED = 2; // LED for walking status
const int runningLED = 4; // LED for running status
const int stoppedLED = 5; // LED for stopped status

// Distance initlization
double total_distance = 0.0; // Total distance traveled - in Kilometers
const char* variable_label_distance = "distance";

void setup_wifi() {
  delay(10); // Delay for stability
  Serial.println(); // Print a blank line
  Serial.print("Connecting to "); // Print connection message
  Serial.println(ssid); // Print SSID

  WiFi.begin(ssid, password); // Connect to WiFi network with SSID and password
  while (WiFi.status() != WL_CONNECTED) { // Wait until WiFi connection is established
    delay(500); // Delay for stability
  }
  Serial.println("WiFi connected"); // Print WiFi connection status
  Serial.print("IP address: "); // Print IP address label
  Serial.println(WiFi.localIP()); // Print local IP address
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection ..."); // Print MQTT connection attempt message
    if (client.connect(mqtt_client_name, token, "")) { // Attempt to connect to MQTT broker
      Serial.println("\nMQTT connected"); // Print connection success message
    } else {
      Serial.print("failed, rc="); // Print connection failure message
      Serial.print(client.state()); // Print MQTT client state
      Serial.println(" try again in 5 seconds"); // Print reconnection attempt message
      delay(5000); // Delay for reconnection attempt
    }
  }
}

void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate
  setup_wifi(); // Setup WiFi connection
  client.setServer(mqtt_server, 1883); // Set MQTT server and port
  dht.begin(); // Initialize DHT sensor
  gaitStartTime = millis(); // Record start time for pressure simulation
  
  // GPS initialization
  serialGPS.begin(9600, SERIAL_8N1, 16, 17); // Initialize GPS serial communication
  
  // MPU initialization
  Wire.begin(); // Initialize I2C communication
  Serial.println("Initializing MPU6050 ..."); // Print MPU6050 initialization message
  mpu.initialize(); // Initialize MPU6050
  if (mpu.testConnection()) { // Check MPU6050 connection
    Serial.println("MPU6050 connection successful"); // Print connection success message
  } else {
    Serial.println("MPU6050 connection failed"); // Print connection failure message
    while (1); // Loop indefinitely if connection fails
  }

  // Initialize LED pins
  pinMode(walkingLED, OUTPUT);
  pinMode(runningLED, OUTPUT);
  pinMode(stoppedLED, OUTPUT);

  Serial.println(F("ESP32 - GPS module Simulation")); // Print ESP32 and GPS module simulation message
}

// Haversine formula to calcluate distance
double haversine(double lat1, double lon1, double lat2, double lon2) {
    // Earth radius in kilometers
    const double R = 6371.0;

    // Convert degrees to radians
    lat1 = radians(lat1);
    lon1 = radians(lon1);
    lat2 = radians(lat2);
    lon2 = radians(lon2);

    // Differences
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Haversine formula
    double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // Distance in kilometers
    double distance = R * c;

    return distance;
}

void displayGPSData() {
  if (gps.location.isValid()) {
    Serial.print(F("Latitude: ")); // Print the latitude label
    Serial.println(gps.location.lat(), 6); // Print the latitude value with 6 decimal places
    Serial.print(F("Longitude: ")); // Print the longitude label
    Serial.println(gps.location.lng(), 6); // Print the longitude value with 6 decimal places

    // Create a JSON payload with the location data
    char payload_location[100];
    sprintf(payload_location, "{\"%s\": {\"value\": 1, \"context\": {\"lat\": %.6f, \"lng\": %.6f}}}",
            variable_label_location, gps.location.lat(), gps.location.lng());
    client.publish("/v1.6/devices/smart-shoes", payload_location); // Publish the location data to Ubidots

  } else {
    Serial.println(F("Location: INVALID")); // Print an error message if the location data is invalid
  }

  // Print altitude
  if (gps.altitude.isValid()) { // Check if altitude is valid
    Serial.print(F("Altitude: ")); // Print altitude label
    Serial.println(gps.altitude.meters()); // Print altitude in meters

    // Publish altitude to Ubidots
    char payload_alt[50]; // Declare a character array for altitude payload
    sprintf(payload_alt, "{\"%s\": %.2f}", variable_label_altitude, gps.altitude.meters()); // Format altitude payload
    client.publish("/v1.6/devices/smart-shoes", payload_alt); // Publish altitude payload to Ubidots
  } else {
    Serial.println(F("Altitude: INVALID")); // Print invalid altitude message
  }

  if (gps.speed.isValid()) {
    Serial.print(F("Speed: ")); // Print the speed label
    Serial.println(gps.speed.kmph()); // Print the speed value in km/h
    // Create a JSON payload with the speed data
    char payload_speed[50];
    sprintf(payload_speed, "{\"%s\": %.2f}", variable_label_speed, gps.speed.kmph());
    client.publish("/v1.6/devices/smart-shoes", payload_speed); // Publish the speed data to Ubidots
  } else {
    Serial.println(F("Speed: INVALID")); // Print an error message if the speed data is invalid
  }

// Read and publish date data
if (gps.date.isValid()) {
    int day = gps.date.day();
    int month = gps.date.month();
    int year = gps.date.year();

    // Log the date data
    Serial.print(F("Date: "));
    Serial.print(day);
    Serial.print(F("/"));
    Serial.print(month);
    Serial.print(F("/"));
    Serial.println(year);


     // Create a JSON payload with the time data in the context field
    char payload_date[200];
    sprintf(payload_date, "{\"%s\": {\"value\": 1, \"context\": {\"day\": %d, \"month\": %d, \"year\": %d}}}",
            variable_label_date, day, month, year);
    client.publish("/v1.6/devices/smart-shoes", payload_date); // Publish the time data to Ubidots
} else {
    Serial.println(F("Date: INVALID")); // Print an error message if the date data is invalid
}


// Read and publish time data
if (gps.time.isValid()) {
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();


    // Log the time data
    Serial.print(F("Time: "));
    Serial.print(hour);
    Serial.print(F(":"));
    Serial.print(minute);
    Serial.print(F(":"));
    Serial.println(second);

        // Create a JSON payload with the time data in the context field
    char payload_time[200];
    sprintf(payload_time, "{\"%s\": {\"value\": 1, \"context\": {\"hour\": %02d, \"minute\": %02d, \"second\": %02d}}}",
            variable_label_time, hour, minute, second);
    client.publish("/v1.6/devices/smart-shoes", payload_time); // Publish the time data to Ubidots
} else {
    Serial.println(F("Time: INVALID")); // Print an error message if the time data is invalid
}



  // Update distance calculation
  double current_latitude = gps.location.lat();
  double current_longitude = gps.location.lng();
  if (prev_latitude != 0.0 && prev_longitude != 0.0) {
    double distance = haversine(prev_latitude, prev_longitude, current_latitude, current_longitude); // Calculate the distance using the Haversine formula
    total_distance += distance; // Add the calculated distance to the total distance
  }
  prev_latitude = current_latitude; // Update previous latitude with current latitude
  prev_longitude = current_longitude; // Update previous longitude with current longitude
}

void updateGaitStatus() {
  if (currentGait == WALKING) {
    gaitString = "Walking"; // Set gait status to walking
    gaitNumber = 1; // Set gait number to 1 for walking
    digitalWrite(walkingLED, LOW); // Turn on the walking LED
    digitalWrite(runningLED, HIGH); // Turn off the running LED
    digitalWrite(stoppedLED, HIGH); // Turn off the stopped LED
  } else if (currentGait == RUNNING) {
    gaitString = "Running"; // Set gait status to running
    gaitNumber = 2; // Set gait number to 2 for running
    digitalWrite(runningLED, LOW); // Turn on the running LED
    digitalWrite(walkingLED, HIGH); // Turn off the walking LED
    digitalWrite(stoppedLED, HIGH); // Turn off the stopped LED
  } else if (currentGait == STOPPED) {
    gaitString = "Stopped"; // Set gait status to stopped
    gaitNumber = 0; // Set gait number to 0 for stopped
    digitalWrite(stoppedLED, LOW); // Turn on the stopped LED
    digitalWrite(walkingLED, HIGH); // Turn off the walking LED
    digitalWrite(runningLED, HIGH); // Turn off the running LED
  } else {
    gaitString = "Unknown"; // Set gait status to unknown
    gaitNumber = -1; // Set gait number to -1 for unknown status
    digitalWrite(stoppedLED, HIGH); // Turn off all LEDs
    digitalWrite(walkingLED, HIGH);
    digitalWrite(runningLED, HIGH);
  }
  Serial.println("\nCurrent Gait: " + gaitString); // Print the current gait status
}


void loop() {
  if (!client.connected()) {
    reconnect(); // Reconnect to MQTT broker if disconnected
  }
  client.loop(); // Maintain MQTT connection

  float h = dht.readHumidity(); // Read humidity from DHT sensor
  h += 10; // To obtain more real measurement
  float t = dht.readTemperature(); // Read temperature from DHT sensor
  t += 5; // To obtain more real measurement

  potValue = analogRead(potPin); // Read potentiometer value
  pressureValue = map(potValue, 0, 4095, 950, 1050); // Map potentiometer value to pressure value (assumed range)

  currentTime = millis(); // Get the current time

  // Update gait status
  if (currentGait == WALKING) {
    if (currentTime - gaitStartTime >= walkDuration) {
      currentGait = RUNNING; // Switch to running gait
      gaitStartTime = currentTime; // Reset gait start time
    } else {
      pressureValue += 8; // Increase pressure slightly when walking
    }
  } else if (currentGait == RUNNING) {
    if (currentTime - gaitStartTime >= runDuration) {
      currentGait = STOPPED; // Switch to stopped gait
      gaitStartTime = currentTime; // Reset gait start time
    } else {
      pressureValue += 30; // Increase pressure more when running
    }
  } else if (currentGait == STOPPED) {
    if (currentTime - gaitStartTime >= walkDuration) {
      currentGait = WALKING; // Switch to walking gait
      gaitStartTime = currentTime; // Reset gait start time
    } else {
      pressureValue -= 4; // Decrease pressure when stopped
    }
  }

  // Publish pressure data to Ubidots
  char payload_pressure[100];
  sprintf(payload_pressure, "{\"%s\": %d}", variable_label_pressure, pressureValue);
  client.publish("/v1.6/devices/smart-shoes", payload_pressure); 

  updateGaitStatus(); // Update LED status based on gait

  // Publish gait data to Ubidots
  char payload_gait[200];
  sprintf(payload_gait, "{\"%s\": {\"value\": %d, \"context\": {\"status\": \"%s\"}}}", variable_label_gait, gaitNumber, gaitString.c_str());
  client.publish("/v1.6/devices/smart-shoes", payload_gait); // Publish gait data to Ubidots

  // Publish temperature and humidity to Ubidots
  char payload_temp[50];
  sprintf(payload_temp, "{\"%s\": %.2f}", variable_label_temp, t);
  client.publish("/v1.6/devices/smart-shoes", payload_temp); // Publish temperature to Ubidots

  char payload_humi[50];
  sprintf(payload_humi, "{\"%s\": %.2f}", variable_label_humi, h);
  client.publish("/v1.6/devices/smart-shoes", payload_humi); // Publish humidity to Ubidots

  // Simulate and publish heart rate
  int heartRate;
  if (currentGait == WALKING) {
    heartRate = random(80, 100); // Simulate heart rate between 80 and 100 BPM when walking
  } else if (currentGait == RUNNING) {
    heartRate = random(100, 140); // Simulate heart rate between 100 and 120 BPM when running
  } else {
    heartRate = random(60, 80); // Simulate heart rate between 60 and 80 BPM when stopped
  }
  char payload_heart_rate[50];
  sprintf(payload_heart_rate, "{\"%s\": %d}", variable_label_heart_rate, heartRate);
  client.publish("/v1.6/devices/smart-shoes", payload_heart_rate); // Publish heart rate to Ubidots

  // Decrease battery level by 0.5 percent
  static int batteryLevel = 100; // Initialize battery level to 100
  batteryLevel -= 0.5; // Decrease battery level
  if (batteryLevel <= 0) {
    batteryLevel = 100; // Reset battery level to 100 when it reaches zero
  }
  // Publish battery level
  char payload_battery[50];
  sprintf(payload_battery, "{\"%s\": %d}", variable_label_battery, batteryLevel);
  client.publish("/v1.6/devices/smart-shoes", payload_battery); // Publish battery level to Ubidots

  // Add distance data to MQTT payload
  char payload_distance[100];
  sprintf(payload_distance, "{\"%s\": %.2f}", variable_label_distance, total_distance);
  client.publish("/v1.6/devices/smart-shoes", payload_distance); // Publish distance data to Ubidots

  // Log heartRate data
  Serial.print("Heart Rate: ");
  Serial.println(heartRate);

  // Log distance data
  Serial.print("Distance: ");
  Serial.println(total_distance);

  // Log Battery data
  Serial.print("Battery: ");
  Serial.println(batteryLevel);

  // Log Pressure data
  Serial.print("Pressure: ");
  Serial.println(pressureValue);

  // Log Humidity data
  Serial.print("Humidity: ");
  Serial.println(h);

  // Log Temperature data
  Serial.print("Temperature: ");
  Serial.println(t);

  // Read and display GPS data
  while (serialGPS.available() > 0) {
    gps.encode(serialGPS.read()); // Encode GPS data
  }
  displayGPSData(); // Display GPS data

  // Read gyroscope and accelerometer data
  int16_t accelX, accelY, accelZ;
  mpu.getAcceleration(&accelX, &accelY, &accelZ);

  int16_t gyroX, gyroY, gyroZ;
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);

  // Format and publish gyroscope data
  char payload_gyro[100];
  sprintf(payload_gyro, "{\"%s\": %d, \"%s\": %d, \"%s\": %d}", 
    variable_label_accel_x, accelX, 
    variable_label_accel_y, accelY, 
    variable_label_accel_z, accelZ);
  client.publish("/v1.6/devices/smart-shoes", payload_gyro);

  // Format and publish accelerometer data
  char payload_accel[100];
  sprintf(payload_accel, "{\"%s\": %d, \"%s\": %d, \"%s\": %d}", 
    variable_label_gyro_x, gyroX, 
    variable_label_gyro_y, gyroY, 
    variable_label_gyro_z, gyroZ);
  client.publish("/v1.6/devices/smart-shoes", payload_accel);
  

  delay(2000); // Delay for stability
}
