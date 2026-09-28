// ================================================================
//        V2V VEHICLE SAFETY SYSTEM - ESP32
// ================================================================
//
//  VEHICLE 1 -> #define VEHICLE_ID 1
//  VEHICLE 2 -> #define VEHICLE_ID 2
//
//  FEATURES:
//  - 4 IR obstacle sensors
//  - DHT11 temperature + humidity
//  - MPU6050 pitch + roll
//  - SH1106 128x64 OLED
//  - Green / Yellow / Red LEDs
//  - Buzzer
//  - ESP-NOW vehicle-to-vehicle communication
//  - Phone web dashboard
//
// ================================================================


// ================================================================
// LIBRARIES
// ================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <DHT.h>


// ================================================================
// VEHICLE ID
// ================================================================

// VEHICLE 1:
#define VEHICLE_ID 2

// For the SECOND ESP32 change the above line to:
//
// #define VEHICLE_ID 2
//
// DO NOT change anything else.


// ================================================================
// PIN CONNECTIONS
// ================================================================

// ---------- I2C ----------
#define SDA_PIN 21
#define SCL_PIN 22

// ---------- DHT11 ----------
#define DHT_PIN 4
#define DHT_TYPE DHT11

// ---------- IR SENSORS ----------
#define IR_FRONT 14
#define IR_RIGHT 27
#define IR_REAR  26
#define IR_LEFT  25

// ---------- LEDs ----------
#define GREEN_LED  18
#define YELLOW_LED 19
#define RED_LED    23

// ---------- BUZZER ----------
#define BUZZER 5


// ================================================================
// IR SENSOR LOGIC
// ================================================================
//
// Most IR obstacle sensors give:
// LOW  = obstacle detected
// HIGH = no obstacle
//
// If your sensors work opposite, change true to false.
//

#define IR_ACTIVE_LOW true


// ================================================================
// OLED - SH1106
// ================================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C

Adafruit_SH1106G display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);


// ================================================================
// SENSORS
// ================================================================

DHT dht(
  DHT_PIN,
  DHT_TYPE
);

Adafruit_MPU6050 mpu;


// ================================================================
// WEB SERVER
// ================================================================

WebServer server(80);


// ================================================================
// ESP-NOW
// ================================================================

#define V2V_CHANNEL 6

uint8_t broadcastAddress[] = {
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF
};


// ================================================================
// DATA SENT BETWEEN VEHICLES
// ================================================================

typedef struct
{
  uint8_t vehicleID;

  bool front;
  bool right;
  bool rear;
  bool left;

  float temperature;
  float humidity;

  float pitch;
  float roll;

  uint8_t sensorCount;

  uint8_t dangerLevel;

} VehicleData;


// Local vehicle data
VehicleData myData;

// Other vehicle data
VehicleData remoteData;

bool remoteAvailable = false;

unsigned long lastRemotePacket = 0;


// ================================================================
// TIMERS
// ================================================================

unsigned long lastSensorRead = 0;
unsigned long lastSend = 0;
unsigned long lastOLED = 0;

unsigned long lastLEDBlink = 0;
unsigned long lastBuzzer = 0;


// ================================================================
// ALERT STATES
// ================================================================

bool ledBlinkState = false;
bool buzzerState = false;

int previousSensorCount = -1;


// ================================================================
// OLED PAGE
// ================================================================

int oledPage = 0;


// ================================================================
// READ IR SENSOR
// ================================================================

bool readIR(int pin)
{
  int value = digitalRead(pin);

  if (IR_ACTIVE_LOW)
  {
    return (value == LOW);
  }
  else
  {
    return (value == HIGH);
  }
}


// ================================================================
// READ ALL SENSORS
// ================================================================

void readSensors()
{
  // ------------------------------------------------
  // VEHICLE ID
  // ------------------------------------------------

  myData.vehicleID = VEHICLE_ID;


  // ------------------------------------------------
  // IR SENSORS
  // ------------------------------------------------

  myData.front = readIR(IR_FRONT);

  myData.right = readIR(IR_RIGHT);

  myData.rear = readIR(IR_REAR);

  myData.left = readIR(IR_LEFT);


  // ------------------------------------------------
  // COUNT DETECTED SENSORS
  // ------------------------------------------------

  myData.sensorCount = 0;

  if (myData.front)
    myData.sensorCount++;

  if (myData.right)
    myData.sensorCount++;

  if (myData.rear)
    myData.sensorCount++;

  if (myData.left)
    myData.sensorCount++;


  // ------------------------------------------------
  // DHT11
  // ------------------------------------------------

  float temperature = dht.readTemperature();

  float humidity = dht.readHumidity();

  if (!isnan(temperature))
  {
    myData.temperature = temperature;
  }

  if (!isnan(humidity))
  {
    myData.humidity = humidity;
  }


  // ------------------------------------------------
  // MPU6050
  // ------------------------------------------------

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(
    &accel,
    &gyro,
    &temp
  );


  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;


  // Pitch
  myData.pitch =
    atan2(
      ax,
      sqrt(
        ay * ay +
        az * az
      )
    )
    * 180.0 / PI;


  // Roll
  myData.roll =
    atan2(
      ay,
      sqrt(
        ax * ax +
        az * az
      )
    )
    * 180.0 / PI;


  // ------------------------------------------------
  // DANGER LEVEL
  // ------------------------------------------------
  //
  // 0 sensors = level 0
  // 1 sensor  = level 1
  // 2 sensors = level 2
  // 3 sensors = level 3
  // 4 sensors = level 4
  //
  // High tilt also produces danger level 4.
  //

  myData.dangerLevel = myData.sensorCount;


  if (
    abs(myData.pitch) > 20 ||
    abs(myData.roll) > 20
  )
  {
    myData.dangerLevel = 4;
  }
}


// ================================================================
// ALERT SYSTEM
// ================================================================
//
// 0 SENSOR:
// GREEN ON
// BUZZER OFF
//
// 1 SENSOR:
// YELLOW BLINK
// BUZZER 500 ms
//
// 2 SENSORS:
// YELLOW ON
// RED BLINK
// BUZZER 300 ms
//
// 3 SENSORS:
// RED FAST BLINK
// BUZZER 100 ms
//
// 4 SENSORS:
// RED ON
// CONTINUOUS BUZZER
//
// ================================================================

void updateAlerts()
{
  int count = myData.sensorCount;

  unsigned long now = millis();


  // ------------------------------------------------
  // RESET TIMERS WHEN SENSOR COUNT CHANGES
  // ------------------------------------------------

  if (count != previousSensorCount)
  {
    previousSensorCount = count;

    lastLEDBlink = now;
    lastBuzzer = now;

    ledBlinkState = false;
    buzzerState = false;

    digitalWrite(
      GREEN_LED,
      LOW
    );

    digitalWrite(
      YELLOW_LED,
      LOW
    );

    digitalWrite(
      RED_LED,
      LOW
    );

    digitalWrite(
      BUZZER,
      LOW
    );
  }


  // =================================================
  // 0 SENSORS
  // =================================================

  if (count == 0)
  {
    digitalWrite(
      GREEN_LED,
      HIGH
    );

    digitalWrite(
      YELLOW_LED,
      LOW
    );

    digitalWrite(
      RED_LED,
      LOW
    );

    digitalWrite(
      BUZZER,
      LOW
    );

    return;
  }


  // =================================================
  // 1 SENSOR
  // =================================================
  //
  // YELLOW BLINK
  // BEEP EVERY 500 ms
  //

  if (count == 1)
  {
    digitalWrite(
      GREEN_LED,
      LOW
    );

    digitalWrite(
      RED_LED,
      LOW
    );


    // Yellow blinking
    if (
      now - lastLEDBlink >= 500
    )
    {
      lastLEDBlink = now;

      ledBlinkState =
        !ledBlinkState;

      digitalWrite(
        YELLOW_LED,
        ledBlinkState
      );
    }


    // Buzzer 500 ms
    if (
      now - lastBuzzer >= 500
    )
    {
      lastBuzzer = now;

      buzzerState =
        !buzzerState;

      digitalWrite(
        BUZZER,
        buzzerState
      );
    }

    return;
  }


  // =================================================
  // 2 SENSORS
  // =================================================
  //
  // YELLOW CONSTANT
  // RED BLINK
  // BEEP EVERY 300 ms
  //

  if (count == 2)
  {
    digitalWrite(
      GREEN_LED,
      LOW
    );

    digitalWrite(
      YELLOW_LED,
      HIGH
    );


    // Red blinking
    if (
      now - lastLEDBlink >= 300
    )
    {
      lastLEDBlink = now;

      ledBlinkState =
        !ledBlinkState;

      digitalWrite(
        RED_LED,
        ledBlinkState
      );
    }


    // Buzzer 300 ms
    if (
      now - lastBuzzer >= 300
    )
    {
      lastBuzzer = now;

      buzzerState =
        !buzzerState;

      digitalWrite(
        BUZZER,
        buzzerState
      );
    }

    return;
  }


  // =================================================
  // 3 SENSORS
  // =================================================
  //
  // RED FAST BLINK
  // BUZZER 100 ms
  //

  if (count == 3)
  {
    digitalWrite(
      GREEN_LED,
      LOW
    );

    digitalWrite(
      YELLOW_LED,
      LOW
    );


    // Very fast red blinking
    if (
      now - lastLEDBlink >= 100
    )
    {
      lastLEDBlink = now;

      ledBlinkState =
        !ledBlinkState;

      digitalWrite(
        RED_LED,
        ledBlinkState
      );
    }


    // Buzzer 100 ms
    if (
      now - lastBuzzer >= 100
    )
    {
      lastBuzzer = now;

      buzzerState =
        !buzzerState;

      digitalWrite(
        BUZZER,
        buzzerState
      );
    }

    return;
  }


  // =================================================
  // 4 SENSORS
  // =================================================
  //
  // RED CONSTANT
  // CONSTANT BUZZER
  //

  if (count >= 4)
  {
    digitalWrite(
      GREEN_LED,
      LOW
    );

    digitalWrite(
      YELLOW_LED,
      LOW
    );

    digitalWrite(
      RED_LED,
      HIGH
    );

    digitalWrite(
      BUZZER,
      HIGH
    );

    return;
  }
}


// ================================================================
// ESP-NOW RECEIVE CALLBACK
// ================================================================

void onDataReceive(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
)
{
  if (
    len != sizeof(VehicleData)
  )
  {
    return;
  }


  VehicleData receivedData;


  memcpy(
    &receivedData,
    incomingData,
    sizeof(receivedData)
  );


  // Ignore our own packet

  if (
    receivedData.vehicleID ==
    VEHICLE_ID
  )
  {
    return;
  }


  remoteData =
    receivedData;


  remoteAvailable = true;

  lastRemotePacket =
    millis();
}


// ================================================================
// SEND VEHICLE DATA
// ================================================================

void sendVehicleData()
{
  esp_err_t result =
    esp_now_send(
      broadcastAddress,
      (uint8_t *)&myData,
      sizeof(myData)
    );


  if (
    result != ESP_OK
  )
  {
    Serial.println(
      "ESP-NOW send failed"
    );
  }
}


// ================================================================
// OLED PAGE 1
// ================================================================

void showDirectionPage()
{
  display.clearDisplay();

  display.setTextColor(
    SH110X_WHITE
  );

  display.setTextSize(1);


  display.setCursor(
    0,
    0
  );

  display.print(
    "VEHICLE "
  );

  display.print(
    VEHICLE_ID
  );


  // Front
  display.setCursor(
    0,
    14
  );

  display.print(
    "FRONT: "
  );

  display.print(
    myData.front ?
    "OBSTACLE" :
    "CLEAR"
  );


  // Right
  display.setCursor(
    0,
    26
  );

  display.print(
    "RIGHT: "
  );

  display.print(
    myData.right ?
    "OBSTACLE" :
    "CLEAR"
  );


  // Rear
  display.setCursor(
    0,
    38
  );

  display.print(
    "REAR : "
  );

  display.print(
    myData.rear ?
    "OBSTACLE" :
    "CLEAR"
  );


  // Left
  display.setCursor(
    0,
    50
  );

  display.print(
    "LEFT : "
  );

  display.print(
    myData.left ?
    "OBSTACLE" :
    "CLEAR"
  );


  display.display();
}


// ================================================================
// OLED PAGE 2
// ================================================================

void showEnvironmentPage()
{
  display.clearDisplay();

  display.setTextColor(
    SH110X_WHITE
  );

  display.setTextSize(1);


  display.setCursor(
    0,
    0
  );

  display.println(
    "ENVIRONMENT"
  );


  display.setCursor(
    0,
    18
  );

  display.print(
    "TEMP: "
  );

  display.print(
    myData.temperature,
    1
  );

  display.println(
    " C"
  );


  display.setCursor(
    0,
    32
  );

  display.print(
    "HUM : "
  );

  display.print(
    myData.humidity,
    1
  );

  display.println(
    " %"
  );


  display.setCursor(
    0,
    50
  );

  display.print(
    "V2V: "
  );


  if (
    remoteAvailable &&
    millis() - lastRemotePacket < 3000
  )
  {
    display.print(
      "CONNECTED"
    );
  }
  else
  {
    display.print(
      "WAITING"
    );
  }


  display.display();
}


// ================================================================
// OLED PAGE 3
// ================================================================

void showTiltPage()
{
  display.clearDisplay();

  display.setTextColor(
    SH110X_WHITE
  );

  display.setTextSize(1);


  display.setCursor(
    0,
    0
  );

  display.println(
    "MPU6050 TILT"
  );


  display.setCursor(
    0,
    18
  );

  display.print(
    "PITCH: "
  );

  display.print(
    myData.pitch,
    1
  );

  display.print(
    " deg"
  );


  display.setCursor(
    0,
    34
  );

  display.print(
    "ROLL : "
  );

  display.print(
    myData.roll,
    1
  );

  display.print(
    " deg"
  );


  display.setCursor(
    0,
    51
  );


  if (
    abs(myData.pitch) > 20 ||
    abs(myData.roll) > 20
  )
  {
    display.print(
      "HIGH TILT!"
    );
  }
  else
  {
    display.print(
      "TILT NORMAL"
    );
  }


  display.display();
}


// ================================================================
// OLED PAGE 4
// ================================================================

void showRemotePage()
{
  display.clearDisplay();

  display.setTextColor(
    SH110X_WHITE
  );

  display.setTextSize(1);


  display.setCursor(
    0,
    0
  );

  display.println(
    "OTHER VEHICLE"
  );


  // No V2V connection

  if (
    !remoteAvailable ||
    millis() - lastRemotePacket > 3000
  )
  {
    display.setCursor(
      0,
      25
    );

    display.println(
      "NO V2V DATA"
    );

    display.display();

    return;
  }


  display.setCursor(
    0,
    14
  );

  display.print(
    "VEHICLE: "
  );

  display.println(
    remoteData.vehicleID
  );


  display.setCursor(
    0,
    27
  );

  display.print(
    "F:"
  );

  display.print(
    remoteData.front ?
    "X" :
    "-"
  );


  display.print(
    " R:"
  );

  display.print(
    remoteData.right ?
    "X" :
    "-"
  );


  display.print(
    " B:"
  );

  display.print(
    remoteData.rear ?
    "X" :
    "-"
  );


  display.print(
    " L:"
  );

  display.print(
    remoteData.left ?
    "X" :
    "-"
  );


  display.setCursor(
    0,
    42
  );

  display.print(
    "TEMP: "
  );

  display.print(
    remoteData.temperature,
    1
  );


  display.setCursor(
    0,
    55
  );

  display.print(
    "HUM : "
  );

  display.print(
    remoteData.humidity,
    0
  );


  display.display();
}


// ================================================================
// OLED UPDATE
// ================================================================

void updateOLED()
{
  if (
    millis() - lastOLED < 2000
  )
  {
    return;
  }


  lastOLED = millis();


  oledPage++;


  if (
    oledPage > 3
  )
  {
    oledPage = 0;
  }


  if (
    oledPage == 0
  )
  {
    showDirectionPage();
  }

  else if (
    oledPage == 1
  )
  {
    showEnvironmentPage();
  }

  else if (
    oledPage == 2
  )
  {
    showTiltPage();
  }

  else if (
    oledPage == 3
  )
  {
    showRemotePage();
  }
}


// ================================================================
// PHONE WEB PAGE
// ================================================================

String htmlPage()
{
  String html = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta name="viewport"
content="width=device-width, initial-scale=1">

<title>V2V Vehicle Safety</title>

<style>

body {
  font-family: Arial;
  background: #111;
  color: white;
  text-align: center;
  margin: 0;
  padding: 15px;
}

h1 {
  font-size: 24px;
}

.card {
  background: #222;
  border-radius: 15px;
  padding: 15px;
  margin: 12px 0;
}

.grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
}

.sensor {
  padding: 15px;
  border-radius: 12px;
  background: #333;
  font-size: 17px;
}

.clear {
  background: #174d2a;
}

.obstacle {
  background: #8b1e1e;
}

.normal {
  color: #55ff77;
}

.warning {
  color: #ffcc33;
}

.danger {
  color: #ff3333;
}

.value {
  font-size: 23px;
  font-weight: bold;
}

</style>

</head>


<body>


<h1>🚗 V2V SAFETY</h1>


<div class="card">

<h2 id="status">
CONNECTING...
</h2>

<p>
Vehicle:
<span id="vehicle">-</span>
</p>

<p>
Sensors Detected:
<span id="count" class="value">0</span>
</p>

</div>


<div class="card">

<h2>LOCAL VEHICLE</h2>


<div class="grid">


<div id="front"
class="sensor clear">

FRONT

<br>

<span class="value">
CLEAR
</span>

</div>


<div id="right"
class="sensor clear">

RIGHT

<br>

<span class="value">
CLEAR
</span>

</div>


<div id="rear"
class="sensor clear">

REAR

<br>

<span class="value">
CLEAR
</span>

</div>


<div id="left"
class="sensor clear">

LEFT

<br>

<span class="value">
CLEAR
</span>

</div>


</div>

</div>


<div class="card">

<h2>ENVIRONMENT</h2>


<p>
Temperature:
<span id="temp" class="value">
-
</span>
°C
</p>


<p>
Humidity:
<span id="hum" class="value">
-
</span>
%
</p>

</div>


<div class="card">

<h2>MPU6050</h2>


<p>
Pitch:
<span id="pitch" class="value">
-
</span>
°
</p>


<p>
Roll:
<span id="roll" class="value">
-
</span>
°
</p>

</div>


<div class="card">

<h2>🚘 OTHER VEHICLE</h2>


<p>
V2V:
<span id="v2v" class="value">
WAITING
</span>
</p>


<p>
Vehicle:
<span id="remoteVehicle">
-
</span>
</p>


<p>
Front:
<span id="remoteFront">
-
</span>
</p>


<p>
Right:
<span id="remoteRight">
-
</span>
</p>


<p>
Rear:
<span id="remoteRear">
-
</span>
</p>


<p>
Left:
<span id="remoteLeft">
-
</span>
</p>


<p>
Temperature:
<span id="remoteTemp">
-
</span>
°C
</p>


<p>
Humidity:
<span id="remoteHum">
-
</span>
%
</p>


</div>


<script>


function setSensor(
  id,
  detected
)
{

  let element =
    document.getElementById(id);

  let value =
    element.querySelector(
      ".value"
    );


  if(detected)
  {

    element.className =
      "sensor obstacle";

    value.innerHTML =
      "OBSTACLE";

  }

  else
  {

    element.className =
      "sensor clear";

    value.innerHTML =
      "CLEAR";

  }

}


function update()
{

  fetch("/api")

  .then(
    response =>
      response.json()
  )

  .then(
    data =>
    {


      // Vehicle

      document.getElementById(
        "vehicle"
      ).innerHTML =
        data.vehicleID;


      // Sensor count

      document.getElementById(
        "count"
      ).innerHTML =
        data.sensorCount;


      // Sensors

      setSensor(
        "front",
        data.front
      );

      setSensor(
        "right",
        data.right
      );

      setSensor(
        "rear",
        data.rear
      );

      setSensor(
        "left",
        data.left
      );


      // Environment

      document.getElementById(
        "temp"
      ).innerHTML =
        data.temperature.toFixed(1);


      document.getElementById(
        "hum"
      ).innerHTML =
        data.humidity.toFixed(1);


      // MPU

      document.getElementById(
        "pitch"
      ).innerHTML =
        data.pitch.toFixed(1);


      document.getElementById(
        "roll"
      ).innerHTML =
        data.roll.toFixed(1);


      // Status

      let status =
        document.getElementById(
          "status"
        );


      if(data.sensorCount == 0)
      {

        status.innerHTML =
          "✓ NORMAL";

        status.className =
          "normal";

      }

      else if(data.sensorCount == 1)
      {

        status.innerHTML =
          "⚠ WARNING";

        status.className =
          "warning";

      }

      else if(data.sensorCount == 2)
      {

        status.innerHTML =
          "⚠ HIGH WARNING";

        status.className =
          "warning";

      }

      else
      {

        status.innerHTML =
          "🚨 DANGER";

        status.className =
          "danger";

      }


      // -----------------------
      // OTHER VEHICLE
      // -----------------------

      if(data.remoteAvailable)
      {

        document.getElementById(
          "v2v"
        ).innerHTML =
          "CONNECTED";

        document.getElementById(
          "v2v"
        ).className =
          "normal";


        document.getElementById(
          "remoteVehicle"
        ).innerHTML =
          data.remoteVehicleID;


        document.getElementById(
          "remoteFront"
        ).innerHTML =
          data.remoteFront ?
          "OBSTACLE" :
          "CLEAR";


        document.getElementById(
          "remoteRight"
        ).innerHTML =
          data.remoteRight ?
          "OBSTACLE" :
          "CLEAR";


        document.getElementById(
          "remoteRear"
        ).innerHTML =
          data.remoteRear ?
          "OBSTACLE" :
          "CLEAR";


        document.getElementById(
          "remoteLeft"
        ).innerHTML =
          data.remoteLeft ?
          "OBSTACLE" :
          "CLEAR";


        document.getElementById(
          "remoteTemp"
        ).innerHTML =
          data.remoteTemp.toFixed(1);


        document.getElementById(
          "remoteHum"
        ).innerHTML =
          data.remoteHum.toFixed(1);

      }

      else
      {

        document.getElementById(
          "v2v"
        ).innerHTML =
          "WAITING";

        document.getElementById(
          "v2v"
        ).className =
          "warning";

      }

    }

  );

}


// Update every second

setInterval(
  update,
  1000
);

update();


</script>


</body>

</html>

)rawliteral";


  return html;
}


// ================================================================
// PHONE API
// ================================================================

void handleAPI()
{

  bool remoteOK =
    remoteAvailable &&
    millis() - lastRemotePacket < 3000;


  String json = "{";


  // ------------------------------------------------
  // LOCAL VEHICLE
  // ------------------------------------------------

  json += "\"vehicleID\":";
  json += String(VEHICLE_ID);


  json += ",\"front\":";
  json +=
    myData.front ?
    "true" :
    "false";


  json += ",\"right\":";
  json +=
    myData.right ?
    "true" :
    "false";


  json += ",\"rear\":";
  json +=
    myData.rear ?
    "true" :
    "false";


  json += ",\"left\":";
  json +=
    myData.left ?
    "true" :
    "false";


  json += ",\"sensorCount\":";
  json +=
    String(
      myData.sensorCount
    );


  json += ",\"temperature\":";
  json +=
    String(
      myData.temperature,
      1
    );


  json += ",\"humidity\":";
  json +=
    String(
      myData.humidity,
      1
    );


  json += ",\"pitch\":";
  json +=
    String(
      myData.pitch,
      1
    );


  json += ",\"roll\":";
  json +=
    String(
      myData.roll,
      1
    );


  json += ",\"dangerLevel\":";
  json +=
    String(
      myData.dangerLevel
    );


  // ------------------------------------------------
  // REMOTE VEHICLE
  // ------------------------------------------------

  json += ",\"remoteAvailable\":";
  json +=
    remoteOK ?
    "true" :
    "false";


  if(remoteOK)
  {

    json += ",\"remoteVehicleID\":";
    json +=
      String(
        remoteData.vehicleID
      );


    json += ",\"remoteFront\":";
    json +=
      remoteData.front ?
      "true" :
      "false";


    json += ",\"remoteRight\":";
    json +=
      remoteData.right ?
      "true" :
      "false";


    json += ",\"remoteRear\":";
    json +=
      remoteData.rear ?
      "true" :
      "false";


    json += ",\"remoteLeft\":";
    json +=
      remoteData.left ?
      "true" :
      "false";


    json += ",\"remoteTemp\":";
    json +=
      String(
        remoteData.temperature,
        1
      );


    json += ",\"remoteHum\":";
    json +=
      String(
        remoteData.humidity,
        1
      );

  }

  else
  {

    json +=
      ",\"remoteVehicleID\":0";

    json +=
      ",\"remoteFront\":false";

    json +=
      ",\"remoteRight\":false";

    json +=
      ",\"remoteRear\":false";

    json +=
      ",\"remoteLeft\":false";

    json +=
      ",\"remoteTemp\":0";

    json +=
      ",\"remoteHum\":0";

  }


  json += "}";


  server.send(
    200,
    "application/json",
    json
  );
}


// ================================================================
// START WEB SERVER
// ================================================================

void setupWebServer()
{

  server.on(
    "/",
    []()
    {
      server.send(
        200,
        "text/html",
        htmlPage()
      );
    }
  );


  server.on(
    "/api",
    handleAPI
  );


  server.begin();


  Serial.println(
    "Web server started"
  );
}


// ================================================================
// START ESP-NOW
// ================================================================

void setupESPNow()
{

  if(
    esp_now_init()
    != ESP_OK
  )
  {
    Serial.println(
      "ESP-NOW initialization FAILED!"
    );

    return;
  }


  esp_now_register_recv_cb(
    onDataReceive
  );


  esp_now_peer_info_t peerInfo = {};


  memcpy(
    peerInfo.peer_addr,
    broadcastAddress,
    6
  );


  peerInfo.channel =
    V2V_CHANNEL;


  peerInfo.encrypt =
    false;


  if(
    esp_now_add_peer(
      &peerInfo
    )
    != ESP_OK
  )
  {
    Serial.println(
      "Failed to add ESP-NOW peer"
    );
  }
  else
  {
    Serial.println(
      "ESP-NOW ready"
    );
  }
}


// ================================================================
// SETUP
// ================================================================

void setup()
{

  Serial.begin(
    115200
  );


  delay(1000);


  // =============================================================
  // GPIO
  // =============================================================

  pinMode(
    IR_FRONT,
    INPUT
  );

  pinMode(
    IR_RIGHT,
    INPUT
  );

  pinMode(
    IR_REAR,
    INPUT
  );

  pinMode(
    IR_LEFT,
    INPUT
  );


  pinMode(
    GREEN_LED,
    OUTPUT
  );

  pinMode(
    YELLOW_LED,
    OUTPUT
  );

  pinMode(
    RED_LED,
    OUTPUT
  );

  pinMode(
    BUZZER,
    OUTPUT
  );


  digitalWrite(
    GREEN_LED,
    LOW
  );

  digitalWrite(
    YELLOW_LED,
    LOW
  );

  digitalWrite(
    RED_LED,
    LOW
  );

  digitalWrite(
    BUZZER,
    LOW
  );


  // =============================================================
  // I2C
  // =============================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );


  // =============================================================
  // SH1106 OLED
  // =============================================================

  if(
    !display.begin(
      OLED_ADDRESS,
      true
    )
  )
  {

    Serial.println(
      "SH1106 OLED NOT FOUND!"
    );

  }

  else
  {

    Serial.println(
      "SH1106 OLED connected!"
    );


    display.clearDisplay();

    display.setTextColor(
      SH110X_WHITE
    );

    display.setTextSize(2);


    display.setCursor(
      10,
      15
    );

    display.println(
      "V2V"
    );


    display.setCursor(
      10,
      40
    );

    display.print(
      "V"
    );

    display.println(
      VEHICLE_ID
    );


    display.display();


    delay(1500);
  }


  // =============================================================
  // DHT11
  // =============================================================

  dht.begin();


  // =============================================================
  // MPU6050
  // =============================================================

  if(
    !mpu.begin()
  )
  {

    Serial.println(
      "MPU6050 NOT FOUND!"
    );

  }

  else
  {

    Serial.println(
      "MPU6050 connected!"
    );


    mpu.setAccelerometerRange(
      MPU6050_RANGE_8_G
    );


    mpu.setGyroRange(
      MPU6050_RANGE_500_DEG
    );


    mpu.setFilterBandwidth(
      MPU6050_BAND_21_HZ
    );

  }


  // =============================================================
  // WIFI
  // =============================================================

  WiFi.mode(
    WIFI_AP_STA
  );


  String ssid =
    "V2V_Vehicle_" +
    String(VEHICLE_ID);


  WiFi.softAP(
    ssid.c_str(),
    "12345678",
    V2V_CHANNEL
  );


  // =============================================================
  // SERIAL INFORMATION
  // =============================================================

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "       V2V VEHICLE SYSTEM"
  );

  Serial.println(
    "================================"
  );


  Serial.print(
    "Vehicle ID: "
  );

  Serial.println(
    VEHICLE_ID
  );


  Serial.print(
    "WiFi Name: "
  );

  Serial.println(
    ssid
  );


  Serial.print(
    "WiFi Password: "
  );

  Serial.println(
    "12345678"
  );


  Serial.print(
    "Phone IP: "
  );

  Serial.println(
    WiFi.softAPIP()
  );


  Serial.print(
    "ESP-NOW Channel: "
  );

  Serial.println(
    V2V_CHANNEL
  );


  Serial.println(
    "================================"
  );


  // =============================================================
  // ESP-NOW
  // =============================================================

  setupESPNow();


  // =============================================================
  // WEB SERVER
  // =============================================================

  setupWebServer();


  // =============================================================
  // FIRST SENSOR READING
  // =============================================================

  readSensors();

  updateAlerts();

  showDirectionPage();
}


// ================================================================
// MAIN LOOP
// ================================================================

void loop()
{

  // ------------------------------------------------
  // PHONE WEB SERVER
  // ------------------------------------------------

  server.handleClient();


  // ------------------------------------------------
  // SENSOR READING
  // ------------------------------------------------

  if(
    millis() - lastSensorRead >= 2000
  )
  {

    lastSensorRead =
      millis();


    readSensors();

  }


  // ------------------------------------------------
  // ALERT SYSTEM
  // ------------------------------------------------

  updateAlerts();


  // ------------------------------------------------
  // ESP-NOW TRANSMISSION
  // ------------------------------------------------

  if(
    millis() - lastSend >= 500
  )
  {

    lastSend =
      millis();


    sendVehicleData();

  }


  // ------------------------------------------------
  // OLED
  // ------------------------------------------------

  updateOLED();
}