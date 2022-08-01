#include <Arduino.h>
#include "SSD1306Wire.h"
#include "OLEDDisplayUi.h"

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#include "BluetoothSerial.h"
#include "ELMduino.h"
BluetoothSerial SerialBT;
#define ELM_PORT SerialBT

extern "C"
{
#include <read_write_lock.h>
}
ReadWriteLock_t *rwLock;

TaskHandle_t oled_task;
TaskHandle_t obd_task;

ELM327 myELM327;

SSD1306Wire display(0x3c, 5, 4);
OLEDDisplayUi ui(&display);

int screenW = 128;
int screenH = 64;
typedef enum
{
  MOTOR_STATE,
  MOTOR_RPM,
  FUEL_RATE,
  MOTOR_LOAD,
} state_t;

uint8_t state = MOTOR_RPM;

struct Car
{
  uint8_t engineLoad = 0;
  uint16_t engineRPM = 0;
  float fuelRate = 0.0;
  uint32_t systemTime = 0;
  bool engineOn = false;
  bool deceleration = false;
} myCar;

void decelOverlay(OLEDDisplay *display, OLEDDisplayUiState *state)
{
  ReaderLock(rwLock);
  bool decel = myCar.deceleration;
  uint32_t time = myCar.systemTime;
  ReaderUnlock(rwLock);

  display->setFont(ArialMT_Plain_10);
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(128, 0, String(decel ? "DEC" : "acc"));

  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(0, 0, String(time / 1000.0, 1));
}

void OBDFrame_1(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  ReaderLock(rwLock);
  bool decel = myCar.deceleration;
  bool on = myCar.engineOn;
  ReaderUnlock(rwLock);

  const int16_t split = 90;
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_16);

  y += 20;
  display->drawString(x, y, "Motor:");
  display->drawString(split + x, y, on ? "ON" : "---");
  y += 20;
  display->drawString(x, y, "Decel:");
  display->drawString(split + x, y, decel ? "YES" : "---");
}

void OBDFrame_2(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  ReaderLock(rwLock);
  uint8_t engineLoad = myCar.engineLoad;
  float fuelRate = myCar.fuelRate;
  ReaderUnlock(rwLock);

  const int16_t split = 20;
  display->setFont(ArialMT_Plain_16);

  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->drawString(x, y + 20, "Load:");
  display->drawString(x, y + 40, "Fuel:");

  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->drawString(screenW + x, y + 20, String(engineLoad) + " %");
  display->drawString(screenW + x, y + 40, String(fuelRate, 1) + "L/h");
}

void OBDFrame_3(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  ReaderLock(rwLock);
  uint16_t engineRPM = myCar.engineRPM;
  ReaderUnlock(rwLock);

  const int16_t split = 90;
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_16);

  y += 20;
  display->drawString(x, y, "U/min:");
  display->drawString(split + x, y, String(engineRPM));
}

FrameCallback frames[] = {
    OBDFrame_1,
    OBDFrame_2,
    OBDFrame_3};

OverlayCallback overlays[] = {decelOverlay};

#define PAIR_MAX_DEVICES 3
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

bool initBluetooth()
{
  if (!btStart())
  {
    Serial.println("Failed to initialize controller");
    return false;
  }

  if (esp_bluedroid_init() != ESP_OK)
  {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }

  if (esp_bluedroid_enable() != ESP_OK)
  {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

char *bda2str(const uint8_t *bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18)
  {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

void removePairedBT()
{
  initBluetooth();
  Serial.print("ESP32 bluetooth address: ");
  Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
  // Get the numbers of bonded/paired devices in the BT module
  int count = esp_bt_gap_get_bond_device_num();
  if (!count)
  {
    Serial.println("No bonded device found.");
  }
  else
  {
    Serial.print("Bonded device count: ");
    Serial.println(count);
    if (PAIR_MAX_DEVICES < count)
    {
      count = PAIR_MAX_DEVICES;
      Serial.print("Reset bonded device count: ");
      Serial.println(count);
    }
    esp_err_t tError = esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    if (ESP_OK == tError)
    {
      for (int i = 0; i < count; i++)
      {
        Serial.print("Found bonded device # ");
        Serial.print(i);
        Serial.print(" -> ");
        Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));
        esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
        if (ESP_OK == tError)
        {
          Serial.print("Removed bonded device # ");
        }
        else
        {
          Serial.print("Failed to remove bonded device # ");
        }
        Serial.println(i);
      }
    }
  }
}

void setupOLED()
{
  // The ESP is capable of rendering 60fps in 80Mhz mode
  // but that won't give you much time for anything else
  // run it in 160Mhz mode or just set it to 30 fps
  ui.setTargetFPS(30);

  // Customize the active and inactive symbol
  // ui.setActiveSymbol(activeSymbol);
  // ui.setInactiveSymbol(inactiveSymbol);

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(TOP);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, sizeof(frames) / sizeof(frames[0]));

  // Add overlays
  ui.setOverlays(overlays, sizeof(overlays) / sizeof(overlays[0]));

  ui.setTimePerFrame(2500);
  ui.setTimePerTransition(200);

  // Initialising the UI will init the display too.
  ui.init();

  display.flipScreenVertically();
}

uint8_t addr[] = {0x66, 0x1E, 0x21, 0x00, 0xAA, 0xFE};
void setupOBD()
{
  ELM_PORT.setPin("1234");
  ELM_PORT.begin("ArduHUD", true);
  bool connected = ELM_PORT.connect(addr);
  // bool connected = ELM_PORT.connect("OBDII");

  if (connected)
  {
    Serial.println("Connected Succesfully!");
  }
  else
  {
    if (!ELM_PORT.connected(10000))
    {
      Serial.println("Failed to connect. Make sure remote device is available and in range, restarting.");
      delay(1000);
      removePairedBT();
      ESP.restart();
    }
  }

  if (!myELM327.begin(ELM_PORT, true, 2000, ISO_15765_11_BIT_500_KBAUD))
  {
    Serial.println("Couldn't connect to OBD scanner - Phase 2");
    delay(1000);
    ESP.restart();
  }
  Serial.println("Connected to ELM327");
}

void getOBDDummy(void *parameters)
{
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(random(7, 99)));
    WriterLock(rwLock);
    myCar.systemTime = millis();
    switch (state)
    {
    case MOTOR_STATE:
    {
      myCar.engineOn = millis() % 3 == 0;
      myCar.deceleration = millis() % 7 == 0;
      state++;
      break;
    }
    case MOTOR_RPM:
    {
      myCar.engineRPM = random(0, 300);
      state++;
      break;
    }
    case FUEL_RATE:
    {
      myCar.fuelRate = random(0, 300) / 10.0;
      state++;
      break;
    }
    case MOTOR_LOAD:
    {
      myCar.engineLoad = random(0, 1000) / 10.0;
      state++;
      break;
    }
    }
    WriterUnlock(rwLock);
    if (state > MOTOR_LOAD)
      state = MOTOR_STATE;
  }
}

void getOBD(void *parameters)
{
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(10));

    switch (state)
    {
    case MOTOR_STATE:
    {
      uint16_t fuelSystemStatus = myELM327.fuelSystemStatus();

      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        // https://en.wikipedia.org/w/index.php?title=OBD-II_PIDs&section=18#Service_01_PID_03_-_Fuel_system_status
        WriterLock(rwLock);
        myCar.engineOn = fuelSystemStatus & (1 << 0) == 0;
        myCar.deceleration = fuelSystemStatus & (1 << 2) == 1;
        WriterUnlock(rwLock);
        Serial.print("Engine On: ");
        Serial.println(myCar.engineOn);
        Serial.print("Deceleration: ");
        Serial.println(myCar.deceleration);
        Serial.print("Status: ");
        Serial.println(fuelSystemStatus);
        state++;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        state++;
        myELM327.printError();
      break;
    }

    case MOTOR_RPM:
    {
      float tempRPM = myELM327.rpm();

      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        WriterLock(rwLock);
        myCar.engineRPM = tempRPM;
        WriterUnlock(rwLock);
        Serial.print("RPM: ");
        Serial.println(myCar.engineRPM);
        state++;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        state++;
        myELM327.printError();
      break;
    }

    case FUEL_RATE:
    {
      float tempFuelRate = myELM327.fuelRate();
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        WriterLock(rwLock);
        myCar.fuelRate = tempFuelRate;
        WriterUnlock(rwLock);
        Serial.print("Fuel Rate: ");
        Serial.println(myCar.fuelRate);
        state++;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        state++;
        myELM327.printError();
      break;
    }

    case MOTOR_LOAD:
    {
      float tempEngineLoad = myELM327.engineLoad();
      if (myELM327.nb_rx_state == ELM_SUCCESS)
      {
        WriterLock(rwLock);
        myCar.engineLoad = tempEngineLoad;
        WriterUnlock(rwLock);
        Serial.print("Engine Load: ");
        Serial.println(myCar.engineLoad);
        state++;
      }
      else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        state++;
        myELM327.printError();
      break;
    }
    }
    if (state > MOTOR_LOAD)
      state = MOTOR_STATE;
  }
}

void updateOLED(void *parameters)
{
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(ui.update()));
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting...");

  rwLock = CreateReadWriteLockPreferWriter();
  setupOLED();

  randomSeed(analogRead(0));
  setupOBD();

  // Do timing specific work on another core
  xTaskCreatePinnedToCore(
      updateOLED,   // Task function.
      "updateOLED", // Name of task.
      10000,        // Stack size of task
      NULL,         // Parameter of the task
      19,           // Priority of the task
      &oled_task,   // Task handle to keep track of created task
      1);           // Pin task to core 0

  // Do blocking OBD call on one core
  xTaskCreatePinnedToCore(
      getOBD,    // Task function.
      "getOBD",  // Name of task.
      10000,     // Stack size of task
      NULL,      // Parameter of the task
      10,        // Priority of the task
      &obd_task, // Task handle to keep track of created task
      0);        // Pin task to core 1
}

void loop()
{
  vTaskDelete(NULL);
  vTaskDelay(pdMS_TO_TICKS(1000));
}