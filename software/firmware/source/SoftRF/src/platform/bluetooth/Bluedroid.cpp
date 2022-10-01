// BLERemoteCharacteristic.cpp
// BLEClient.cpp 

//	if (errRc != ESP_OK) {
//		log_e(...);
// +        m_semaphoreWriteCharEvt.timedWait("writeValue", 100);
//		return;
//	}

/*
 * BluetoothHelper.cpp
 * Copyright (C) 2018-2024 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(ESP32)
#include "sdkconfig.h"

#include "../../system/SoC.h"
#include "esp_task_wdt.h"

#if defined(CONFIG_BLUEDROID_ENABLED) && !defined(USE_NIMBLE)
/*
 *  BLE code is based on Neil Kolban example for IDF:
 *    https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
 *  Ported to Arduino ESP32 by Evandro Copercini
 *  HM-10 emulation and adaptation for SoftRF is done by Linar Yusupov.
 */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "esp_gap_bt_api.h"

#include "../../driver/EEPROM.h"
#include "../../driver/Bluetooth.h"
#include "../../driver/WiFi.h"
#include "../../driver/Battery.h"

#include <core_version.h>
#include <cbuf.h>

BLEServer* pServer = NULL;
BLECharacteristic* pUARTCharacteristic = NULL;
BLECharacteristic* pBATCharacteristic  = NULL;

BLECharacteristic* pModelCharacteristic         = NULL;
BLECharacteristic* pSerialCharacteristic        = NULL;
BLECharacteristic* pFirmwareCharacteristic      = NULL;
BLECharacteristic* pHardwareCharacteristic      = NULL;
BLECharacteristic* pSoftwareCharacteristic      = NULL;
BLECharacteristic* pManufacturerCharacteristic  = NULL;

bool deviceConnected    = false;
bool oldDeviceConnected = false;

static boolean doConnect = false;
static boolean connected = false;
static BLEAdvertisedDevice* LXNAVDevice = NULL;
static BLERemoteCharacteristic* LXNAVRXCharacteristic = NULL;
static BLERemoteCharacteristic* LXNAVTXCharacteristic = NULL;

#if defined(USE_BLE_MIDI)
BLECharacteristic* pMIDICharacteristic = NULL;
#endif /* USE_BLE_MIDI */

cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;
cbuf *LXNAV_FIFO_RX = NULL; //*LXNAV_FIFO_TX = NULL;

#if defined(CONFIG_IDF_TARGET_ESP32)
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(ENABLE_BT_VOICE)
#include "BluetoothA2DPSource.h"
#include "piano16bit.h"

BluetoothA2DPSource a2dp_source;
SoundData *sound_data = new OneChannelSoundData((int16_t*)piano16bit_raw, piano16bit_raw_len/2);
#endif /* ENABLE_BT_VOICE */

String BT_name = HOSTNAME;

static unsigned long Discovery_TimeMarker = 0;
static unsigned long BLE_Notify_TimeMarker = 0;
static unsigned long BLE_Advertising_TimeMarker = 0;

BLEDescriptor UserDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      BLE_Advertising_TimeMarker = millis();
    }
};

class UARTCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pUARTCharacteristic) {
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=5
      String rxValue = pUARTCharacteristic->getValue();
#else
      std::string rxValue = pUARTCharacteristic->getValue();
#endif /* ESP_IDF_VERSION_MAJOR */

      if (rxValue.length() > 0) {
        BLE_FIFO_RX->write(rxValue.c_str(),
                      (BLE_FIFO_RX->room() > rxValue.length() ?
                      rxValue.length() : BLE_FIFO_RX->room()));
      }
    }
};

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
void SerialBT_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if (event == ESP_SPP_OPEN_EVT) {
    Serial.println(F("Bluetooth conection open."));
    deviceConnected = true;
  } else if (event == ESP_SPP_DISCOVERY_COMP_EVT) {
    Serial.println(F("Bluetooth discovery completed."));
  } else if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println(F("Bluetooth connection closed."));
    deviceConnected = false;
  }
}
//esp_spp_cb_t pSerialBT_callback = &SerialBT_callback;
#endif

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    // Serial.print(">>> Notify callback for characteristic ");
    // Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    // Serial.print(" of data length ");
    // Serial.println(length);
    // pData[length] = '\0';
    // Serial.println((char*)pData);


    LXNAV_FIFO_RX->write((const char*)pData,
              (LXNAV_FIFO_RX->room() > length ?
              length : LXNAV_FIFO_RX->room()));
    // Serial.println("<<< notifyCallback");
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    Serial.println("LXNAV disconnected");
    connected = false;
    delete LXNAVDevice;
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(LXNAVDevice->getAddress().toString().c_str());

    BLEClient*  pClient  = BLEDevice::createClient();
    vTaskDelay(10);
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    if (pClient->connect(LXNAVDevice)) {  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
        vTaskDelay(10);
        Serial.println(" - Connected to LXNAV");
        pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)

        // Obtain a reference to the service we are after in the remote BLE server.
        BLERemoteService* pRemoteService = pClient->getService(BLEUUID(LXNAV_SERVICE_UUID));
        vTaskDelay(10);
        if (pRemoteService == nullptr) {
          Serial.println("Failed to find LXNAV service UUID");
          pClient->disconnect();
          return false;
        }
        Serial.println(" - Found LXNAV service");


        // Obtain a reference to the characteristic in the service of the remote BLE server.
        LXNAVRXCharacteristic = pRemoteService->getCharacteristic(BLEUUID(LXNAVRX_CHARACTERISTIC_UUID));
        vTaskDelay(10);
        if (LXNAVRXCharacteristic == nullptr) {
          Serial.println("Failed to find RX characteristic UUID");
          pClient->disconnect();
          return false;
        }
        Serial.println(" - Found RX characteristic");

        // // Read the value of the characteristic.
        // if(LXNAVRXCharacteristic->canRead()) {
          // std::string value = LXNAVRXCharacteristic->readValue();
          // Serial.print("The characteristic value was: ");
          // Serial.println(value.c_str());
        // }

        if(LXNAVRXCharacteristic->canNotify())
          LXNAVRXCharacteristic->registerForNotify(notifyCallback);

        // Obtain a reference to the characteristic in the service of the remote BLE server.
        LXNAVTXCharacteristic = pRemoteService->getCharacteristic(BLEUUID(LXNAVTX_CHARACTERISTIC_UUID));
        vTaskDelay(10);
        if (LXNAVTXCharacteristic == nullptr) {
          Serial.println("Failed to find TX characteristic UUID");
          pClient->disconnect();
          return false;
        }
        Serial.println(" - Found TX characteristic");


        connected = true;
        return true;
    }
    return false;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Serial.print("BLE Advertised Device found: ");
    // Serial.println(advertisedDevice.toString().c_str());

    const char* name = advertisedDevice.getName().c_str();
    if (strcmp(name, settings->LXNAV_name)==0) {
      BLEDevice::getScan()->stop();
      LXNAVDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  } // onResult
}; // MyAdvertisedDeviceCallbacks

static void ESP32_Bluetooth_setup()
{
  BT_name += "-";
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

  switch (settings->bluetooth)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP_SLAVE:
    {
      esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

      SerialBT.begin(BT_name.c_str());
    }
    break;
  case BLUETOOTH_SPP_MASTER:
    {
      esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

      SerialBT.begin(BT_name.c_str(), true);
      SerialBT.register_callback(SerialBT_callback);
      SerialBT.connect(settings->LXNAV_name);

      // TO BE CHANGED IN BluetoothSerial.cpp
    // static bool waitForConnect(int timeout) {
        // #define CYCLE 50
        // TickType_t xTicksToWait = timeout / portTICK_PERIOD_MS / CYCLE;
        // for (int i=0;i<xTicksToWait;i++) {
            // if ((xEventGroupWaitBits(_spp_event_group, SPP_CONNECTED, pdFALSE, pdTRUE, CYCLE) & SPP_CONNECTED) != 0) {
                // return 1;
            // }
            // esp_task_wdt_reset();
        // }
        // return 0;
    // }

    // static bool waitForConnect(int timeout) {
        // #define CYCLE 50
        // TickType_t xTicksToWait = timeout / portTICK_PERIOD_MS / CYCLE;
        // for (int i=0;i<xTicksToWait;i++) {
            // // wait for connected or closed
            // EventBits_t rc = xEventGroupWaitBits(_spp_event_group, SPP_CONNECTED | SPP_CLOSED, pdFALSE, pdFALSE, CYCLE);
            // if((rc & SPP_CONNECTED) != 0)
                // return true;
            // else if((rc & SPP_CLOSED) != 0) {
                // log_d("connection closed!");
                // return false;
            // }
            // esp_task_wdt_reset();
        // }
        // log_d("timeout");
        // return false;
    // }

      Discovery_TimeMarker = millis();
    }
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_SIZE);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_SIZE);

#if defined(CONFIG_IDF_TARGET_ESP32)
      esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
#endif /* CONFIG_IDF_TARGET_ESP32 */

      // Create the BLE Device
      BLEDevice::init((BT_name+"-LE").c_str());

      /*
       * Set the MTU of the packets sent,
       * maximum is 500, Apple needs 23 apparently.
       */
      // BLEDevice::setMTU(23);

      // Create the BLE Server
      pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
      BLEService *pService = pServer->createService(BLEUUID(UART_SERVICE_UUID16));

      // Create a BLE Characteristic
      pUARTCharacteristic = pService->createCharacteristic(
                              BLEUUID(UART_CHARACTERISTIC_UUID16),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_NOTIFY |
                              BLECharacteristic::PROPERTY_WRITE_NR
                            );

      UserDescriptor.setValue("HMSoft");
      pUARTCharacteristic->addDescriptor(&UserDescriptor);
      pUARTCharacteristic->addDescriptor(new BLE2902());

      pUARTCharacteristic->setCallbacks(new UARTCallbacks());

      // Start the service
      pService->start();

      // Create the BLE Service
      pService = pServer->createService(BLEUUID(UUID16_SVC_BATTERY));

      // Create a BLE Characteristic
      pBATCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_BATTERY_LEVEL),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_NOTIFY
                            );
      pBATCharacteristic->addDescriptor(new BLE2902());

      // Start the service
      pService->start();

      // Create the BLE Service
      pService = pServer->createService(BLEUUID(UUID16_SVC_DEVICE_INFORMATION));

      // Create BLE Characteristics
      pModelCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_MODEL_NUMBER_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pSerialCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_SERIAL_NUMBER_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pFirmwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_FIRMWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pHardwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_HARDWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pSoftwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_SOFTWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pManufacturerCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_MANUFACTURER_NAME_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );

      const char *Model         = hw_info.model == SOFTRF_MODEL_STANDALONE ? "Standalone Edition" :
                                  hw_info.model == SOFTRF_MODEL_PRIME_MK2  ? "Prime Mark II"      :
                                  hw_info.model == SOFTRF_MODEL_PRIME_MK3  ? "Prime Mark III"     :
                                  hw_info.model == SOFTRF_MODEL_HAM        ? "Ham Edition"        :
                                  hw_info.model == SOFTRF_MODEL_MIDI       ? "Midi Edition"       :
                                  "Unknown";
      char SerialNum[9];
      snprintf(SerialNum, sizeof(SerialNum), "%08X", SoC->getChipId());

      const char *Firmware      = "Arduino ESP32 " ARDUINO_ESP32_RELEASE;

      char Hardware[9];
      snprintf(Hardware, sizeof(Hardware), "%08X", hw_info.revision);

      const char *Manufacturer  = SOFTRF_IDENT;
      const char *Software      = SOFTRF_FIRMWARE_VERSION;

      pModelCharacteristic->       setValue((uint8_t *) Model,        strlen(Model));
      pSerialCharacteristic->      setValue((uint8_t *) SerialNum,    strlen(SerialNum));
      pFirmwareCharacteristic->    setValue((uint8_t *) Firmware,     strlen(Firmware));
      pHardwareCharacteristic->    setValue((uint8_t *) Hardware,     strlen(Hardware));
      pSoftwareCharacteristic->    setValue((uint8_t *) Software,     strlen(Software));
      pManufacturerCharacteristic->setValue((uint8_t *) Manufacturer, strlen(Manufacturer));

      // Start the service
      pService->start();

#if defined(USE_BLE_MIDI)
      // Create the BLE Service
      pService = pServer->createService(BLEUUID(MIDI_SERVICE_UUID));

      // Create a BLE Characteristic
      pMIDICharacteristic = pService->createCharacteristic(
                              BLEUUID(MIDI_CHARACTERISTIC_UUID),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_WRITE  |
                              BLECharacteristic::PROPERTY_NOTIFY |
                              BLECharacteristic::PROPERTY_WRITE_NR
                            );

      // Create a BLE Descriptor
      pMIDICharacteristic->addDescriptor(new BLE2902());

      // Start the service
      pService->start();
#endif /* USE_BLE_MIDI */

      // Start advertising
      BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
#if 0
      pAdvertising->addServiceUUID(BLEUUID(UART_SERVICE_UUID16));
      pAdvertising->addServiceUUID(BLEUUID(UUID16_SVC_BATTERY));
#if defined(USE_BLE_MIDI)
      pAdvertising->addServiceUUID(BLEUUID(MIDI_SERVICE_UUID));
#endif /* USE_BLE_MIDI */
#else
      /* work around https://github.com/espressif/arduino-esp32/issues/6750 */
      BLEAdvertisementData BLEAdvData;
      BLEAdvData.setFlags(0x06);
      BLEAdvData.setCompleteServices(BLEUUID(UART_SERVICE_UUID16));
      BLEAdvData.setCompleteServices(BLEUUID(UUID16_SVC_BATTERY));
#if defined(USE_BLE_MIDI)
      BLEAdvData.setCompleteServices(BLEUUID(MIDI_SERVICE_UUID));
#endif /* USE_BLE_MIDI */
      pAdvertising->setAdvertisementData(BLEAdvData);
#endif
      pAdvertising->setScanResponse(true);
      pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
      pAdvertising->setMaxPreferred(0x12);
      BLEDevice::startAdvertising();

      BLE_Advertising_TimeMarker = millis();

      if (strlen(settings->LXNAV_name)) {
        LXNAV_FIFO_RX = new cbuf(BLE_FIFO_RX_SIZE);
        // LXNAV_FIFO_TX = new cbuf(BLE_FIFO_TX_SIZE);
        
        // Retrieve a Scanner and set the callback we want to use to be informed when we
        // have detected a new device.  Specify that we want active scanning and start the
        // scan to run for 5 seconds.
        Discovery_TimeMarker = millis();

        BLEScan* pBLEScan = BLEDevice::getScan();
        pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
        pBLEScan->setInterval(1349);
        pBLEScan->setWindow(449);
        pBLEScan->setActiveScan(true);
        Serial.println("Starting new BLE scan");
        pBLEScan->start(5); //, false);
      }
    }
    break;
  case BLUETOOTH_A2DP_SOURCE:
#if defined(ENABLE_BT_VOICE)
    //a2dp_source.set_auto_reconnect(false);
    a2dp_source.start("BT SPEAKER");
    a2dp_source.set_volume(100);
    a2dp_source.write_data(sound_data);
#endif
    break;
  case BLUETOOTH_NONE:
  default:
    break;
  }
}

static void ESP32_Bluetooth_loop()
{
  switch (settings->bluetooth)
  {
  case BLUETOOTH_LE_HM10_SERIAL:
    {
      // notify changed value
      // bluetooth stack will go into congestion, if too many packets are sent
      if (deviceConnected && (millis() - BLE_Notify_TimeMarker > 10)) { /* < 18000 baud */

          uint8_t chunk[BLE_MAX_WRITE_CHUNK_SIZE];
          size_t size = BLE_FIFO_TX->available();
          size = size < BLE_MAX_WRITE_CHUNK_SIZE ? size : BLE_MAX_WRITE_CHUNK_SIZE;

          if (size > 0) {
            BLE_FIFO_TX->read((char *) chunk, size);

            pUARTCharacteristic->setValue(chunk, size);
            pUARTCharacteristic->notify();
            
            if (connected and size) {
                LXNAVTXCharacteristic->writeValue (chunk, size);
            }
            
            BLE_Notify_TimeMarker = millis();
          }
      }
      // disconnecting
      if (!deviceConnected && oldDeviceConnected && (millis() - BLE_Advertising_TimeMarker > 500) ) {
          // give the bluetooth stack the chance to get things ready
          pServer->startAdvertising(); // restart advertising
          oldDeviceConnected = deviceConnected;
          BLE_Advertising_TimeMarker = millis();
      }
      // connecting
      if (deviceConnected && !oldDeviceConnected) {
          // do stuff here on connecting
          oldDeviceConnected = deviceConnected;
      }
      if (deviceConnected && isTimeToBattery()) {
        uint8_t battery_level = Battery_charge();

        pBATCharacteristic->setValue(&battery_level, 1);
        pBATCharacteristic->notify();
      }

      if (doConnect == true) {
        if (connectToServer()) {
          Serial.println("We are now connected to the BLE Server.");
        } else {
          Serial.println("We have failed to connect to the server; there is nothin more we will do.");
        }
        doConnect = false;
      }

      if (!connected &&
        (LXNAVDevice || strlen(settings->LXNAV_name)) &&
        millis() - Discovery_TimeMarker > 20e3)
     {
        Serial.println("Starting new BLE scan");
        Discovery_TimeMarker = millis();
        
        BLEScan* pBLEScan = BLEDevice::getScan();
        vTaskDelay(10);
        pBLEScan->start(2);
      }
    }
    break;
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP_MASTER:
    {
        if (!deviceConnected && millis() - Discovery_TimeMarker > 60e3) {
            bool zeros = true;
            for (uint8_t i=0; i<6; i++) {zeros &= !(settings->LXNAV_name[0]);}
            if (!zeros) {
                Discovery_TimeMarker = millis();
                SerialBT.connect(settings->LXNAV_name);
            }
        }
    }
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_NONE:
  case BLUETOOTH_SPP_SLAVE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }
}

static void ESP32_Bluetooth_fini()
{
  /* TBD */
}

static int ESP32_Bluetooth_available()
{
  int rval = 0;

  switch (settings->bluetooth)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP_SLAVE:
  case BLUETOOTH_SPP_MASTER:
    rval = SerialBT.available();
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    rval = BLE_FIFO_RX->available() || (LXNAV_FIFO_RX && LXNAV_FIFO_RX->available());
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static int ESP32_Bluetooth_read()
{
  int rval = -1;

  switch (settings->bluetooth)
  {
#if defined(CONFIG_IDF_TARGET_ESP32)
  case BLUETOOTH_SPP_SLAVE:
  case BLUETOOTH_SPP_MASTER:
    rval = SerialBT.read();
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
    if (LXNAV_FIFO_RX && LXNAV_FIFO_RX->available()) {
        rval = LXNAV_FIFO_RX->read();
    } else {
        rval = BLE_FIFO_RX->read();
    }
    break;
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

static size_t ESP32_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  size_t rval = size;

  switch (settings->bluetooth)
  {
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
  case BLUETOOTH_SPP_SLAVE:
  case BLUETOOTH_SPP_MASTER:
    rval = SerialBT.write(buffer, size);
    break;
#endif /* CONFIG_IDF_TARGET_ESP32 */
  case BLUETOOTH_LE_HM10_SERIAL:
  {
    char* cbuffer = (char *)malloc(size+1);
    cbuffer[size]='\0';
    memcpy(cbuffer, buffer, size);
    Serial.print(cbuffer);
    free(cbuffer);
    
    rval = BLE_FIFO_TX->write((char *) buffer,
                        (BLE_FIFO_TX->room() > size ? size : BLE_FIFO_TX->room()));
    break;
  }
  case BLUETOOTH_NONE:
  case BLUETOOTH_A2DP_SOURCE:
  default:
    break;
  }

  return rval;
}

IODev_ops_t ESP32_Bluetooth_ops = {
  "ESP32 Bluetooth",
  ESP32_Bluetooth_setup,
  ESP32_Bluetooth_loop,
  ESP32_Bluetooth_fini,
  ESP32_Bluetooth_available,
  ESP32_Bluetooth_read,
  ESP32_Bluetooth_write
};

#endif /* CONFIG_BLUEDROID_ENABLED */
#endif /* ESP32 */
