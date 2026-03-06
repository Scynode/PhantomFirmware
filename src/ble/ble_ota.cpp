/**
 * @file ble_ota.cpp
 * @brief Over-the-air (OTA) firmware update callback for the Phantom sculpture.
 *
 * This file implements FilesCallbacks_receiveOta, which handles binary firmware
 * delivery over the BLE characteristic FILE_UUID_RECEIVE_OTA.
 *
 * ─── OTA Protocol ──────────────────────────────────────────────────────────
 *
 * The host app (iOS/Android) drives the update in two phases:
 *
 *  Phase 1 – First write (initialisation packet):
 *    Payload: 4 bytes, little-endian uint32 = total firmware image size.
 *    If the NVS key "updatingStatus" is NOT already 0 (i.e. this is the very
 *    first OTA attempt since boot), the board:
 *      a) saves updatingStatus = 0  so it will skip this first-time check
 *         on the next boot, and
 *      b) immediately calls esp_restart() to reboot into a clean state where
 *         only the BLE stack is running (no motor tasks, etc.).
 *    On the second call (after the reboot), initializeOTA() is run:
 *      - total size is decoded from the 4-byte header
 *      - esp_ota_get_next_update_partition() selects the inactive OTA slot
 *      - esp_ota_begin() opens the partition for writing
 *    After initializeOTA() succeeds, updateFlag = true and partitionReady = true.
 *
 *  Phase 2 – Subsequent writes (data packets):
 *    Each write carries a chunk of raw firmware bytes (any size ≤ MTU).
 *    esp_ota_write() appends the chunk to the open OTA partition.
 *    After each successful write the board notifies {0x01, 0x00} so the host
 *    knows it can send the next chunk.
 *    When receivedSize >= totalSize the board calls finalizeOTA():
 *      - esp_ota_end() validates and closes the image
 *      - esp_ota_set_boot_partition() makes the new image the boot target
 *      - "updatingStatus" = -1 is saved to NVS (clears the OTA-mode flag)
 *      - The board notifies {0x02, 0x00} (finish) then reboots after 3 s.
 *
 * ─── Notification format ───────────────────────────────────────────────────
 *
 *   {0x01, 0x00}  – chunk accepted; host may send next chunk
 *   {0x02, 0x00}  – OTA complete; reboot imminent
 *   {0xFF, code}  – error; ESP reboots after 3 s.  Error codes:
 *       1  invalid first packet (< 4 bytes)
 *       2  no OTA update partition found
 *       3  esp_ota_begin() failed
 *       4  esp_ota_write() failed
 *       5  esp_ota_end() failed
 *       6  esp_ota_set_boot_partition() failed
 *
 * ─── onRead behaviour ──────────────────────────────────────────────────────
 *   Returns a single byte: 0x10 if the partition is ready (i.e. after a
 *   successful initializeOTA()), 0x00 otherwise.  The host can poll this to
 *   confirm the board is ready to receive data chunks before sending them.
 */

#include "BLE.h"
#include <config.h>
#include <Arduino.h>
#include "esp_ota_ops.h"
#include "esp_int_wdt.h"
#include "esp_task_wdt.h"
#include "ble_state.h"
#include "ble_callbacks.h"

// ---------------------------------------------------------------------------
// onWrite – entry point for every BLE write to FILE_UUID_RECEIVE_OTA
// ---------------------------------------------------------------------------
void FilesCallbacks_receiveOta::onWrite(BLECharacteristic *characteristic)
{
    // Cache the characteristic pointer the first time we see it
    if (localCharacteristic == nullptr)
    {
        localCharacteristic = characteristic;
    }

    if (!updateFlag)
    {
        // Not yet in OTA mode: check whether this is the very first OTA attempt
        preferences.begin("myApp", false);
        int updatingStatus = preferences.getInt("updatingStatus", -1);
        preferences.end();

        if (updatingStatus == -1)
        {
            // First time receiving an OTA packet since last factory reset.
            // Save the flag and reboot so the firmware starts in a clean state
            // (motor tasks, etc. are not running during OTA).
            preferences.begin("myApp", false);
            preferences.putInt("updatingStatus", 0);
            preferences.end();
            esp_restart();
        }

        // updatingStatus == 0: we are in OTA-mode after the reboot – proceed
        initializeOTA();
    }
    else
    {
        // Already initialised – write the next data chunk
        writeOTAData();
    }
}

// ---------------------------------------------------------------------------
// onRead – host can poll to check whether the partition is ready
// ---------------------------------------------------------------------------
void FilesCallbacks_receiveOta::onRead(BLECharacteristic *characteristic)
{
    uint8_t send = partitionReady ? 0x10 : 0x00;
    characteristic->setValue(&send, sizeof(send));
}

// ---------------------------------------------------------------------------
// initializeOTA – process the first packet and open the OTA partition
// ---------------------------------------------------------------------------
void FilesCallbacks_receiveOta::initializeOTA()
{
    Serial.println("Initializing OTA");

    // Extend the watchdog to 15 s so the partition-erase overhead does not
    // trigger a reset mid-write.
    esp_task_wdt_init(15, 0);
    delay(100);

    std::string rxData = localCharacteristic->getValue();

    // The first packet must be at least 4 bytes (the little-endian image size)
    if (rxData.length() < 4)
    {
        notifyError(1, "Invalid initial packet");
        return;
    }

    // Decode the 4-byte little-endian total size from the packet header
    totalSize = *reinterpret_cast<const uint32_t *>(rxData.c_str());
    Serial.printf("Total size: %u bytes\n", totalSize);

    // Locate the next unused OTA partition slot (alternates between OTA_0 and OTA_1)
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == nullptr)
    {
        notifyError(2, "No update partition found");
        return;
    }

    // Open the OTA partition for writing; esp_ota_begin() erases it first
    esp_err_t error = esp_ota_begin(update_partition, totalSize, &otaHandler);
    if (error != ESP_OK)
    {
        notifyError(3, "esp_ota_begin failed");
        return;
    }

    updateFlag     = true;
    receivedSize   = 0;
    partitionReady = true;

    Serial.println("OTA initialized");
}

// ---------------------------------------------------------------------------
// writeOTAData – append one chunk to the open OTA partition
// ---------------------------------------------------------------------------
void FilesCallbacks_receiveOta::writeOTAData()
{
    std::string rxData = localCharacteristic->getValue();

    esp_err_t writeResult = esp_ota_write(otaHandler, rxData.c_str(), rxData.length());
    if (writeResult != ESP_OK)
    {
        notifyError(4, "esp_ota_write failed");
        return;
    }

    receivedSize += rxData.length();
    Serial.printf("Received: %u / %u bytes\n", receivedSize, totalSize);

    if (receivedSize >= totalSize)
    {
        // All bytes received – validate and activate the new image
        finalizeOTA();
    }
    else
    {
        // Acknowledge the chunk; host may send the next one
        notifySuccess();
    }
}

// ---------------------------------------------------------------------------
// finalizeOTA – validate, activate, and reboot into the new firmware
// ---------------------------------------------------------------------------
void FilesCallbacks_receiveOta::finalizeOTA()
{
    esp_err_t endResult = esp_ota_end(otaHandler);
    if (endResult != ESP_OK)
    {
        notifyError(5, "esp_ota_end failed");
        return;
    }

    if (esp_ota_set_boot_partition(update_partition) == ESP_OK)
    {
        Serial.println("OTA successful, restarting...");

        // Clear the OTA-mode flag so normal firmware boots next time
        preferences.begin("myApp", false);
        preferences.putInt("updatingStatus", -1);
        preferences.end();

        notifyFinish(); // {0x02, 0x00} – tells the host the update is done
        delay(3000);    // Give the host time to display a success message
        esp_restart();
    }
    else
    {
        notifyError(6, "Unable to set boot partition");
    }
}

// ---------------------------------------------------------------------------
// notifyError – abort the OTA session, erase the partition, and restart
// ---------------------------------------------------------------------------
void FilesCallbacks_receiveOta::notifyError(uint8_t errorCode, const char *message)
{
    Serial.printf("OTA Error: %s (Code: %d)\n", message, errorCode);

    updateFlag = false;

    // Close the OTA handle if it was opened
    if (otaHandler)
    {
        esp_ota_end(otaHandler);
        otaHandler = 0;
    }

    // Erase the partially-written partition so it cannot accidentally boot
    if (update_partition != nullptr)
    {
        esp_partition_erase_range(update_partition, 0, update_partition->size);
    }

    // Clear the OTA-mode flag so normal firmware boots after restart
    preferences.begin("myApp", false);
    preferences.putInt("updatingStatus", -1);
    preferences.end();

    notify(0xFF, errorCode); // {0xFF, errorCode} tells the host what went wrong
    delay(3000);
    esp_restart();
}

// ---------------------------------------------------------------------------
// Notification helpers
// ---------------------------------------------------------------------------
void FilesCallbacks_receiveOta::notifySuccess()
{
    notify(0x01, 0); // chunk accepted; continue
}

void FilesCallbacks_receiveOta::notifyFinish()
{
    notify(0x02, 0); // OTA complete
}

void FilesCallbacks_receiveOta::notify(uint8_t status, uint8_t code)
{
    uint8_t response[2] = {status, code};
    localCharacteristic->setValue(response, 2);
    localCharacteristic->notify();
}
