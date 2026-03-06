/**
 * @file ble_fragment.cpp
 * @brief Fragment class implementation and playlist reset helper.
 *
 * ─── Fragment protocol ──────────────────────────────────────────────────────
 *
 * BLE has a small MTU (typically 20–512 bytes, often 100–200 bytes in practice).
 * Payloads larger than one BLE packet must be split by the sender and
 * reassembled by the receiver.  The Fragment class implements a simple
 * framing protocol:
 *
 *  Sender side (app → board):
 *    Write 1: "START<first chunk of data>"
 *    Write 2: "<middle chunk>"      (may be repeated N times)
 *    Write N: "<last chunk>END"
 *
 *  Receiver side (board, this class):
 *    processData() is called once per BLE write.
 *    - If the data starts with "START", the buffer is cleared and the
 *      remaining bytes (after the 5-character prefix) are stored.
 *    - If the data contains "END" anywhere, everything before "END" is
 *      appended to the buffer and complete is set to true.
 *    - Otherwise the entire write is appended to the buffer.
 *
 *  After processData() returns, the caller checks isComplete().  When true,
 *  getBuffer() returns the fully reassembled payload and clear() resets state.
 *
 *  Sender side (board → app, sendFragmentedData):
 *    1. Sends the string "START" as a notification.
 *    2. Splits data into 100-byte chunks; notifies each chunk.
 *    3. Sends the string "END" as the final notification.
 *    A 50 ms delay between notifications gives the app time to process each
 *    packet before the next one arrives (BLE indication / notification queue).
 *
 * ─── resetPlaylist ──────────────────────────────────────────────────────────
 *
 * Wipes all playlist-related NVS namespaces and resets the in-memory variables
 * to their factory defaults:
 *   playlist   = ""    (empty → firmware will use built-in default games)
 *   playlistDB = ""
 *   NVS "PL_setting"  → playlist="1-2-3", playlistDB="", pgnIndex=201,
 *                         pgnCount=0, playlistCount=0
 *   NVS "PL_metadata" → cleared
 *   NVS "PL_pgn"      → cleared
 */

#include "BLE.h"
#include <config.h>
#include <Arduino.h>
#include "ble_state.h"

// ---------------------------------------------------------------------------
// Fragment constructor
// ---------------------------------------------------------------------------
Fragment::Fragment() : buffer(""), complete(false) {}

// ---------------------------------------------------------------------------
// Fragment::processData – accumulate one BLE write into the buffer
// ---------------------------------------------------------------------------
void Fragment::processData(const std::string &data)
{
    if (data.find("START") == 0)
    {
        // New fragmented transfer: clear any previous state and store the
        // payload that follows the 5-character "START" prefix.
        Serial.println("Start of data received.");
        buffer.clear();
        buffer  += data.substr(5);
        complete = false;
    }
    else if (data.find("END") != std::string::npos)
    {
        // End marker found: append everything before "END" and mark complete.
        buffer += data.substr(0, data.find("END"));
        Serial.println("End of data received.");
        Serial.println("Complete data: ");
        Serial.println(buffer.c_str());
        complete = true;
    }
    else
    {
        // Middle chunk: append verbatim
        buffer += data;
    }
}

// ---------------------------------------------------------------------------
// Fragment::getBuffer – return the accumulated payload
// ---------------------------------------------------------------------------
std::string Fragment::getBuffer() const
{
    return buffer;
}

// ---------------------------------------------------------------------------
// Fragment::isComplete – true when the END marker has been received
// ---------------------------------------------------------------------------
bool Fragment::isComplete() const
{
    return complete;
}

// ---------------------------------------------------------------------------
// Fragment::clear – reset for the next transfer
// ---------------------------------------------------------------------------
void Fragment::clear()
{
    buffer.clear();
    complete = false;
}

// ---------------------------------------------------------------------------
// Fragment::sendFragmentedData – send a large string as START/chunk/END
// ---------------------------------------------------------------------------
void Fragment::sendFragmentedData(BLECharacteristic *characteristic, const std::string &data)
{
    const size_t chunkSize = 100; // Max bytes per BLE notification
    size_t       dataLength = data.length();
    size_t       offset     = 0;

    // Signal the start of a fragmented transfer
    characteristic->setValue("START");
    characteristic->notify();
    delay(50); // Allow the app to process the START marker

    // Send data in chunks
    while (offset < dataLength)
    {
        size_t end = offset + chunkSize;
        if (end > dataLength) end = dataLength;

        std::string chunk = data.substr(offset, end - offset);
        characteristic->setValue(chunk);
        characteristic->notify();
        offset = end;
        delay(50); // Pacing: prevent overrun in the BLE notification queue
    }

    // Signal the end of the fragmented transfer
    characteristic->setValue("END");
    characteristic->notify();
    delay(50);
}

// ---------------------------------------------------------------------------
// resetPlaylist – wipe playlist NVS data and reset in-memory variables
// ---------------------------------------------------------------------------
void resetPlaylist()
{
    // Restore default playlist settings
    preferences.begin("PL_setting");
    preferences.putString("playlist",      "1-2-3");
    preferences.putString("playlistDB",    "");
    preferences.putInt("pgnIndex",         201); // User PGN records start at index 201
    preferences.putInt("pgnCount",         0);
    preferences.putInt("playlistCount",    0);
    preferences.end();

    // Wipe all stored metadata records
    preferences.begin("PL_metadata");
    preferences.clear();
    preferences.end();

    // Wipe all stored PGN text records
    preferences.begin("PL_pgn");
    preferences.clear();
    preferences.end();

    // Reset the in-memory mirrors
    playlist   = "";
    playlistDB = "";
}
