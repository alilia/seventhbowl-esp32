// ============================================================================
//  Seventh Bowl — shared WiFi + network settings for every ESP32 module
//
//  THIS IS THE TEMPLATE IN THE REPO — keep the placeholders here.
//  Your real copy lives OUTSIDE the repo, in the Arduino libraries folder:
//    <sketchbook>\libraries\SeventhBowlSecrets\SeventhBowlSecrets.h
//  (sketchbook = Arduino IDE > File > Preferences > Sketchbook location).
//  Copy this whole SeventhBowlSecrets folder there once and fill it in.
//  New network = edit that one file, then re-upload each module.
//
//  Each module sets only its own fixed address (MODULE_IP_LAST) and whether
//  it uses it (USE_STATIC_IP) in its own config.
// ============================================================================
#pragma once
#include <Arduino.h>

// ---- WiFi (2.4 GHz only — ESP32 boards can't join 5 GHz networks) ----------
#define WIFI_SSID          "your-network"
#define WIFI_PASSWORD      "your-password"

// ---- Network ---------------------------------------------------------------
// First three numbers of the network's addresses (laptop: ipconfig -> IPv4).
// Modules add their own last number to this for a fixed IP.
#define NET_PREFIX         192, 168, 1
static IPAddress GATEWAY        (NET_PREFIX, 1);
static IPAddress SUBNET         (255, 255, 255, 0);

// ---- Where OSC goes --------------------------------------------------------
// 255 = broadcast to every computer on the network; or the laptop's own last
// number (e.g. 70) to send only to it.
static IPAddress OSC_TARGET_IP  (NET_PREFIX, 255);
