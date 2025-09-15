# Hardware Definition
This project use ESP32S3 micro controller with following parts:

## Silicon Microphone
An I2S PDM microphone.
SEL on GPIO6
CLK on GPIO14
DATA on GPIO13

## LCD
1.14inch 65K colors LCD with 135x240 pixels
DC on GPIO8
CS on GPIO10
CLK on GPIO12
MOSI on GPIO11
RST on GPIO9
BL on GPIO7

## TF Card
SCK on GPIO36
SDIO CMD/MOSI on GPIO35
SDIO D0/MISO on GPIO37
SDIO D1 on GPIO33
SDIO D2 on GPIO38
SDIO D3/CS on GPIO34

## W25Q128 Flash
CS on SPICS0
DO on SPIQ
WP on SPIWP
HOLD on SPIHD
CLK on SPICLK
DI on SPID

# References
Always use context7 when I need code generation, setup or configuration steps, or
library/API documentation. This means you should automatically use the Context7 MCP
tools to resolve library id and get library docs without me having to explicitly ask.

For ESP32 or ESP-IDF related knowledge, use library /espressif/esp-idf for API and docs.

# Build
Always use idf.py to build this project, DO NOT use cmake directly.
