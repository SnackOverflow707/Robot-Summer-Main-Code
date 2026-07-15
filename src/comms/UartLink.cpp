#include "comms/UartLink.h"
#include "comms/uart_protocol.h"
#include "config/pins.h"

static HardwareSerial CommLink(UART_NUM);

static uint8_t lastSeq = 0;
static bool firstPacket = true;
static uint32_t droppedPackets = 0; // print this if you suspect link quality issues

// TODO: once StateMachine/RobotState are built out, replace this print with
// a real call into the state machine. Keeping it as a Serial print for now
// so the link can be verified independently of that work.
static void handleIRDetection(uint16_t freq1Mag, uint16_t freq2Mag, uint8_t detectedMask) {
    Serial.printf("IR packet: f1=%u f2=%u mask=%u dropped=%lu\n",
                  freq1Mag, freq2Mag, detectedMask, droppedPackets);
}

// Byte-at-a-time state machine - resyncs on the next 0xAA if anything
// gets corrupted, rather than getting stuck waiting on bytes that never come.
static bool readIRPacket(IRPacket& out) {
    static uint8_t buf[sizeof(IRPacket)];
    static size_t idx = 0;

    while (CommLink.available()) {
        uint8_t b = CommLink.read();

        if (idx == 0 && b != PKT_START_BYTE) {
            continue; // still scanning for sync byte
        }
        buf[idx++] = b;

        if (idx == sizeof(IRPacket)) {
            idx = 0;
            memcpy(&out, buf, sizeof(IRPacket));
            if (irPacketChecksum(out) != out.checksum) {
                continue; // corrupted packet, drop it and keep scanning
            }
            if (!firstPacket && (uint8_t)(out.seq - lastSeq) != 1) {
                droppedPackets++;
            }
            lastSeq = out.seq;
            firstPacket = false;
            return true;
        }
    }
    return false;
}

static void uartReceiveTask(void* pvParameters) {
    CommLink.begin(UART_BAUD, SERIAL_8N1, SENSOR_UART_RX, SENSOR_UART_TX);
    IRPacket pkt;
    for (;;) {
        if (readIRPacket(pkt)) {
            handleIRDetection(pkt.freq1_mag, pkt.freq2_mag, pkt.detected_mask);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void uartLinkBegin() {
    xTaskCreatePinnedToCore(uartReceiveTask, "uartRx", 4096, NULL, 1, NULL, 0);
}
