#pragma once

// Starts the UART link to the sensor ESP and spins up the receive task
// on core 0, out of the way of the drive loop. Call once from setup().
void uartLinkBegin();