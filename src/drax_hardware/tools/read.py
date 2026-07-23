import can
import struct
import time

# -------------------------
# Configuration
# -------------------------
CHANNEL = "can0"
BITRATE = 1000000
NODE_ID = 1                    # Change to your motor ID

SDO_TX = 0x600 + NODE_ID       # Master -> Slave
SDO_RX = 0x580 + NODE_ID       # Slave -> Master

# -------------------------
# Open CAN Bus
# -------------------------
bus = can.interface.Bus(
    channel=CHANNEL,
    interface="socketcan"
)

def sdo_read(index, subindex=0):
    """
    Perform an SDO upload (read)
    """

    msg = can.Message(
        arbitration_id=SDO_TX,
        data=[
            0x40,
            index & 0xFF,
            (index >> 8) & 0xFF,
            subindex,
            0, 0, 0, 0
        ],
        is_extended_id=False
    )

    bus.send(msg)

    response = bus.recv(timeout=1.0)

    if response is None:
        raise TimeoutError("No SDO response received")

    if response.arbitration_id != SDO_RX:
        raise RuntimeError("Unexpected CAN ID")

    cmd = response.data[0]

    if cmd not in (0x43, 0x4B, 0x4F):
        raise RuntimeError(
            f"Unexpected SDO response: {response.data.hex()}"
        )

    value = struct.unpack("<i", bytes(response.data[4:8]))[0]
    return value


print("Reading encoder...")

try:
    while True:
        encoder = sdo_read(0x6063)

        print(f"Encoder = {encoder}")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped.")