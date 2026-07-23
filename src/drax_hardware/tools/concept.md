


SocketCanBus

This is the lowest layer.

It doesn't know anything about motors.

It doesn't know anything about CANopen.

It only knows

"Send these 8 bytes."

or

"Receive 8 bytes."

That's all.

Suppose we want to send

ID = 0x601

Data

23 FF 60 00 55 00 00 00

SocketCanBus never asks

Is this RPM?

No.

It only says

Here are your bytes.

Exactly like write() in Linux.

Think about UART.

Suppose you do

Serial.write(buffer);

Does Serial know you are sending velocity?

No.

CAN is identical.

SocketCanBus responsibilities

Only these.

Open socket

↓

Send frame

↓

Receive frame

↓

Close socket

Nothing else.

CanOpenNode

Now comes the second layer.

This class DOES understand CANopen.

It knows things like

0x6060

0x6040

0x6063

0x60FF

These are CANopen object dictionary indexes.

SocketCanBus has absolutely no idea what these mean.

Example

Instead of writing

send()

23 FF 60 00

we'll simply call

motor.write(0x60FF, rpm)

The CanOpenNode converts it into bytes.

Motor class

Now we make life even easier.

Instead of

node.write(0x60FF,...)

we'll simply write

motor.setRPM(15);

Motor knows

RPM should go to object 0x60FF.

DraxHardware

This is what ros2_control talks to.

It doesn't know CAN.

It doesn't know SDO.

It doesn't know object dictionary.

It simply says

write()

and

read()

Everything else happens underneath.

See how nice the architecture becomes?

ros2_control

↓

set velocity

↓

Motor.setRPM()

↓

CanOpenNode.write()

↓

SocketCanBus.send()

↓

Motor

Exactly the same on the way back

Motor

↓

SocketCanBus.recv()

↓

CanOpenNode.read()

↓

Motor.readEncoder()

↓

ros2_control