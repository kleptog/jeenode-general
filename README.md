This is the JeeNode General, a single sketch for all your measurement needs!

This is a sketch for the [JeeNode] [1] which turns it into a generic very
configurable sensor.

The idea is that you can upload this sketch, configure the JeeNode using
the serial port and the measurements will be made and transmitted to the
base station with sufficient information that the base station will know
what to do with them.

One sketch to rule them all...

2013-05-11 <kleptog@svana.org> Martijn van Oosterhout
http://opensource.org/licenses/mit-license.php

[1]: http://jeelabs.com/products/jeenode "The JeeNode"

BASIC WORKINGS
--------------

Terms:

*  Device: Something like a Room Board, Compass Board or even a pair of
           resistors setup as a voltage divider. It is essentially a bit of
           code to manage the configuration and reading out of such a
           device.

           A device can do a number of measurements (maximum 4)

*  Plug:   An instance of a device on a particular port. It is configured
           with a port (where is the device) and a period (how ofter to
           measure).

           The maximum number of defined plugs is 8.

*  Measurement: a measurement is an integer combined with the units it
           represents and a scale. The units is usually an SI unit, like
           Volts or Celsuis, but Scalar (just a number) and Relative
           Humidity are also possibilities.

The choice to use actual units was made because there will hopefully be less
units than the number of devices people can make to read them.

The sketch maintains a configuration in the EEPROM memory (at offset 0x80)
which contains information about the current set of defined plugs.

When started the sketch will output a help screen and information about the
current stored configuration on the serial.  Since monitoring the serial
port requires the JeeNode to not be sleeping, the sketch will monitor the
serial port for one minute and if nothing it received in that time will go
into deep sleep.  If something is received the JeeNode will never go to
sleep so you can configure it.

The help screen is as follows:

    Commands:
      h                   Help
      a <port> <device>   Add device to port
      d <port> <device>   Remove device from port
      c <port>            Clear all devices from port
      l                   List defined devices
      p                   Print current configuration
      m                   Do test measurements
    RF12 configuration:
      b <band>            set band: 4 = 433, 8 = 868, 9 = 915
      g <group>           RF12 group id (1..255)
      n <node>            RF12 node id (A..Z or 1..26)

As you can see you can also configure the RF12 configuration from this
sketch.  The rest are commands to configure which plugs are connected.  You
can also run test measurements and see the current values.

The list of available devices is something like:

    Defined devices:
      0 LDR sensor, measures: Scalar,
      1 SHT11 sensor, measures: Relative humidity, Temperature,
      2 Voltage divider, measures: Voltage,
      3 Compass board, measures: Magnetic field, Magnetic field, Magnetic field,
      4 Pressure board, measures: Temperature, Pressure,

This just describes the devices this sketch knows about and what
measurements they can do.

Once a plug has been defined (using the add command), you can print the
current configuration with 'p' and you'll see something like:

    Current configuration:
      port 1 Compass board measuring: Magnetic field (1e-7 T) , Magnetic field (1e-7 T) , Magnetic field (1e-7 T) ,
      port 2 SHT11 sensor measuring: Relative humidity (1e0 %) , Temperature (1e-1 C) ,
      port 3 ADC measuring: Voltage (1e-2 V) ,
      port 4 Pressure board measuring: Temperature (1e-1 C) , Pressure (1e2 Pa) ,
      RF12:  C i3 g100 @ 868 MHz

Here you see for each port which device is defined and how the measurements
are represented.  So for example the pressure board returns the temperature
in tenths of degrees and the pressure in hectopascals.

With the above configuration measuring (with 'm') looks like:

    jn-g> m
    Doing measurements:
    1: -219, 341, 162,
    2: 44, 233,
    3: 2538,
    4: 222, 1013,

NETWORK PROTOCOL
----------------

The goal of a sensor is ofcourse to transmit its measurements to somewhere
central.  Because each sensor knows its own configuration best, it
broadcasts this to any central node.  This message describes the kinds of
measurements this node takes and the units.  It does *not* describe the
individual devices attached.  Any central node will thus be able to receive
the reading and plot them with sensible axes.

The configuration message also decribes the width of the measure, in bits
and this is used to encode the measurement itself efficiently.  This means
that the configuration message is needed to be able to decode the data
message.  So after start the configuration message is sent once a minute
five times and subsequently once an hour.  It is also sent on any
configuration change.

Data messages are sent ASAP after the measurement is taken, though it does
try to combine multiple measurements into a single message wherever
possible.

Currently configuration messages being with a 'C', the data messages with a
'D'. The network protocol is subject to change.

TODO
----

Configurable measurement periods (now everything is just 1 minute)

FUTURE IDEAS
------------

- Allow nodes to name themselves and transmit this as well, giving the
  central node more information to help users distinguish their nodes.

- Add encryption. Since messages now can contain lots of information about
  what is being measured and are decypherable for anyone, encryption by
  default seems a very good idea.  As a nice side-effect it prevents your
  nodes registering themselves with your neighbours.

- Enable/disable measurements, so you can ignore the temperature from the
  pressure plug for example.

- Add a numeric node ID, the seperate the transport addressing (groups/node
  IDs) from the JeeNode itself.  This would make it possible to make
  transparent proxies for far off nodes. You could also move nodes without
  losing information.

- Options to adjust the measurements on the JeeNode itself, like dividing by
  ten because you only want the temperature in whole degrees.  Thus you can
  reduce the number of bytes that need to be sent.

- Some measurements are counters over time. It would be nice to be able to
  have an efficient way of communicating this without having to worry about
  losing packets.

- A briq in HouseMon that takes all the information it receives and uses it
  in a sensible way.

- If the node had a concept of time then it would be able to timestamp
  messages as they are sent removing the need to send measurements as soon
  as they are made.  This would probably require piggybacking the time on
  the ACK.
