Drone Command Protocol Specification

This document defines the ASCII HEX command protocol used between PC or Raspberry Pi and the STM32 flight processor. All values are sent over UART as ASCII characters, not binary bytes.

Packet Format

Every command frame follows this structure:

AA | CMD | SUB | PARAMS (optional) | FF


Each field consists of ASCII characters representing hexadecimal values.

Example:

AA0501FF


Sent over UART exactly as:

'A' 'A' '0' '5' '0' '1' 'F' 'F'

Top Level Commands

These commands appear in the CMD position.

CMD	Meaning
01	Calibration
02	Wait or Idle
05	Instruction Command
Instruction Subcommands

These appear only when CMD is 05.

SUB	Action
01	Takeoff
02	Land
03	Square flight pattern
04	Move North
05	Move East
06	Move South
07	Move West
08	Climb
09	Descend
0A	Return to Launch
0B	Hover
Packet Examples
Takeoff
AA0501FF

Land
AA0502FF

Fly square pattern
AA0503FF

Move North
AA0504FF

Move East
AA0505FF

Move South
AA0506FF

Move West
AA0507FF

Climb
AA0508FF

Descend
AA0509FF

Return to Launch
AA050AFF

Hover
AA050BFF

Optional Parameterized Format

Commands that require extra data can include ASCII HEX parameters:

AA | CMD | SUB | P1 | P2 | FF


Example: Move forward 5 meters at speed 2:

AA05040502FF
