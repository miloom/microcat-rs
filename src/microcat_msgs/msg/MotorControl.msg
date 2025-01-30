uint8 FRONT_LEFT  = 0
uint8 FRONT_RIGHT = 1
uint8 REAR_LEFT   = 2
uint8 REAR_RIGHT  = 3

uint8 location  # Use one of the defined constants
float32 frequency # The frequency that the motor will oscillate with around the target position
int32 position # Target position for the motor in degrees
uint32 amplitude # Amplitude that the motor will use when oscillating around the target position
