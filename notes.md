```cpp
/*
    This project should be able to count the number of cans in a set.
    Thinking of generating some sort of matrix grid and showing available sections for the cans

    there will be one sensor in the back of the can section to count how mamy cans are in
        (less distance to first can meams more cans in)

    the sensors will all work together to return the total count of each can in each section in unparsed json.

    data looks like: 
    {
        "section_1": 5, // section_1 can count is 5
        "section_2": 3,
        "section_3": 7
    }

    This should be returned every second depending on the hardware capabilities.

    it uses a VL53L0X time-of-flight distance sensor to measure the distance to the first can in each section.
    it uses the arduino uno Q board for interfacing with the sensors and handling serial communication with the frontend.

    There is a limitation however, the very last can in the section (meaning only 1 can left) may not be accurately detected by the sensor.
    because it fits a small space. i will try however to get the most accurate reading possible by possibly measuring the distance past the can.

    the can count will be provided to the frontend. but only when the frontend requests the information.
    so this code must listen to the frontned using usb serial comms when it sends a request for the can count.
    
    commands:
    LISTEN_CAN_COUNT: command sent by the frontend to start listening for can count updates irt
    STOP: command sent by the frontend to stop listening for can count updates

    Ill probably have to turn the thing off by sending stop to it
*/
```