Idea: a motion detector sensor should turn on an LED light smoothly and should consume very low power when idle. The PIR sensors as such are very low power, but when keeping them connected at mains, the supply losses are orders of magnitude higher than the power needed to keep the PIR active. 
So a battery is used to power an AVR microcontroller and the PIR, and when the motion event occurs, the AVR will nicely PWM the light on. 
When idle, the AVR will wake up every couple seconds and check the battery voltage. If below threshold, it will turn the mains on and charge the batt. 
