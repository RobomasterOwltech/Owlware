# Owlware = Owltech Middleware

This repo contains all the necessary low level-ish software to communicate the STM32 HAL and application code for FreeRTOS, controlling Robomaster and some GoBilda hardware.

## When to use C/C++ for the class I want to create?

Try to use C++ for all you can. The only reason why we use C, is for the compatibility of the HAL interface. 

Tho, we know some functions might not be so transparent to convert into classes, then it is valid to use functional programming in C.


 
