The project involves simulating a digital twin to predict the change in room temperature with a change in the PID controller which controls a heater

The room temperature changes based on the amount of heat energy in and the amount of heat energy let out. so the formula would be c*(dT/dt)=(dq_in/dt)-(dq_out/dt). Where C is the amount of energy needed to change the room temperature. A concrete room has a higher c value and a room made up of tin sheets have lower c value. This formula is derived from the first law of thermodynamics.

In real world scenario, the amount of heat energy let out depends on the internal temperature (T_room) and the external temperature (T_amb). The formula for the same is (dq_out/dt)=k*(T_room-T_amb). Where K is the loss coefficient, the value of k is larger for a poor insulator indicating a rapid heat escape and for a smaller value of k for good insulators indicating slow heat escape.

To control the amount of heat let inside the room, (dq_in/dt)=P_max*u, where P_max is the max power output of the heater and u is the control signal which controls the PID (Between 0 to 1). 

To predict the next temperature value, we use the formula T_next=T_current+(delta_t/c)[(P_max*u)-(k*(T_room-T_amb))]. 

The reason for using delta_t in the above equation is that in embedded systems the values are calculated in discrete time intervals of ticks. Thus we convert the dT/dt into delta_T/delta_t where delta_T is the T_next-T_current 


