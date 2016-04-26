import Motor_Comms_2
s = Motor_Comms_2.MoveSpatula()
command = s.advance(2)
s.reset_spatula(command)


