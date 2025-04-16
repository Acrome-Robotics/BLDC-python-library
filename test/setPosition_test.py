from joint.joint import Master, Joint, Index
import math
import time

'''
port = "COM6"
m = Master(port)

m.attach(Joint(0))

m.set_variables(0, [[Index.TorqueEn, 1]])
'''

a = 10

def busy_wait_delay(seconds):
    end_time = time.time() + seconds
    while time.time() < end_time:
        pass

def x_up(t, initial_pos, initial_speed):
    pos = initial_pos + ((a * (t**2)) / 2) + initial_speed * t
    return pos

def x_down(t, initial_pos, initial_speed):
    pos = initial_pos - (a * (t**2)) / 2 - initial_speed * t
    return pos

def x_const_speed(t, initial_pos, constant_speed):
    pos = initial_pos + (constant_speed * t)
    return pos

# numbers in ms.
count = 0
start = time.time()
print("-------------------YUKSELIYOR-------------------")
for i in range(0, 1000, 5):
    # m.set_variables(0, [[Index.GoalPosition, x_up(i*0.001, 0, 0)]])
    busy_wait_delay(0.005)
    count += 1
end = time.time()
print("gecen sure ", end - start)
print("sayi : ", count)

"""
current_pos = x_up(i*0.001, 0, 0)

current_speed = i * 0.001 * a
print("current pos: ", current_pos)
print("speed: ", current_speed)
print("-------------------SABIT HIZDA-------------------")
for j in range(0, 3000, 5):
    m.set_variables(0, [[Index.GoalPosition, x_const_speed(j*0.001, current_pos, current_speed)]])
    print(x_const_speed(j*0.001, current_pos, current_speed))
    time.sleep(0.005)

current_pos = x_const_speed(j*0.001, current_pos, current_speed)
print("current pos: ", current_pos)
print("speed: ", current_speed)
print("-------------------DUSUYOR-------------------")
for k in range(0, 3000, 5):
    # m.set_variables(0, [[Index.GoalPosition, x_down(k*0.001, current_pos, current_speed)]])
    # print(x_down(k*0.001, current_pos, current_speed))
    time.sleep(0.005)

current_pos = x_down(k*0.001, current_pos, current_speed)
current_speed = current_speed - (a * (k**2)) / 2

print("current pos: ", current_pos)
print("speed: ", current_speed)
"""
