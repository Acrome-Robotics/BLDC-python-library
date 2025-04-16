from joint.joint import Master, Joint, Index
import time

# Seri port tanımı (Windows için COM6)
port = "COM6"
m = Master(port, baudrate=115200)

# Cihazı ata ve torque’u aktif et
m.attach(Joint(0))
print("Max speed index:", Index.MaxSpeed_VELOCITY)

m.set_variables(0, [(Index.TorqueEn, 1)])

# Hız artırma döngüsü
for i in range(0, 200):
    speed = i * 300
    m.set_variables(0, [(Index.GoalSpeed, speed)])
    print("Setting speed to", speed)
    time.sleep(0.05)

time.sleep(10)

# Hızı azaltma döngüsü
for i in range(200, 0, -1):
    speed = i * 300
    m.set_variables(0, [(Index.GoalSpeed, speed)])
    print("Setting speed to", speed)
    time.sleep(0.05)

# Son olarak hızı sıfırla
m.set_variables(0, [(Index.GoalSpeed, 0)])
print("Motor stopped.")
