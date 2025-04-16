from joint.joint import Joint, Master, Index
from joint.osModules import USB_serial_port

import time

# init Joint (BLDC) cihazları
BATCH_ID = 0xFF
SERIAL_PORT = USB_serial_port()
print(f"Using serial port: {SERIAL_PORT}")

m = Master(SERIAL_PORT, baudrate=115200)
# Cihazları ata
m.attach(Joint(BATCH_ID))
m.attach(Joint(1))

# İlk torque enable komutu
m.set_variables(1, [(Index.TorqueEn, 1)])

while True:
    pos = int(input("Enter position: "))
    time_traj = 10

    # Senkron pozisyon komutu: 3 cihaz için [123, pos, 123] pozisyon, [1, time_traj, 5] zaman
    m.set_variable_combined(
        [Index.GoalPosition, Index.Trajectory_time],
        [
            [123, pos, 123],
            [1, time_traj, 5]
        ],
        device_size=3
    )
    time.sleep(0.005)

    # Cihaz 1’in o anki pozisyonunu oku
    values = m.get_variables(1, [Index.CurrentPosition])
    current_pos = values[0]

    # Hedefe yaklaşana dek dön
    while abs(current_pos - pos) > 10:
        values = m.get_variables(1, [Index.CurrentPosition])
        current_pos = values[0]
        print(f"Current pos: {current_pos}, target: {pos}")
        time.sleep(0.01)
