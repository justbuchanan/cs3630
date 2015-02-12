import time

def test_cmd(cmd):
    start = time.time()
    s.setMotors(cmd[0], cmd[1])
    while (time.time() - start < cmd[2]):
        time.sleep(0.1)
    s.setMotors(0,0)
