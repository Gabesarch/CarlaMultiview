from subprocess import Popen, PIPE
import shlex
import time

carla_sim = "../../../CarlaUE4.sh -carla-server -windows -ResX=100 -ResY=100 -benchmark"
carla_sim_args = shlex.split(carla_sim)

cnt = 0
for i in range(26):
    for j in range(15):
        p1 = Popen(carla_sim_args, stdout=PIPE, stderr=PIPE)
        time.sleep(4)
        print("Number of times carla simulator started: ", cnt)
        cnt+=1
        p2 = Popen(["python3.5","carla_unique_far_camera.py", str(i)], stdout=PIPE, stderr=PIPE)
        time.sleep(1)

        out, err = p2.communicate()
        print(err)
        
        print("Done with single iteration. Terminating everything")
        p1.terminate()
        # p2.terminate()
        time.sleep(2)
