from subprocess import Popen, PIPE
import shlex
import time

carla_sim = "../../CarlaUE4.sh -carla-server -windows -ResX=100 -ResY=100 -benchmark"
carla_sim_args = shlex.split(carla_sim)

cnt = 0
while True:
    p1 = Popen(carla_sim_args, stdout=PIPE, stderr=PIPE)
    time.sleep(2)
    print("Number of times carla simulator started: ", cnt)
    cnt+=1
    p2 = Popen(["python","carla_multiview.py"], stdout=PIPE, stderr=PIPE)
    time.sleep(2)
    p3 = Popen(["python","dynamic_weather.py"], stdout=PIPE, stderr=PIPE)

    # while True:
    #     print("Inside while loop")
    #     time.sleep(1)
    #     print("Getting output of p3")
    #     output = p3.stdout.readline()
    #     print(output.strip())
    #     print("Got output of p3")
    #     # print("Getting error of p3")
    #     # output = p3.stderr.readline()
    #     # print(output.strip())
    #     # print("Getting output of p2")
    #     # output = p2.stdout.readline()
    #     # print("Got output of p2")
    #     # print(output.strip())

    out, err = p2.communicate()
    print(err)
    
    print("Done with single iteration. Terminating everything")
    p1.terminate()
    # p2.terminate()
    p3.terminate()
    time.sleep(2)
