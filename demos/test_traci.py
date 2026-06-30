import traci

cmd = [
    "/Users/sachindev/sumo-env/bin/sumo",
    "-c",
    "./demos/demo_simulation_models/example_intersection_management/Configuration.sumocfg"
]

traci.start(cmd)
print("Connected to SUMO!")

for _ in range(10):
    traci.simulationStep()

traci.close()
print("Done")