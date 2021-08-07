from environs import Env
import lgsvl



def main():
    env = Env()

    # LGSVL__SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "")
    LGSVL__SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "172.17.0.1")
    LGSVL__SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
    LGSVL__AUTOPILOT_0_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
    LGSVL__AUTOPILOT_0_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)

    sim = lgsvl.Simulator(LGSVL__SIMULATOR_HOST, LGSVL__SIMULATOR_PORT)
    sim.load(scene="62765742-57bf-4ccd-85e5-db8295d34ead", seed = 650387)
    # sim.load(env.str("LGSVL__MAP"))

    egoState = lgsvl.AgentState()
    ego = sim.add_agent(env.str("LGSVL__VEHICLE_0"), lgsvl.AgentType.EGO, egoState)

    sim.run(30.0)


if __name__ == '__main__':
    main()
