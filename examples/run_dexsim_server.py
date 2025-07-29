# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

from dexe_robot_client.sim_server import DexsimServer

if __name__ == "__main__":
    dexsim_server = DexsimServer()
    dexsim_server.run()