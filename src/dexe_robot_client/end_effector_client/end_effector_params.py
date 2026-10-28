'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

from dataclasses import dataclass
from typing import Tuple

from end_effector_interfaces.msg import EEJointControlMode

from dexe_rp.utils.type import EndEffectorType as DexeRP_EEType


@dataclass(frozen=True, slots=True)
class Limit:
    lb: Tuple[float, ...]
    ub: Tuple[float, ...]


@dataclass(frozen=True, slots=True)
class EndEffectorParams:
    dof: int
    joint_names: Tuple[str, ...]
    supported_modes: Tuple[EEJointControlMode, ...]
    position: Limit
    velocity: Limit
    current: Limit
    force: Limit


class EndEffectorFactory:
    _registry = {}

    @classmethod
    def register(cls, ee_type: DexeRP_EEType):

        def wrapper(fn):
            cls._registry[ee_type] = fn
            return fn

        return wrapper

    @classmethod
    def create(cls, ee_type: DexeRP_EEType) -> EndEffectorParams:
        if ee_type not in cls._registry:
            raise ValueError(f"Unsupport EndEffectorType: {ee_type}")
        return cls._registry[ee_type]()

@EndEffectorFactory.register(DexeRP_EEType.Dual_DH_PGC_50_35)
def _():
    return EndEffectorParams(
        dof=2,
        joint_names=("FINGER1", "FINGER2"),
        supported_modes=(EEJointControlMode.POSITION,),
        position=Limit((0,)*2, (100,)*2),
        velocity=Limit((-100,)*2, (100,)*2),
        current=Limit((-100,)*2, (100,)*2),
        force=Limit((-100,)*2, (100,)*2),
    )

@EndEffectorFactory.register(DexeRP_EEType.DH_PGC_140_50)
def _():
    return EndEffectorParams(
        dof=1,
        joint_names=("FINGER1",),
        supported_modes=(EEJointControlMode.POSITION,),
        position=Limit((0,), (100,)),
        velocity=Limit((-100,), (100,)),
        current=Limit((-100,), (100,)),
        force=Limit((-100,), (100,)),
    )


@EndEffectorFactory.register(DexeRP_EEType.DH_AG_160_95)
def _():
    return EndEffectorParams(
        dof=1,
        joint_names=("FINGER1",),
        supported_modes=(EEJointControlMode.POSITION,),
        position=Limit((0,), (100,)),
        velocity=Limit((-100,), (100,)),
        current=Limit((-100,), (100,)),
        force=Limit((-100,), (100,)),
    )


@EndEffectorFactory.register(DexeRP_EEType.BrainCo_Revo1_R)
def _():
    return EndEffectorParams(
        dof=6,
        joint_names=("T_CMC_YAW", "T_MCP", "IF_MCP_PITCH", "MF_MCP_PITCH",
                     "RF_MCP_PITCH", "LF_MCP_PITCH"),
        supported_modes=(
            EEJointControlMode.POSITION,
            EEJointControlMode.VELOCITY,
            EEJointControlMode.CURRENT,
        ),
        position=Limit((0,) * 6, (100,) * 6),
        velocity=Limit((-100,) * 6, (100,) * 6),
        current=Limit((-100,) * 6, (100,) * 6),
        force=Limit((-100,) * 6, (100,) * 6),
    )


@EndEffectorFactory.register(DexeRP_EEType.Linker_L6)
def _():
    return EndEffectorParams(
        dof=6,
        joint_names=("T_CMC_YAW", "T_MCP", "IF_MCP_PITCH", "MF_MCP_PITCH",
                     "RF_MCP_PITCH", "LF_MCP_PITCH"),
        supported_modes=(EEJointControlMode.POSITION,),
        position=Limit((0,) * 6, (100,) * 6),
        velocity=Limit((-100,) * 6, (100,) * 6),
        current=Limit((-100,) * 6, (100,) * 6),
        force=Limit((-100,) * 6, (100,) * 6),
    )


@EndEffectorFactory.register(DexeRP_EEType.Linker_L20)
def _():
    return EndEffectorParams(
        dof=16,
        joint_names=("T_CMC_ROLL", "T_CMC_YAW", "T_CMC_PITCH", "T_MCP",
                     "IF_MCP_ROLL", "IF_MCP_PITCH", "IF_PIP", "MF_MCP_ROLL",
                     "MF_MCP_PITCH", "MF_PIP", "RF_MCP_ROLL", "RF_MCP_PITCH",
                     "RF_PIP", "LF_MCP_ROLL", "LF_MCP_PITCH", "LF_PIP"),
        supported_modes=(EEJointControlMode.POSITION,),
        position=Limit(
            (0, 0, 0, 0, -100, 0, 0, -100, 0, 0, -100, 0, 0, -100, 0, 0),
            (100,) * 16),
        velocity=Limit((-100,) * 16, (100,) * 16),
        current=Limit((-100,) * 16, (100,) * 16),
        force=Limit((-100,) * 16, (100,) * 16),
    )


@EndEffectorFactory.register(DexeRP_EEType.PaXini_Dex_H13)
def _():
    return EndEffectorParams(
        dof=13,
        joint_names=("T_CMC_ROLL", "T_CMC_YAW", "T_MCP", "T_IP", "IF_MCP_ROLL",
                     "IF_MCP_PITCH", "IF_PIP", "MF_MCP_ROLL", "MF_MCP_PITCH",
                     "MF_PIP", "RF_MCP_ROLL", "RF_MCP_PITCH", "RF_PIP"),
        supported_modes=(EEJointControlMode.POSITION,),
        position=Limit((0, 0, 0, 0, -100, 0, 0, -100, 0, 0, -100, 0, 0),
                       (100,) * 13),
        velocity=Limit((-100,) * 13, (100,) * 13),
        current=Limit((-100,) * 13, (100,) * 13),
        force=Limit((-100,) * 13, (100,) * 13),
    )
