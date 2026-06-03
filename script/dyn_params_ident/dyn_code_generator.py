import warnings
warnings.filterwarnings("ignore", category=SyntaxWarning)

import sympy
import sympybotics
import os
from typing import List, Tuple
from sympybotics.symcode import code_to_func, Subexprs
import re
from urdf2dh import get_dh_params_for_arm, get_robot_info

# ============================================================================
# Robot Dynamics Generator Class
# ============================================================================

class RobotUnifiedDynamicsGenerator:
    def __init__(self, arm_name: str, dh_params: List[Tuple], robot_type: str, version: str):
        self.arm_name = arm_name
        self.dh_params = dh_params
        self.robot_type = robot_type
        self.version = version
        
        # 建立输出目录结构: generated/{robot_type}_{version_prefix}/
        version_prefix = self.version[:3]
        self.full_version_key = f"{self.robot_type}_{version_prefix}"
        self.output_dir = os.path.join(
            os.path.dirname(__file__), 
            "generated", 
            self.full_version_key
        )
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.rbt, self.rbtdef = self.create_robot_model()
        self.generate_all()

    # -------------------------------------------------
    def create_robot_model(self):
        rbtdef = sympybotics.RobotDef(
            f"{self.robot_type}_{self.version}_{self.arm_name}",
            self.dh_params,
            dh_convention="standard",
        )

        rbtdef.frictionmodel = {"Coulomb", "viscous", "offset"}
        rbtdef.driveinertiamodel = "simplified"
        rbtdef.gravityacc = sympy.Matrix([-9.81, 0.0, 0.0])

        rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)
        rbt.calc_base_parms()
        return rbt, rbtdef

    # -------------------------------------------------
    def generate_all(self):
        # 预先清理或准备合并后的文件内容（如果需要的话，或者直接由 generate 方法追加）
        self.generate_M_term_code()
        self.generate_inv_dyn_code()

    # -------------------------------------------------

    def generate_M_term_code(self):
        import re
        from sympybotics.symcode import code_to_func
        rbt_no_gravity = sympybotics.RobotDef(
            f"{self.robot_type}_{self.version}_{self.arm_name}",
            self.dh_params,
            dh_convention="standard",
        )
        rbt_no_gravity.frictionmodel = {"Coulomb", "viscous", "offset"}
        rbt_no_gravity.driveinertiamodel = "simplified"
        rbt_no_gravity.gravityacc = (sympy.Matrix([0.0, 0.0, 0.0]))
        rbt_sym = sympybotics.RobotDynCode(rbt_no_gravity, verbose=True)

        # 构造 tau = H(g) * Pb * dyn_base_parameters（7x1）
        pb_syms = sympy.symbols(f'pb0:{len(self.rbt.dyn.baseparms)}')  # 仅用于构造表达式
        pb = sympy.Matrix(pb_syms)
        tau_expr = rbt_sym.dyn.H * self.rbt.dyn.Pb * pb  # 7x1

        # 针对 sympybotics 预计算中内置的 tmp 子表达式进行拓扑递归展开，彻底还原为无 tmp 的基础状态
        symbol_to_expr = {sym: expr for expr, sym in rbt_sym._H_se.items()}
        def expand_expr(expr, sym_to_expr, cache):
            if expr in cache:
                return cache[expr]
            if expr.is_Atom:
                if expr in sym_to_expr:
                    res = expand_expr(sym_to_expr[expr], sym_to_expr, cache)
                    cache[expr] = res
                    return res
                return expr
            args = [expand_expr(arg, sym_to_expr, cache) for arg in expr.args]
            res = type(expr)(*args)
            cache[expr] = res
            return res

        cache = {}
        expanded_elements = []
        for i in range(len(tau_expr)):
            expanded_elements.append(expand_expr(tau_expr[i], symbol_to_expr, cache))
        tau_expr_expanded = sympy.Matrix(expanded_elements)

        # 在 SymPy 符号代数层面将关节速度 dq 替换为数常数 0，从而对多项式和公式在符号层面实现代数自动简化、消除所有零值乘积和加项
        dq_zero_map = {dq: 0 for dq in rbt_no_gravity.dq}
        tau_expr_simplified = tau_expr_expanded.subs(dq_zero_map)

        # 构造无 CSE（即完全平面展开、无 tmp 变量）的计算结构形式，保证生成代码直观可用
        tau_code = ([], [tau_expr_simplified])

        # 构造替换映射与函数签名（包含 dyn_base_params 与 gravity_acc）
        q_subs = {q: f'q[{i}]' for i, q in enumerate(rbt_no_gravity.q)}
        ddq_subs = {ddq: f'ddq[{i}]' for i, ddq in enumerate(rbt_no_gravity.ddq)}
        pb_subs = {sym: f'dyn_base_params[{i}]' for i, sym in enumerate(pb_syms)}
        subs = {**q_subs, **ddq_subs, **pb_subs}

        # 6) 生成 C++ 版本（用 C 代码风格，.cpp 可直接编译）
        func_name = f"{self.arm_name}_mass_term"
        func_src_c = code_to_func(
            'C', tau_code, 'M_term_out', func_name,
            ['q', 'ddq', 'dyn_base_params'], subs
        )
        
        # 增加命名空间
        ns = f"robot_server_lib::dynamics::{self.full_version_key}"
        func_src_c = f"namespace {ns} {{\n{func_src_c}\n}}"

        header_cpp = (
            '#include <math.h>\n'
            '#include <stddef.h>\n'
            '#ifndef DYN_SIGN_FUNC\n'
            '#define DYN_SIGN_FUNC\n'
            'static inline double sign(double x){ return (x>0) - (x<0); }\n'
            '#endif\n\n'
        )

        # 替换 ((x) > 0) - ((x) < 0) 为 sign(x)
        pattern = re.compile(r'\(\(([^()]+?)\) > 0\) - \(\(\1\) < 0\)')
        while True:
            new_func_src_c = pattern.sub(r'sign(\1)', func_src_c)
            if new_func_src_c == func_src_c:
                break
            func_src_c = new_func_src_c

        out_path = os.path.join(self.output_dir, 'mass_matrix.cpp')
        mode = 'a' if os.path.exists(out_path) else 'w'
        with open(out_path, mode) as f:
            if mode == 'w':
                f.write(header_cpp)
            f.write(func_src_c + "\n")

    def generate_inv_dyn_code(self):
        import re
        from sympybotics.symcode import code_to_func
        g0, g1, g2 = sympy.symbols('g0 g1 g2')
        # 创建一个带有符号重力加速度的模型
        rbtdef_sym = sympybotics.RobotDef(
            f"{self.robot_type}_{self.version}_{self.arm_name}",
            self.dh_params,
            dh_convention="standard",
        )
        rbtdef_sym.frictionmodel = {"Coulomb", "viscous", "offset"}
        rbtdef_sym.driveinertiamodel = "simplified"
        rbtdef_sym.gravityacc = sympy.Matrix([g0, g1, g2])
        rbt_sym = sympybotics.RobotDynCode(rbtdef_sym, verbose=True)

        # 构造 tau = H(g) * Pb * dyn_base_parameters（7x1）
        pb_syms = sympy.symbols(f'pb0:{len(self.rbt.dyn.baseparms)}')  # 仅用于构造表达式
        pb = sympy.Matrix(pb_syms)
        tau_expr = rbt_sym.dyn.H * self.rbt.dyn.Pb * pb  # 7x1

        # 复用 H 的子表达式，生成 code 结构
        T_se = sympybotics.symcode.Subexprs()
        T_se._subexp_iv = rbt_sym._H_se
        tau_code = T_se.get(tau_expr)

        # 构造替换映射与函数签名（包含 dyn_base_params 与 gravity_acc）
        q_subs = {q: f'q[{i}]' for i, q in enumerate(rbtdef_sym.q)}
        dq_subs = {dq: f'dq[{i}]' for i, dq in enumerate(rbtdef_sym.dq)}
        ddq_subs = {ddq: f'ddq[{i}]' for i, ddq in enumerate(rbtdef_sym.ddq)}
        pb_subs = {sym: f'dyn_base_params[{i}]' for i, sym in enumerate(pb_syms)}
        g_subs = {g0: 'gravity_acc[0]', g1: 'gravity_acc[1]', g2: 'gravity_acc[2]'}
        subs = {**q_subs, **dq_subs, **ddq_subs, **pb_subs, **g_subs}

        # 6) 生成 C++ 版本（用 C 代码风格，.cpp 可直接编译）
        func_name = f"{self.arm_name}_invdyn_tau"
        func_src_c = code_to_func(
            'C', tau_code, 'tau_out', func_name,
            ['q', 'dq', 'ddq', 'dyn_base_params', 'gravity_acc'], subs
        )
        
        # 增加命名空间
        ns = f"robot_server_lib::dynamics::{self.full_version_key}"
        func_src_c = f"namespace {ns} {{\n{func_src_c}\n}}"

        header_cpp = (
            '#include <math.h>\n'
            '#include <stddef.h>\n'
            '#ifndef DYN_SIGN_FUNC\n'
            '#define DYN_SIGN_FUNC\n'
            'static inline double sign(double x){ return (x>0) - (x<0); }\n'
            '#endif\n\n'
        )

        # 替换 ((x) > 0) - ((x) < 0) 为 sign(x)
        pattern = re.compile(r'\(\(([^()]+?)\) > 0\) - \(\(\1\) < 0\)')
        while True:
            new_func_src_c = pattern.sub(r'sign(\1)', func_src_c)
            if new_func_src_c == func_src_c:
                break
            func_src_c = new_func_src_c

        out_path = os.path.join(self.output_dir, 'invdyn_tau.cpp')
        mode = 'a' if os.path.exists(out_path) else 'w'
        with open(out_path, mode) as f:
            if mode == 'w':
                f.write(header_cpp)
            f.write(func_src_c + "\n")

if __name__ == "__main__":
    base_urdf_path, robot_type, version = get_robot_info()
    robot_type = robot_type if robot_type else "unknown"
    version = version if version else "v01"
    
    print(f"机器人构型: {robot_type}")
    print(f"机器人版本: {version}")

    for arm in ["left", "right"]:
        # 确定该手臂对应的 URDF 路径
        arm_filename = f"{arm}_arm.urdf"
        arm_urdf_path = os.path.join(os.path.dirname(base_urdf_path), arm_filename) if base_urdf_path else None
        actual_urdf = arm_urdf_path if arm_urdf_path and os.path.exists(arm_urdf_path) else base_urdf_path
        print(f"\n>> Generating dynamics for {arm.upper()} arm... 使用 URDF: {actual_urdf} ")
        try:
            dh_params = get_dh_params_for_arm(arm)
            print(f"DH parameters for {arm}:")
            for p in dh_params:
                print(f"  {p}")
            RobotUnifiedDynamicsGenerator(arm, dh_params, robot_type, version)
            print(f"<<< Finished {arm.upper()} arm.")
        except Exception as e:
            print(f"!!! Error generating {arm} arm: {e}")
            import traceback
            traceback.print_exc()
